// ============================================================================
// axilite_to_reg.sv
// AXI4-Lite slave → simple register request/response interface
//
// Purpose: sits between the AXI-Lite crossbar and plic_regs.sv.
// The PULP register interface (reg_req_t / reg_rsp_t) is much simpler than
// AXI — it has no separate read/write channels, just a single valid/ready
// handshake.  This bridge serialises AXI write (AW+W) and read (AR)
// transactions into that interface one at a time.
//
// Connects to:
//   IN  — xbar_mst_req[N] / xbar_mst_resp[N]  (AXI4-Lite structs)
//   OUT — reg_req_o / reg_rsp_i                (plic_regs.sv ports)
//
// Usage in soc_top.sv (PERIPH_ADD_4 section):
//
//   axilite_to_reg #(
//     .axi_req_t  (axi_req_t),
//     .axi_resp_t (axi_resp_t),
//     .reg_req_t  (plic_reg_req_t),
//     .reg_rsp_t  (plic_reg_rsp_t)
//   ) u_plic_bridge (
//     .clk_i      (clk_i),
//     .rst_ni     (rst_ni),
//     .axi_req_i  (xbar_mst_req[N]),
//     .axi_resp_o (xbar_mst_resp[N]),
//     .reg_req_o  (plic_reg_req),
//     .reg_rsp_i  (plic_reg_rsp)
//   );
//
// ============================================================================

module axilite_to_reg #(
  parameter type axi_req_t  = logic,   // AXI4-Lite request  struct (from soc_top)
  parameter type axi_resp_t = logic,   // AXI4-Lite response struct (from soc_top)
  parameter type reg_req_t  = logic,   // Register request  struct (from plic_reg_pkg)
  parameter type reg_rsp_t  = logic    // Register response struct (from plic_reg_pkg)
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // AXI4-Lite slave (connected to xbar master port)
  input  axi_req_t  axi_req_i,
  output axi_resp_t axi_resp_o,

  // Register interface master (connected to plic_regs)
  output reg_req_t  reg_req_o,
  input  reg_rsp_t  reg_rsp_i
);

  // --------------------------------------------------------------------------
  // State machine
  // Transactions are serialised: one at a time (write beats read when both
  // arrive simultaneously — you can flip this if needed).
  // --------------------------------------------------------------------------
  typedef enum logic [2:0] {
    IDLE,         // waiting for AXI request
    WRITE_REG,    // issued write to register interface, waiting for ready
    WRITE_RESP,   // register done, waiting for AXI B-channel handshake
    READ_REG,     // issued read  to register interface, waiting for ready
    READ_RESP     // register done, waiting for AXI R-channel handshake
  } state_e;

  state_e state_q, state_d;

  // Capture AXI request fields so we can hold them stable during the
  // register transaction (AXI master may de-assert after handshake).
  logic [31:0] addr_q;
  logic [31:0] wdata_q;
  logic [ 3:0] wstrb_q;

  // Capture register response so we can hold it for the AXI response
  logic [31:0] rdata_q;
  logic        error_q;

  // --------------------------------------------------------------------------
  // State register + capture registers
  // --------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= IDLE;
      addr_q  <= '0;
      wdata_q <= '0;
      wstrb_q <= '0;
      rdata_q <= '0;
      error_q <= 1'b0;
    end else begin
      state_q <= state_d;

      // Latch write address/data when AXI handshake fires in IDLE
      if (state_q == IDLE && state_d == WRITE_REG) begin
        addr_q  <= axi_req_i.aw.addr;
        wdata_q <= axi_req_i.w.data;
        wstrb_q <= axi_req_i.w.strb;
      end

      // Latch read address when AXI AR handshake fires in IDLE
      if (state_q == IDLE && state_d == READ_REG) begin
        addr_q  <= axi_req_i.ar.addr;
      end

      // Latch register response when register interface completes
      if ((state_q == WRITE_REG || state_q == READ_REG) && reg_rsp_i.ready) begin
        rdata_q <= reg_rsp_i.rdata;
        error_q <= reg_rsp_i.error;
      end
    end
  end

  // --------------------------------------------------------------------------
  // Next-state logic + output driving
  // --------------------------------------------------------------------------
  always_comb begin
    // Safe defaults — no AXI handshake, no register request
    state_d    = state_q;
    axi_resp_o = '0;
    reg_req_o  = '0;

    case (state_q)

      // ----------------------------------------------------------------
      IDLE: begin
        // Prioritise writes: require both AW and W valid simultaneously.
        // (AXI4-Lite spec allows them to arrive independently, but Ibex's
        //  OBI→AXI bridge always sends them together, so this is fine.)
        if (axi_req_i.aw_valid && axi_req_i.w_valid) begin
          // Accept both channels in the same cycle
          axi_resp_o.aw_ready = 1'b1;
          axi_resp_o.w_ready  = 1'b1;
          state_d             = WRITE_REG;
        end else if (axi_req_i.ar_valid) begin
          axi_resp_o.ar_ready = 1'b1;
          state_d             = READ_REG;
        end
      end

      // ----------------------------------------------------------------
      // Issue write to register interface.
      // plic_regs.sv always has rsp.ready = 1 (combinational), so this
      // state typically lasts exactly one cycle.
      WRITE_REG: begin
        reg_req_o.valid = 1'b1;
        reg_req_o.write = 1'b1;
        reg_req_o.addr  = addr_q;
        reg_req_o.wdata = wdata_q;
        // wstrb is not part of plic_regs' interface but include for
        // future peripherals that do use it
        if (reg_rsp_i.ready)
          state_d = WRITE_RESP;
      end

      // ----------------------------------------------------------------
      // Send AXI write response (B channel)
      WRITE_RESP: begin
        axi_resp_o.b_valid = 1'b1;
        axi_resp_o.b.resp  = error_q ? 2'b10 : 2'b00;  // SLVERR or OKAY
        if (axi_req_i.b_ready)
          state_d = IDLE;
      end

      // ----------------------------------------------------------------
      // Issue read to register interface.
      READ_REG: begin
        reg_req_o.valid = 1'b1;
        reg_req_o.write = 1'b0;
        reg_req_o.addr  = addr_q;
        if (reg_rsp_i.ready)
          state_d = READ_RESP;
      end

      // ----------------------------------------------------------------
      // Send AXI read response (R channel)
      READ_RESP: begin
        axi_resp_o.r_valid = 1'b1;
        axi_resp_o.r.data  = rdata_q;
        axi_resp_o.r.resp  = error_q ? 2'b10 : 2'b00;
        if (axi_req_i.r_ready)
          state_d = IDLE;
      end

      default: state_d = IDLE;

    endcase
  end

endmodule