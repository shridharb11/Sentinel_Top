module ibex_to_axilite #(
  parameter int AW = 32,
  parameter int DW = 32
)(
  input  logic          clk_i,
  input  logic          rst_ni,

  // Ibex data port
  input  logic          data_req_i,
  output logic          data_gnt_o,
  output logic          data_rvalid_o,
  input  logic [AW-1:0] data_addr_i,
  input  logic          data_we_i,
  input  logic [3:0]    data_be_i,
  input  logic [DW-1:0] data_wdata_i,
  output logic [DW-1:0] data_rdata_o,
  output logic          data_err_o,

  // AXI-Lite master port
  output logic [AW-1:0] m_axi_awaddr,
  output logic          m_axi_awvalid,
  input  logic          m_axi_awready,
  output logic [DW-1:0] m_axi_wdata,
  output logic [3:0]    m_axi_wstrb,
  output logic          m_axi_wvalid,
  input  logic          m_axi_wready,
  input  logic [1:0]    m_axi_bresp,
  input  logic          m_axi_bvalid,
  output logic          m_axi_bready,
  output logic [AW-1:0] m_axi_araddr,
  output logic          m_axi_arvalid,
  input  logic          m_axi_arready,
  input  logic [DW-1:0] m_axi_rdata,
  input  logic [1:0]    m_axi_rresp,
  input  logic          m_axi_rvalid,
  output logic          m_axi_rready
);

  typedef enum logic [2:0] {
    IDLE,
    WRITE_ADDR,
    WRITE_WAIT,
    READ_ADDR,
    READ_WAIT
  } state_e;

  state_e state_q, state_d;

  // Registered request
  logic [AW-1:0] addr_q;
  logic [DW-1:0] wdata_q;
  logic [3:0]    be_q;

  logic aw_done_q, w_done_q;

  // ---------------- FSM ----------------
  always_comb begin
    state_d = state_q;
    case (state_q)
      IDLE:
        if (data_req_i)
          state_d = data_we_i ? WRITE_ADDR : READ_ADDR;

      WRITE_ADDR:
        if ((m_axi_awready || aw_done_q) &&
            (m_axi_wready  || w_done_q))
          state_d = WRITE_WAIT;

      // FIX: wait for VALID && READY
      WRITE_WAIT:
        if (m_axi_bvalid && m_axi_bready)
          state_d = IDLE;

      READ_ADDR:
        if (m_axi_arready)
          state_d = READ_WAIT;

      // FIX: wait for VALID && READY
      READ_WAIT:
        if (m_axi_rvalid && m_axi_rready)
          state_d = IDLE;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q   <= IDLE;
      aw_done_q <= '0;
      w_done_q  <= '0;
    end else begin
      state_q <= state_d;

      // FIX: capture only on req && gnt
      if (data_req_i && data_gnt_o) begin
        addr_q  <= data_addr_i;
        wdata_q <= data_wdata_i;
        be_q    <= data_be_i;
      end

      if (state_q == WRITE_ADDR) begin
        if (m_axi_awready) aw_done_q <= 1'b1;
        if (m_axi_wready)  w_done_q  <= 1'b1;
      end else begin
        aw_done_q <= '0;
        w_done_q  <= '0;
      end
    end
  end

  // ---------------- Ibex side ----------------

  // FIX: proper grant (only when we can accept request)
  assign data_gnt_o = (state_q == IDLE) && data_req_i;

  // FIX: handshake-aligned valid
  assign data_rvalid_o =
      (state_q == WRITE_WAIT && m_axi_bvalid && m_axi_bready) ||
      (state_q == READ_WAIT  && m_axi_rvalid && m_axi_rready);

  // FIX: do NOT gate by state
  assign data_rdata_o = m_axi_rdata;

  // FIX: error only on handshake
  assign data_err_o =
      (state_q == WRITE_WAIT && m_axi_bvalid && m_axi_bready && m_axi_bresp[1]) ||
      (state_q == READ_WAIT  && m_axi_rvalid && m_axi_rready && m_axi_rresp[1]);

  // ---------------- AXI-Lite side ----------------

  assign m_axi_awaddr  = addr_q;
  assign m_axi_awvalid = (state_q == WRITE_ADDR) && !aw_done_q;

  assign m_axi_wdata   = wdata_q;
  assign m_axi_wstrb   = be_q;
  assign m_axi_wvalid  = (state_q == WRITE_ADDR) && !w_done_q;

  assign m_axi_bready  = (state_q == WRITE_WAIT);

  assign m_axi_araddr  = addr_q;
  assign m_axi_arvalid = (state_q == READ_ADDR);

  assign m_axi_rready  = (state_q == READ_WAIT);

endmodule