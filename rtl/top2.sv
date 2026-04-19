// ============================================================================
// soc_top.sv
// Ibex RV32I SoC — AXI-Lite crossbar topology
// ============================================================================
//
// Architecture:
//
//   Ibex Core
//    ├── Instruction port ──► Internal BRAM (1-cycle latency, $readmemh init)
//    └── Data port (OBI) ──► ibex_to_axilite bridge
//                                   │
//                            axi_lite_xbar (ETH Zurich)
//                                   │
//                    ┌──────────────┴──────────────┐
//                 mst[0]                         mst[1] ...
//               Data RAM                        Peripherals
//              (exported                      (UART, GPIO, ...)
//             AXI-Lite pins)
//
// ============================================================================
// HOW TO ADD A PERIPHERAL — follow these 5 labelled steps:
//
//   PERIPH_ADD_1 : Increment NoMstPorts and NoAddrRules by 1 each
//   PERIPH_ADD_2 : Add one address rule to AddrMap (end_addr is EXCLUSIVE)
//   PERIPH_ADD_3 : Declare any top-level I/O ports for the peripheral
//   PERIPH_ADD_4 : Instantiate the peripheral module
//   PERIPH_ADD_5 : Wire peripheral to xbar_mst_req/resp at the new index
//
// That's it. Nothing else in this file needs to change.
// ============================================================================

`include "axi/typedef.svh"

module soc_top
  import ibex_pkg::*;
#(
  // ── Ibex core parameters ──────────────────────────────────────────────────
  parameter bit          RV32E            = 1'b0,
  parameter rv32m_e      RV32M            = RV32MFast,
  parameter rv32b_e      RV32B            = RV32BNone,
  parameter bit          WritebackStage   = 1'b1,
  parameter bit          BranchTargetALU  = 1'b0,
  parameter regfile_e    RegFile          = RegFileFF,

  // ── Instruction BRAM parameters ───────────────────────────────────────────
  // ImemDepthWords: number of 32-bit words. 4096 = 16 KB.
  // ImemInitFile  : path to Intel/Xilinx hex file loaded via $readmemh.
  parameter int unsigned ImemDepthWords   = 4096,
  parameter string       ImemInitFile     = "firmware.hex"
)(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic        test_en_i,
  input  logic [31:0] hart_id_i,
  input  logic [31:0] boot_addr_i,
  input  ibex_mubi_t  fetch_enable_i,

  // ── Interrupts ────────────────────────────────────────────────────────────
  input  logic        irq_software_i,
  input  logic        irq_timer_i,
  input  logic        irq_external_i,
  input  logic [14:0] irq_fast_i,
  input  logic        irq_nm_i,

  // ── Debug ─────────────────────────────────────────────────────────────────
  input  logic        debug_req_i,
  output crash_dump_t crash_dump_o,
  output logic        double_fault_seen_o,

  // ── Status / alerts ───────────────────────────────────────────────────────
  output logic        alert_minor_o,
  output logic        alert_major_internal_o,
  output logic        alert_major_bus_o,
  output logic        core_sleep_o,

  // ── AXI-Lite Master Port 0 — Data RAM (external to this module) ───────────
  // Map: 0x1000_0000 – 0x1000_FFFF (64 KB)
  output logic [31:0] m0_axi_awaddr_o,
  output logic        m0_axi_awvalid_o,
  input  logic        m0_axi_awready_i,
  output logic [31:0] m0_axi_wdata_o,
  output logic [ 3:0] m0_axi_wstrb_o,
  output logic        m0_axi_wvalid_o,
  input  logic        m0_axi_wready_i,
  input  logic [ 1:0] m0_axi_bresp_i,
  input  logic        m0_axi_bvalid_i,
  output logic        m0_axi_bready_o,
  output logic [31:0] m0_axi_araddr_o,
  output logic        m0_axi_arvalid_o,
  input  logic        m0_axi_arready_i,
  input  logic [31:0] m0_axi_rdata_i,
  input  logic [ 1:0] m0_axi_rresp_i,
  input  logic        m0_axi_rvalid_i,
  output logic        m0_axi_rready_o

  // ── PERIPH_ADD_3: Declare peripheral I/O ports here ──────────────────────
  //
  // Add a comma after m0_axi_rready_o above, then add your ports, e.g.:
  //
  // ,  // ← add this comma to the line above first
  //
  // // UART (AXI-Lite UART, e.g. Xilinx axi_uartlite)
  // output logic        uart_tx_o,
  // input  logic        uart_rx_i
  //
  // // SPI master
  // output logic        spi_sclk_o,
  // output logic        spi_mosi_o,
  // input  logic        spi_miso_i,
  // output logic        spi_csn_o
  //
  // ─────────────────────────────────────────────────────────────────────────
);

  // ==========================================================================
  // AXI-Lite type definitions (32-bit address & data throughout)
  // ==========================================================================
  typedef logic [31:0] addr_t;
  typedef logic [31:0] data_t;
  typedef logic [ 3:0] strb_t;

  `AXI_LITE_TYPEDEF_AW_CHAN_T(aw_chan_t, addr_t)
  `AXI_LITE_TYPEDEF_W_CHAN_T (w_chan_t,  data_t, strb_t)
  `AXI_LITE_TYPEDEF_B_CHAN_T (b_chan_t)
  `AXI_LITE_TYPEDEF_AR_CHAN_T(ar_chan_t, addr_t)
  `AXI_LITE_TYPEDEF_R_CHAN_T (r_chan_t,  data_t)
  `AXI_LITE_TYPEDEF_REQ_T   (axi_req_t,  aw_chan_t, w_chan_t,  ar_chan_t)
  `AXI_LITE_TYPEDEF_RESP_T  (axi_resp_t, b_chan_t,  r_chan_t)

  // ==========================================================================
  // Crossbar port counts
  // ==========================================================================
  //
  // PERIPH_ADD_1: When adding a peripheral, change NoMstPorts and
  //               NoAddrRules from N to N+1. NoSlvPorts stays 1 forever
  //               (only one master on the bus — the Ibex data port).
  //
  //               Current mapping:
  //                 mst port 0  →  Data RAM  (0x1000_0000 – 0x1000_FFFF)
  //
  //               After adding UART:
  //                 NoMstPorts  = 2
  //                 NoAddrRules = 2
  //                 mst port 0  →  Data RAM  (0x1000_0000 – 0x1000_FFFF)
  //                 mst port 1  →  UART      (0x4000_0000 – 0x4000_0FFF)
  //
  localparam int unsigned NoSlvPorts  = 1;  // fixed — only Ibex data port
  localparam int unsigned NoMstPorts  = 2;  // ← PERIPH_ADD_1: +1 per peripheral
  localparam int unsigned NoAddrRules = 2;  // ← PERIPH_ADD_1: +1 per peripheral

  // ==========================================================================
  // Crossbar configuration struct
  // ==========================================================================
  // LatencyMode = CUT_ALL_PORTS inserts spill registers on every channel —
  // this is the safest choice for FPGA timing closure.
  //
  // If your ETH Zurich axi_pkg version does NOT have PipelineStages /
  // UniqueIds fields, remove those two lines from the struct literal.
  // ==========================================================================
  localparam axi_pkg::xbar_cfg_t XbarCfg = '{
    NoSlvPorts:     NoSlvPorts,
    NoMstPorts:     NoMstPorts,
    MaxMstTrans:    4,
    MaxSlvTrans:    4,
    FallThrough:    1'b0,
    LatencyMode:    axi_pkg::CUT_ALL_PORTS,
    PipelineStages: 0,
    AxiAddrWidth:   32,
    AxiDataWidth:   32,
    NoAddrRules:    NoAddrRules,
    UniqueIds:      1'b0
  };

  typedef axi_pkg::xbar_rule_32_t rule_t;

  // ==========================================================================
  // Address map
  // ==========================================================================
  //
  // PERIPH_ADD_2: Add one rule per peripheral.
  //
  //   Rules:
  //     • idx         : xbar master port index (0-based, matches NoMstPorts order)
  //     • start_addr  : first byte of region (inclusive)
  //     • end_addr    : first byte BEYOND the region (exclusive)
  //                     e.g. 64 KB RAM → end = start + 0x10000
  //
  //   Address space suggestions (leave gaps so you can expand later):
  //     0x1000_0000 – 0x1FFF_FFFF  : memories
  //     0x4000_0000 – 0x4FFF_FFFF  : peripherals (UART, SPI, GPIO, I2C …)
  //
  //   Example after adding UART at mst port 1:
  //     localparam rule_t [NoAddrRules-1:0] AddrMap = '{
  //       '{idx: 32'd1, start_addr: 32'h4000_0000, end_addr: 32'h4000_1000}, // UART  4KB
  //       '{idx: 32'd0, start_addr: 32'h1000_0000, end_addr: 32'h1001_0000}  // RAM  64KB
  //     };
  //   (List highest-index rule first in the array literal — SV fills
  //    [N-1:0] from left, so element [N-1] is written first.)
  //
  localparam rule_t [NoAddrRules-1:0] AddrMap = '{
    '{idx: 32'd1, start_addr: 32'h0C00_0000, end_addr: 32'h0C30_0000},
    '{idx: 32'd0, start_addr: 32'h1000_0000, end_addr: 32'h1001_0000}  // Data RAM 64KB
    // PERIPH_ADD_2: prepend new rules here (higher index first), e.g.:
    // '{idx: 32'd1, start_addr: 32'h4000_0000, end_addr: 32'h4000_1000}  // UART 4KB
  };

  // ==========================================================================
  // Ibex signal declarations
  // ==========================================================================

  // Instruction port → internal BRAM
  logic        instr_req;
  logic        instr_gnt;
  logic        instr_rvalid;
  logic        instr_err;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;

  // Data port → AXI-Lite bridge
  logic        data_req;
  logic        data_gnt;
  logic        data_rvalid;
  logic        data_err;
  logic        data_we;
  logic [ 3:0] data_be;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [31:0] data_rdata;

  // ==========================================================================
  // Ibex core
  // ==========================================================================
  // Notes:
  //   • ram_cfg_i  : tie to '0 on FPGA (no special RAM config needed)
  //   • *_intg_*  : integrity signals — not used here, tied to '0
  //   • DbgTriggerEn = 0 saves area; enable if you wire up a debug module
  //   • DmHaltAddr / DmExceptionAddr: standard OpenTitan debug ROM addresses;
  //     change these if you add a RISC-V debug module (JTAG) later
  // ==========================================================================
  ibex_top #(
    .RV32E           (RV32E),
    .RV32M           (RV32M),
    .RV32B           (RV32B),
    .WritebackStage  (WritebackStage),
    .BranchTargetALU (BranchTargetALU),
    .RegFile         (RegFile),
    .DbgTriggerEn    (1'b0),
    .DmHaltAddr      (32'h1A11_0800),
    .DmExceptionAddr (32'h1A11_0808)
  ) u_ibex_top (
    .clk_i,
    .rst_ni,
    .test_en_i,
    .ram_cfg_i          ('0),

    .hart_id_i,
    .boot_addr_i,

    // ── Instruction fetch → BRAM ───────────────────────────────────────────
    .instr_req_o        (instr_req),
    .instr_gnt_i        (instr_gnt),
    .instr_rvalid_i     (instr_rvalid),
    .instr_addr_o       (instr_addr),
    .instr_rdata_i      (instr_rdata),
    .instr_rdata_intg_i ('0),
    .instr_err_i        (instr_err),

    // ── Data access → AXI-Lite bridge ─────────────────────────────────────
    .data_req_o         (data_req),
    .data_gnt_i         (data_gnt),
    .data_rvalid_i      (data_rvalid),
    .data_addr_o        (data_addr),
    .data_we_o          (data_we),
    .data_be_o          (data_be),
    .data_wdata_o       (data_wdata),
    .data_rdata_i       (data_rdata),
    .data_rdata_intg_i  ('0),
    .data_err_i         (data_err),

    // ── Interrupts ─────────────────────────────────────────────────────────
    .irq_software_i,
    .irq_timer_i,
    .irq_external_i(plic_irq_o),
    .irq_fast_i,
    .irq_nm_i,
    .irq_pending_o      (),         // not used; tie off

    // ── Debug ──────────────────────────────────────────────────────────────
    .debug_req_i,
    .crash_dump_o,
    .double_fault_seen_o,

    // ── Fetch enable / alerts ──────────────────────────────────────────────
    .fetch_enable_i,
    .alert_minor_o,
    .alert_major_internal_o,
    .alert_major_bus_o,
    .core_sleep_o
  );



  // ==========================================================================
  // Instruction memory — Xilinx BRAM (inferred), initialised from hex file
  // ==========================================================================
  //
  // Timing (matches Ibex blocking-LSU expectations):
  //   Cycle 0 : instr_req=1 → instr_gnt=1 (combinational, BRAM always ready)
  //   Cycle 1 : instr_rvalid=1, instr_rdata valid
  //
  // To resize: change ImemDepthWords parameter (must be a power of 2).
  // To load a different program: change ImemInitFile parameter or the
  //   $readmemh call below. The hex file must use word addresses.
  //
  // Address decode: byte address [AddrHigh:2] indexes the word array.
  //   For 4096 words (16 KB): bits [13:2] → indices 0..4095
  //
  // ==========================================================================
  localparam int unsigned ImemAddrBits = $clog2(ImemDepthWords); // e.g. 12 for 4096

  logic [31:0] imem [0:ImemDepthWords-1];
  // synthesis translate_off
  initial begin
    $readmemh(ImemInitFile, imem);
  end
  // synthesis translate_on
  // For Xilinx: use the equivalent ROM initialisation attribute if needed,
  // but $readmemh inside an initial block IS supported for BRAM inference.

  logic instr_rvalid_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      instr_rvalid_q <= 1'b0;
      instr_rdata    <= 32'h0000_0013; // NOP (addi x0,x0,0) on reset
    end else begin
      instr_rvalid_q <= instr_req; // gnt is always 1, so rvalid follows req
      if (instr_req)
        // Word index = byte address >> 2, masked to array depth
        instr_rdata <= imem[instr_addr[ImemAddrBits+1:2]];
    end
  end

  assign instr_gnt    = instr_req;      // BRAM accepts every cycle
  assign instr_rvalid = instr_rvalid_q;
  assign instr_err    = 1'b0;

  // ==========================================================================
  // AXI-Lite wires — bridge output signals
  // ==========================================================================
  logic [31:0] db_awaddr;
  logic        db_awvalid, db_awready;
  logic [31:0] db_wdata;
  logic [ 3:0] db_wstrb;
  logic        db_wvalid,  db_wready;
  logic [ 1:0] db_bresp;
  logic        db_bvalid,  db_bready;
  logic [31:0] db_araddr;
  logic        db_arvalid, db_arready;
  logic [31:0] db_rdata;
  logic [ 1:0] db_rresp;
  logic        db_rvalid,  db_rready;

  // ==========================================================================
  // Data bridge — Ibex OBI → AXI-Lite
  // ==========================================================================
  ibex_to_axilite #(
    .AW (32),
    .DW (32)
  ) u_data_bridge (
    .clk_i,
    .rst_ni,

    // Ibex data port
    .data_req_i    (data_req),
    .data_gnt_o    (data_gnt),
    .data_rvalid_o (data_rvalid),
    .data_addr_i   (data_addr),
    .data_we_i     (data_we),
    .data_be_i     (data_be),
    .data_wdata_i  (data_wdata),
    .data_rdata_o  (data_rdata),
    .data_err_o    (data_err),

    // AXI-Lite master → crossbar slave port 0
    .m_axi_awaddr  (db_awaddr),
    .m_axi_awvalid (db_awvalid),
    .m_axi_awready (db_awready),
    .m_axi_wdata   (db_wdata),
    .m_axi_wstrb   (db_wstrb),
    .m_axi_wvalid  (db_wvalid),
    .m_axi_wready  (db_wready),
    .m_axi_bresp   (db_bresp),
    .m_axi_bvalid  (db_bvalid),
    .m_axi_bready  (db_bready),
    .m_axi_araddr  (db_araddr),
    .m_axi_arvalid (db_arvalid),
    .m_axi_arready (db_arready),
    .m_axi_rdata   (db_rdata),
    .m_axi_rresp   (db_rresp),
    .m_axi_rvalid  (db_rvalid),
    .m_axi_rready  (db_rready)
  );

  // ==========================================================================
  // Crossbar request/response arrays
  // ==========================================================================
  axi_req_t  [NoSlvPorts-1:0] xbar_slv_req;
  axi_resp_t [NoSlvPorts-1:0] xbar_slv_resp;
  axi_req_t  [NoMstPorts-1:0] xbar_mst_req;
  axi_resp_t [NoMstPorts-1:0] xbar_mst_resp;

  // ==========================================================================
  // Pack bridge flat signals → xbar slave port 0 struct
  // (Nothing here changes when adding peripherals — this is always port 0)
  // ==========================================================================
  assign xbar_slv_req[0].aw.addr  = db_awaddr;
  assign xbar_slv_req[0].aw_valid = db_awvalid;
  assign xbar_slv_req[0].w.data   = db_wdata;
  assign xbar_slv_req[0].w.strb   = db_wstrb;
  assign xbar_slv_req[0].w_valid  = db_wvalid;
  assign xbar_slv_req[0].b_ready  = db_bready;
  assign xbar_slv_req[0].ar.addr  = db_araddr;
  assign xbar_slv_req[0].ar_valid = db_arvalid;
  assign xbar_slv_req[0].r_ready  = db_rready;

  assign db_awready = xbar_slv_resp[0].aw_ready;
  assign db_wready  = xbar_slv_resp[0].w_ready;
  assign db_bresp   = xbar_slv_resp[0].b.resp;
  assign db_bvalid  = xbar_slv_resp[0].b_valid;
  assign db_arready = xbar_slv_resp[0].ar_ready;
  assign db_rdata   = xbar_slv_resp[0].r.data;
  assign db_rresp   = xbar_slv_resp[0].r.resp;
  assign db_rvalid  = xbar_slv_resp[0].r_valid;

  // ==========================================================================
  // ETH Zurich AXI-Lite crossbar
  // ==========================================================================
  axi_lite_xbar #(
    .Cfg        (XbarCfg),
    .aw_chan_t  (aw_chan_t),
    .w_chan_t   (w_chan_t),
    .b_chan_t   (b_chan_t),
    .ar_chan_t  (ar_chan_t),
    .r_chan_t   (r_chan_t),
    .axi_req_t  (axi_req_t),
    .axi_resp_t (axi_resp_t),
    .rule_t     (rule_t)
  ) u_xbar (
    .clk_i,
    .rst_ni,
    .test_i                (test_en_i),
    .slv_ports_req_i       (xbar_slv_req),
    .slv_ports_resp_o      (xbar_slv_resp),
    .mst_ports_req_o       (xbar_mst_req),
    .mst_ports_resp_i      (xbar_mst_resp),
    .addr_map_i            (AddrMap),
    .en_default_mst_port_i ('0),    // no default route; unmapped → decode error
    .default_mst_port_i    ('0)
  );

  // ==========================================================================
  // Xbar master port 0 — Data RAM (pinned out as top-level AXI-Lite signals)
  // ==========================================================================
  assign m0_axi_awaddr_o  = xbar_mst_req[0].aw.addr;
  assign m0_axi_awvalid_o = xbar_mst_req[0].aw_valid;
  assign m0_axi_wdata_o   = xbar_mst_req[0].w.data;
  assign m0_axi_wstrb_o   = xbar_mst_req[0].w.strb;
  assign m0_axi_wvalid_o  = xbar_mst_req[0].w_valid;
  assign m0_axi_bready_o  = xbar_mst_req[0].b_ready;
  assign m0_axi_araddr_o  = xbar_mst_req[0].ar.addr;
  assign m0_axi_arvalid_o = xbar_mst_req[0].ar_valid;
  assign m0_axi_rready_o  = xbar_mst_req[0].r_ready;

  assign xbar_mst_resp[0].aw_ready = m0_axi_awready_i;
  assign xbar_mst_resp[0].w_ready  = m0_axi_wready_i;
  assign xbar_mst_resp[0].b.resp   = m0_axi_bresp_i;
  assign xbar_mst_resp[0].b_valid  = m0_axi_bvalid_i;
  assign xbar_mst_resp[0].ar_ready = m0_axi_arready_i;
  assign xbar_mst_resp[0].r.data   = m0_axi_rdata_i;
  assign xbar_mst_resp[0].r.resp   = m0_axi_rresp_i;
  assign xbar_mst_resp[0].r_valid  = m0_axi_rvalid_i;

  // ==========================================================================
  // PERIPH_ADD_4 + PERIPH_ADD_5: Peripheral instances go here
  // ==========================================================================
  //
  // Each peripheral consumes one xbar master port (index N).
  // The xbar_mst_req[N] struct carries all the AXI-Lite request channels
  // OUT of the crossbar TO the peripheral (slave).
  // The xbar_mst_resp[N] struct carries all the AXI-Lite response channels
  // back FROM the peripheral TO the crossbar.
  //
  // ── UART example (Xilinx axi_uartlite or any AXI-Lite UART slave) ────────
  //
  // Step summary for UART:
  //   PERIPH_ADD_1: NoMstPorts = 2, NoAddrRules = 2
  //   PERIPH_ADD_2: add rule for 0x4000_0000..0x4000_1000 at idx 1
  //   PERIPH_ADD_3: add uart_tx_o / uart_rx_i to module port list above
  //   PERIPH_ADD_4+5 below (uncomment):
  //
  // axi_uartlite_0 u_uart (        // generated from Xilinx IP catalog
  //   .s_axi_aclk    (clk_i),
  //   .s_axi_aresetn (rst_ni),
  //
  //   .s_axi_awaddr  (xbar_mst_req[1].aw.addr[3:0]), // UARTLITE only needs [3:0]
  //   .s_axi_awvalid (xbar_mst_req[1].aw_valid),
  //   .s_axi_awready (xbar_mst_resp[1].aw_ready),
  //   .s_axi_wdata   (xbar_mst_req[1].w.data),
  //   .s_axi_wstrb   (xbar_mst_req[1].w.strb),
  //   .s_axi_wvalid  (xbar_mst_req[1].w_valid),
  //   .s_axi_wready  (xbar_mst_resp[1].w_ready),
  //   .s_axi_bresp   (xbar_mst_resp[1].b.resp),
  //   .s_axi_bvalid  (xbar_mst_resp[1].b_valid),
  //   .s_axi_bready  (xbar_mst_req[1].b_ready),
  //   .s_axi_araddr  (xbar_mst_req[1].ar.addr[3:0]),
  //   .s_axi_arvalid (xbar_mst_req[1].ar_valid),
  //   .s_axi_arready (xbar_mst_resp[1].ar_ready),
  //   .s_axi_rdata   (xbar_mst_resp[1].r.data),
  //   .s_axi_rresp   (xbar_mst_resp[1].r.resp),
  //   .s_axi_rvalid  (xbar_mst_resp[1].r_valid),
  //   .s_axi_rready  (xbar_mst_req[1].r_ready),
  //
  //   .tx            (uart_tx_o),
  //   .rx            (uart_rx_i),
  //   .interrupt      ()           // tie off or wire to irq_external_i
  // );
  //
  // ── GPIO example (any AXI-Lite GPIO slave) at master port 2 ──────────────
  //
  // my_axi_gpio u_gpio (
  //   .s_axi_aclk    (clk_i),
  //   .s_axi_aresetn (rst_ni),
  //   .s_axi_awaddr  (xbar_mst_req[2].aw.addr),
  //   ... (same pattern as UART above, just use index [2]) ...
  //   .gpio_o        (gpio_o)      // declared in PERIPH_ADD_3
  // );
  //
  // ── Pattern for every peripheral ─────────────────────────────────────────
  //   xbar_mst_req[N]  → .s_axi_aw*, .s_axi_w*, .s_axi_b_ready,
  //                       .s_axi_ar*, .s_axi_r_ready
  //   xbar_mst_resp[N] ← .s_axi_aw_ready, .s_axi_w_ready,
  //                       .s_axi_b*, .s_axi_ar_ready, .s_axi_r*
  //
  // ==========================================================================

    // Define the simple reg interface types (matching plic_regs.sv)
typedef struct packed {
  logic        valid;
  logic        write;
  logic [31:0] addr;
  logic [31:0] wdata;
  logic [ 3:0] wstrb;
} plic_reg_req_t;

typedef struct packed {
  logic        ready;
  logic [31:0] rdata;
  logic        error;
} plic_reg_rsp_t;

plic_reg_req_t plic_reg_req;
plic_reg_rsp_t plic_reg_rsp;

// Bridge: AXI-Lite → register interface
axilite_to_reg #(
  .axi_req_t  (axi_req_t),
  .axi_resp_t (axi_resp_t),
  .reg_req_t  (plic_reg_req_t),
  .reg_rsp_t  (plic_reg_rsp_t)
) u_plic_bridge (
  .clk_i      (clk_i),
  .rst_ni     (rst_ni),
  .axi_req_i  (xbar_mst_req[1]),
  .axi_resp_o (xbar_mst_resp[1]),
  .reg_req_o  (plic_reg_req),
  .reg_rsp_i  (plic_reg_rsp)
);

// PLIC interrupt source wiring
logic [12:0] plic_intr_src;
assign plic_intr_src[0]  = 1'b0;
assign plic_intr_src[1]  = uart_tx_irq;
assign plic_intr_src[2]  = uart_rx_irq;
assign plic_intr_src[3]  = gpio_irq[0];
assign plic_intr_src[4]  = gpio_irq[1];
assign plic_intr_src[5]  = gpio_irq[2];
assign plic_intr_src[6]  = gpio_irq[3];
assign plic_intr_src[7]  = timer_irq;
assign plic_intr_src[8]  = spi_irq;
assign plic_intr_src[9]  = 1'b0;
assign plic_intr_src[10] = 1'b0;
assign plic_intr_src[11] = 1'b0;
assign plic_intr_src[12] = 1'b0;

logic plic_irq_o;
// PLIC top
plic_top #(
  .N_SOURCE (12),
  .N_TARGET (1),
  .MAX_PRIO (3)
) u_plic (
  .clk_i       (clk_i),
  .rst_ni      (rst_ni),
  .intr_src_i  (plic_intr_src),
  .irq_o       (plic_irq_o),
  .irq_id_o    (),              // not needed unless you do fast claim in firmware
  .req_i       (plic_reg_req),
  .resp_o      (plic_reg_rsp)
);

endmodule