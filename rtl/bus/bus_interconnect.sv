// bus_interconnect.v — Address decoder
// Memory map:
//   0x0010_0000–0x0010_3FFF : Data BRAM
//   0x0002_0000              : UART TX data (W)
//   0x0002_0004              : UART TX status (R)
//   0x0002_0008              : UART RX data (R)
module bus_interconnect #(
    parameter DATA_MEM_WORDS = 4096
) (
    input  wire        clk,
    input  wire        rst_n,
    // Ibex data port
    input  wire        data_req,
    input  wire        data_we,
    input  wire [3:0]  data_be,
    input  wire [31:0] data_addr,
    input  wire [31:0] data_wdata,
    output reg         data_rvalid,
    output reg  [31:0] data_rdata,
    // Data BRAM port
    output reg         bram_req,
    output wire        bram_we,
    output wire [3:0]  bram_be,
    output wire [31:0] bram_addr,
    output wire [31:0] bram_wdata,
    input  wire        bram_rvalid,
    input  wire [31:0] bram_rdata,
    // UART TX
    output wire        uart_tx_valid,
    output reg  [7:0]  uart_tx_data,
    input  wire        uart_tx_ready,
    // UART RX
    input  wire        uart_rx_valid,
    input  wire [7:0]  uart_rx_data,
    // PLIC 
    output wire        plic_valid_o,
    output wire        plic_write_o,
    output wire [31:0] plic_addr_o,
    output wire [31:0] plic_wdata_o,
    input  wire [31:0] plic_rdata_i,   

    // CLINT
    output wire [31:0] clint_awaddr_o,
    output wire        clint_awvalid_o,
    input  wire        clint_awready_i,
    output wire [31:0] clint_wdata_o,
    output wire [3:0]  clint_wstrb_o,
    output wire        clint_wvalid_o,
    input  wire        clint_wready_i,
    input  wire        clint_bvalid_i,
    output wire        clint_bready_o,
    output wire [31:0] clint_araddr_o,
    output wire        clint_arvalid_o,
    input  wire        clint_arready_i,
    input  wire [31:0] clint_rdata_i,
    input  wire        clint_rvalid_i,
    output wire        clint_rready_o
);

    assign bram_we    = data_we;
    assign bram_be    = data_be;
    assign bram_addr  = data_addr;
    assign bram_wdata = data_wdata;

    // Combinational BRAM decode
    always @(*) begin
        bram_req = 1'b0;
        if (data_req) begin
            if (data_addr[31:16] == 16'h0010)
                bram_req = 1'b1;
        end
    end

    // PLIC  0x0C00_0000 – 0x0FFF_FFFF  (addr[31:26] == 6'h03)
    // CLINT 0x0200_0000 – 0x0200_FFFF  (addr[31:16] == 16'h0200)
    wire sel_plic  = data_req && (data_addr[31:26] == 6'h03);
    wire sel_clint = data_req && (data_addr[31:16] == 16'h0200);

    // Combinational request drive — PLIC is always-ready, no handshake needed
    assign plic_valid_o = sel_plic;
    assign plic_write_o = data_we;
    assign plic_addr_o  = data_addr;
    assign plic_wdata_o = data_wdata;

    // Register PLIC read response on the request cycle
    reg [31:0] plic_rdata_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) plic_rdata_q <= 32'b0;
        else if (sel_plic && !data_we)
            plic_rdata_q <= plic_rdata_i;   // latch while combinational output is valid
    end

        // Write channels: assert AW and W simultaneously in the request cycle
    assign clint_awaddr_o  = data_addr;
    assign clint_awvalid_o = sel_clint && data_we;
    assign clint_wdata_o   = data_wdata;
    assign clint_wstrb_o   = data_be;
    assign clint_wvalid_o  = sel_clint && data_we;
    assign clint_bready_o  = 1'b1;   // always absorb B response; no error checking needed

    // Read channel
    assign clint_araddr_o  = data_addr;
    assign clint_arvalid_o = sel_clint && !data_we;
    assign clint_rready_o  = 1'b1;

    

    // Registered UART TX valid + data latch
    reg uart_tx_valid_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx_valid_r <= 1'b0;
            uart_tx_data    <= 8'b0;
        end else begin
            uart_tx_valid_r <= (data_req & data_we & (data_addr == 32'h0002_0000));
            if (data_req & data_we & (data_addr == 32'h0002_0000))
                uart_tx_data <= data_wdata[7:0];
        end
    end
    assign uart_tx_valid = uart_tx_valid_r;

    // Return data mux (registered, 1-cycle latency)
    reg [31:0] addr_q;
    reg        req_q, we_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_rvalid <= 1'b0;
            data_rdata  <= 32'b0;
            addr_q      <= 32'b0;
            req_q       <= 1'b0;
            we_q        <= 1'b0;
        end else begin
            req_q  <= data_req & ~data_we;
            we_q   <= data_req &  data_we;
            addr_q <= data_addr;
            if (bram_rvalid) begin
                data_rvalid <= 1'b1;
                data_rdata  <= bram_rdata;
            end else if (req_q) begin
                data_rvalid <= 1'b1;
                if (addr_q[31:26] == 6'h03) begin
                // PLIC read — data was latched into plic_rdata_q last cycle
                data_rdata <= plic_rdata_q;
            end else if (addr_q[31:16] == 16'h0200) begin
                // CLINT read — CLINT drives rvalid exactly this cycle
                data_rdata <= clint_rdata_i;
            end else begin
                case (addr_q)
                    32'h0002_0004: data_rdata <= {31'b0, uart_tx_ready};
                    32'h0002_0008: data_rdata <= {24'b0, uart_rx_data};
                    default:       data_rdata <= 32'hDEAD_BEEF;
                endcase
            end
            end else if (we_q) begin
                data_rvalid <= 1'b1;
                data_rdata  <= 32'b0;
            end else begin
                data_rvalid <= 1'b0;
            end
        end
    end

endmodule