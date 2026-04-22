`timescale 1ns / 1ps

module URAM_Weight_Array #(
    parameter DATA_WIDTH = 774,
    parameter DEPTH      = 32768   // BATCH_SIZE * BATCH_NUMS (e.g., 1024 * 32)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // --- Port A: 主机写入接口 (Host -> URAM) ---
    input  wire                      host_we,
    input  wire [$clog2(DEPTH)-1:0]  host_addr,
    input  wire [DATA_WIDTH-1:0]     host_data,

    // --- Port B: DMA 读取接口 (URAM -> Pipeline -> BRAM) ---
    input  wire                      dma_rd_en,
    input  wire [$clog2(DEPTH)-1:0]  dma_rd_addr,
    output wire [DATA_WIDTH-1:0]     dma_rd_data
);

    // =========================================================================
    // 逻辑实现 (例化 Xilinx True Dual Port RAM)
    // =========================================================================
    localparam ADDR_W = $clog2(DEPTH);

    xpm_memory_tdpram #(
        .ADDR_WIDTH_A       (ADDR_W),
        .ADDR_WIDTH_B       (ADDR_W),
        .MEMORY_SIZE        (DEPTH * DATA_WIDTH), 
        .WRITE_DATA_WIDTH_A (DATA_WIDTH),
        .READ_DATA_WIDTH_B  (DATA_WIDTH),
        .READ_LATENCY_B     (1),                // 1 拍读出延迟
        .WRITE_MODE_A       ("no_change"),
        .WRITE_MODE_B       ("read_first"),
        .MEMORY_PRIMITIVE   ("ultra"),          // 强制使用 URAM
        .CLOCKING_MODE      ("common_clock"),
        .MEMORY_INIT_PARAM  ("0")
    ) u_xpm_uram (
        .clka           (clk),
        .ena            (1'b1),
        .wea            (host_we),
        .addra          (host_addr),
        .dina           (host_data),
        
        .clkb           (clk),
        .enb            (dma_rd_en),
        .addrb          (dma_rd_addr),
        .doutb          (dma_rd_data),
        
        // 闲置端口绑零
        .regceb         (1'b1),
        .rstb           (1'b0),
        .sleep          (1'b0),
        .injectsbiterra (1'b0),
        .injectdbiterra (1'b0),
        .sbiterrb       (),
        .dbiterrb       ()
    );

endmodule