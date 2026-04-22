`timescale 1ns / 1ps

module Memory_Subsystem #(
    parameter DATA_WIDTH  = 774,
    parameter BATCH_SIZE  = 1024,
    parameter BATCH_NUMS  = 32,
    parameter PIPE_STAGES = 2
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // --- 控制接口 ---
    input  wire                               start_dma,
    input  wire [$clog2(BATCH_NUMS)-1:0]      batch_idx,
    output wire                               dma_done,

    // --- 主机配置接口 (写 URAM) ---
    input  wire                               host_we_uram,
    input  wire [$clog2(BATCH_SIZE*BATCH_NUMS)-1:0] host_addr_uram,
    input  wire [DATA_WIDTH-1:0]              host_data_uram,

    // --- 数据流出接口 (至 Compute Core 的 Ping-Pong BRAM) ---
    output wire                               bram_wr_en,
    output wire [$clog2(BATCH_SIZE)-1:0]      bram_wr_addr,
    output wire [DATA_WIDTH-1:0]              bram_wr_data_wide
);

    // 内部连线
    wire                                      uram_rd_en;
    wire [$clog2(BATCH_SIZE*BATCH_NUMS)-1:0]  uram_rd_addr;
    wire [DATA_WIDTH-1:0]                     uram_dout;

    // 1. DMA 引擎
    Data_DMA_Engine #(
        .BATCH_SIZE(BATCH_SIZE), .BATCH_NUMS(BATCH_NUMS),
        .RAM_LATENCY(1), .PIPE_STAGES(PIPE_STAGES)
    ) u_dma (
        .clk(clk), .rst_n(rst_n),
        .start_dma(start_dma), .batch_idx(batch_idx), .dma_done(dma_done),
        .uram_rd_en(uram_rd_en), .uram_rd_addr(uram_rd_addr),
        .bram_wr_en(bram_wr_en), .bram_wr_addr(bram_wr_addr)
    );

    // 2. 核心大容量 URAM
    URAM_Weight_Array #(
        .DATA_WIDTH(DATA_WIDTH), .DEPTH(BATCH_SIZE*BATCH_NUMS)
    ) u_uram (
        .clk(clk), .rst_n(rst_n),
        .host_we(host_we_uram), .host_addr(host_addr_uram), .host_data(host_data_uram),
        .dma_rd_en(uram_rd_en), .dma_rd_addr(uram_rd_addr), .dma_rd_data(uram_dout)
    );

    // 3. 缓解布线压力的流水线
    Pipeline_Registers #(
        .WIDTH(DATA_WIDTH), .STAGES(PIPE_STAGES)
    ) u_pipe (
        .clk(clk), .rst_n(rst_n),
        .data_in(uram_dout), .data_out(bram_wr_data_wide)
    );

endmodule