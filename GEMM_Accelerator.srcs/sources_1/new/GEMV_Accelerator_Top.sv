`timescale 1ns / 1ps

module GEMV_Accelerator_Top #(
    parameter M                 = 256,
    parameter N                 = 256,
    parameter B_VEC_LEN         = 2048,
    parameter T_WIDTH           = 8,
    parameter OUT_WIDTH         = 32,
    parameter RAM_LATENCY       = 3,
    parameter PIPE_STAGES       = 2,

    parameter CU_NUMS        = B_VEC_LEN / N,   
    parameter BATCH_NUMS     = N / T_WIDTH,
    parameter BATCH_SIZE     = M * 4,
    parameter A_PACKED_WIDTH = 10 + T_WIDTH,
    parameter WIDE_BUS_WIDTH = CU_NUMS * A_PACKED_WIDTH
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // --- 主机控制接口 ---
    input  wire                               host_start,
    input  wire                               host_uram_batch_ready, // Host每写完一个Batch的URAM数据，拉高1拍
    output wire                               system_done,

    // --- 主机配置 L2 URAM 宽接口 (权重 A) ---
    input  wire                               host_we_uram,
    input  wire [$clog2(BATCH_SIZE*BATCH_NUMS)-1:0] host_addr_uram,
    input  wire [WIDE_BUS_WIDTH-1:0]          host_data_uram,

    // --- 主机配置 L1 BRAM 广播中心接口 (Vector B) ---
    input  wire                               host_we_act,
    input  wire [$clog2(B_VEC_LEN)-1:0]       host_addr_act,
    input  wire [7:0]                         host_data_act,
    input  wire                               host_start_broadcast,
    output wire                               broadcast_done,

    // --- 结果读取接口 ---
    input  wire                               out_rd_en,
    input  wire [$clog2(M)-1:0]               out_rd_addr,
    output wire [OUT_WIDTH-1:0]               out_data
);

    // =========================================================================
    // 内部互联信号
    // =========================================================================
    // Ctrl -> Mem
    wire                                 ctrl_start_dma;
    wire [$clog2(BATCH_NUMS)-1:0]        ctrl_dma_batch_idx;
    wire                                 mem_dma_done;

    // Ctrl -> Compute
    wire                                 ctrl_ping_pong_sel;
    wire                                 ctrl_start_compute;
    wire [$clog2(BATCH_NUMS)-1:0]        ctrl_compute_batch_idx;
    wire                                 compute_all_batch_done;
    wire                                 compute_all_matrix_done;
    wire                                 compute_all_cu_ready;       
    wire                                 ctrl_start_accum;
    wire                                 ctrl_accum_done;

    // Mem -> Compute
    wire                                 mem_bram_wr_en;
    wire [$clog2(BATCH_SIZE)-1:0]        mem_bram_wr_addr;
    wire [WIDE_BUS_WIDTH-1:0]            mem_bram_wr_data_wide;

    // =========================================================================
    // 1. 全局控制器 (大脑)
    // =========================================================================
    Global_Controller #(
        .N(N), .T_WIDTH(T_WIDTH)
    ) u_controller (
        .clk(clk), .rst_n(rst_n),
        .start_system(host_start), .host_uram_batch_ready(host_uram_batch_ready), .system_done(system_done),
        .start_dma(ctrl_start_dma), .dma_batch_idx(ctrl_dma_batch_idx), .dma_done(mem_dma_done),
        .ping_pong_sel(ctrl_ping_pong_sel), .start_compute(ctrl_start_compute),
        .compute_batch_idx(ctrl_compute_batch_idx), 
        .all_batch_done(compute_all_batch_done),
        .all_matrix_done(compute_all_matrix_done), 
        .all_cu_ready(compute_all_cu_ready),
        .start_accum(ctrl_start_accum), .accum_done(ctrl_accum_done)
    );

    // =========================================================================
    // 2. 存储子系统 (仓库与搬运)
    // =========================================================================
    Memory_Subsystem #(
        .M(M), .N(N), .B_VEC_LEN(B_VEC_LEN), .T_WIDTH(T_WIDTH),
        .RAM_LATENCY(RAM_LATENCY), .PIPE_STAGES(PIPE_STAGES)
    ) u_memory (
        .clk(clk), .rst_n(rst_n),
        .start_dma(ctrl_start_dma), .batch_idx(ctrl_dma_batch_idx), .dma_done(mem_dma_done),
        .host_we_uram(host_we_uram), .host_addr_uram(host_addr_uram), .host_data_uram(host_data_uram),
        .bram_wr_en(mem_bram_wr_en), .bram_wr_addr(mem_bram_wr_addr), .bram_wr_data_wide(mem_bram_wr_data_wide)
    );

    // =========================================================================
    // 3. 计算阵列核心 (Data Plane)
    // =========================================================================
    Compute_Core_Array #(
        .T_WIDTH(T_WIDTH), .M(M), .N(N), 
        .B_VEC_LEN(B_VEC_LEN), .CU_NUMS(CU_NUMS),
        .OUT_WIDTH(OUT_WIDTH)
    ) u_compute_array (
        .clk(clk), .rst_n(rst_n),
        
        .host_we_act(host_we_act), .host_addr_act(host_addr_act), .host_data_act(host_data_act),
        .start_broadcast(host_start_broadcast), .broadcast_done(broadcast_done),
        
        .ping_pong_sel(ctrl_ping_pong_sel),
        .dma_wr_en(mem_bram_wr_en), .dma_wr_addr(mem_bram_wr_addr), .dma_wr_data_wide(mem_bram_wr_data_wide),
        
        .start_compute(ctrl_start_compute), .current_k_offset(ctrl_compute_batch_idx),
        .all_batch_done(compute_all_batch_done), 
        .all_matrix_done(compute_all_matrix_done),
        .all_cu_ready(compute_all_cu_ready),
        .start_accum(ctrl_start_accum),.accum_done(ctrl_accum_done),
        .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
    );

endmodule