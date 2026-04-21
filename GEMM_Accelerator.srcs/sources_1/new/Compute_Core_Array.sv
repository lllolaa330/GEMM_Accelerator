`timescale 1ns / 1ps

module Compute_Core_Array #(
    parameter CU_NUMS           = 43,                 // CU 数量
    parameter M                 = 256,
    parameter N                 = 256,
    parameter T_WIDTH           = 8,
    parameter A_PACKED_WIDTH    = 18,
    parameter BATCH_NUMS        = N / T_WIDTH,
    parameter BATCH_SIZE        = M * 4,
    parameter OUT_WIDTH         = 32,
    parameter WIDE_BUS_WIDTH    = CU_NUMS * A_PACKED_WIDTH // 43 * 18 = 774
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // --- 主机配置广播中心接口 ---
    input  wire                               host_we_act,
    input  wire [7:0]                         host_addr_act,
    input  wire [7:0]                         host_data_act,
    input  wire                               start_broadcast,
    output wire                               broadcast_done,

    // --- DMA 宽总线数据流写入 (L2 URAM -> L1 PingPong) ---
    input  wire                               ping_pong_sel,
    input  wire                               dma_wr_en,
    input  wire [$clog2(BATCH_SIZE)-1:0]      dma_wr_addr,
    input  wire [WIDE_BUS_WIDTH-1:0]          dma_wr_data_wide,

    // --- 计算控制接口 ---
    input  wire                               start_compute,
    input  wire [$clog2(BATCH_NUMS)-1:0]      current_k_offset,
    output wire                               all_batch_done, // 43个CU全部完成一个Batch
    output wire                               all_matrix_done,

    // --- 结果读出接口 (供后级累加器使用) ---
    input  wire                               out_rd_en,
    input  wire [$clog2(M)-1:0]               out_rd_addr,
    output wire [CU_NUMS*OUT_WIDTH-1:0]       out_data_wide
);

    // =========================================================================
    // 内部广播信号与状态聚合
    // =========================================================================
    wire       brd_cfg_we_b;
    wire [7:0] brd_cfg_addr_b;
    wire [7:0] brd_cfg_data_b;

    wire [CU_NUMS-1:0] cu_batch_done_arr;
    wire [CU_NUMS-1:0] cu_matrix_done_arr;

    assign all_batch_done  = &cu_batch_done_arr;   // 按位与：所有CU都拉高才算完成
    assign all_matrix_done = &cu_matrix_done_arr;

    // =========================================================================
    // 1. 例化广播中心
    // =========================================================================
    Act_SI_Broadcast_Buffer #(
        .MAX_ACT_LEN(N)
    ) u_broadcast_center (
        .clk(clk), .rst_n(rst_n),
        .host_we_act(host_we_act), .host_addr_act(host_addr_act), .host_data_act(host_data_act),
        .host_we_si(1'b0), .host_addr_si(8'd0), .host_data_si(8'd0), // SI暂不接入
        .start_broadcast(start_broadcast),
        .broadcast_done(broadcast_done),
        .cfg_we_b(brd_cfg_we_b),
        .cfg_addr_b(brd_cfg_addr_b),
        .cfg_data_b(brd_cfg_data_b)
    );

    // =========================================================================
    // 2. 例化 43 个 CU Wrapper
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < CU_NUMS; i = i + 1) begin : CU_INST
            CU_Wrapper #(
                .M(M), .N(N), .T_WIDTH(T_WIDTH), 
                .BATCH_SIZE(BATCH_SIZE), .BATCH_NUMS(BATCH_NUMS)
            ) u_cu_wrapper (
                .clk(clk),
                .rst_n(rst_n),
                
                // --- DMA 接线：从宽总线中截取属于自己的 18-bit ---
                .ping_pong_sel(ping_pong_sel),
                .dma_wr_en(dma_wr_en),
                .dma_wr_addr(dma_wr_addr),
                .dma_wr_data(dma_wr_data_wide[i*A_PACKED_WIDTH +: A_PACKED_WIDTH]),
                
                // --- 控制与广播接线 (所有 CU 共享) ---
                .start_compute(start_compute),
                .current_k_offset(current_k_offset),
                .cfg_we_b(brd_cfg_we_b),
                .cfg_addr_b(brd_cfg_addr_b),
                .cfg_data_b(brd_cfg_data_b),
                
                // --- 输出接线：拼接回宽总线 ---
                .out_rd_en(out_rd_en),
                .out_rd_addr(out_rd_addr),
                .out_data(out_data_wide[i*OUT_WIDTH +: OUT_WIDTH]),
                .matrix_done(cu_matrix_done_arr[i]),
                .batch_done(cu_batch_done_arr[i]) 
                // 注意：由于修改了 SlidingWindow_CU，Wrapper 的端口需要加上 batch_done
            );
        end
    endgenerate

endmodule