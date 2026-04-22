`timescale 1ns / 1ps

module Compute_Core_Array #(
    parameter CU_C_NUMS         = 4,                 
    parameter M                 = 256,
    parameter N                 = 256,
    parameter T_WIDTH           = 8,
    parameter A_PACKED_WIDTH    = 10 + T_WIDTH,
    parameter BATCH_NUMS        = N / T_WIDTH,
    parameter BATCH_SIZE        = M * 4,
    parameter OUT_WIDTH         = 32,
    parameter WIDE_BUS_WIDTH    = CU_C_NUMS * A_PACKED_WIDTH
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // 主机配置
    input  wire                               host_we_act,
    input  wire [7:0]                         host_addr_act,
    input  wire [7:0]                         host_data_act,
    input  wire                               start_broadcast,
    output wire                               broadcast_done,

    // DMA 数据
    input  wire                               ping_pong_sel,
    input  wire                               dma_wr_en,
    input  wire [$clog2(BATCH_SIZE)-1:0]      dma_wr_addr,
    input  wire [WIDE_BUS_WIDTH-1:0]          dma_wr_data_wide,

    // 计算控制
    input  wire                               start_compute,
    input  wire [$clog2(BATCH_NUMS)-1:0]      current_k_offset,
    output wire                               all_batch_done,
    output wire                               all_matrix_done,

    // --- 新增：累加器控制与输出 ---
    input  wire                               start_accum,
    output wire                               accum_done,
    input  wire                               out_rd_en,
    input  wire [$clog2(M)-1:0]               out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]        out_data        // 变成干净的 32-bit
);

    wire       brd_cfg_we_b;
    wire [7:0] brd_cfg_addr_b;
    wire [7:0] brd_cfg_data_b;

    wire [CU_C_NUMS-1:0] cu_batch_done_arr;
    wire [CU_C_NUMS-1:0] cu_matrix_done_arr;

    assign all_batch_done  = &cu_batch_done_arr;   
    assign all_matrix_done = &cu_matrix_done_arr;

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

    // 内部总线：连接 CU 和 Accumulator
    wire                                 internal_rd_en;
    wire [$clog2(M)-1:0]                 internal_rd_addr;
    wire [CU_C_NUMS*OUT_WIDTH-1:0]         internal_out_wide;

    genvar i;
    generate
        for (i = 0; i < CU_C_NUMS; i = i + 1) begin : CU_INST
            CU_Wrapper #(
                .M(M), .N(N), .T_WIDTH(T_WIDTH), 
                .BATCH_SIZE(BATCH_SIZE), .BATCH_NUMS(BATCH_NUMS)
            ) u_cu_wrapper (
                .clk(clk), .rst_n(rst_n),
                .ping_pong_sel(ping_pong_sel), .dma_wr_en(dma_wr_en), .dma_wr_addr(dma_wr_addr),
                .dma_wr_data(dma_wr_data_wide[i*A_PACKED_WIDTH +: A_PACKED_WIDTH]),
                .start_compute(start_compute), .current_k_offset(current_k_offset),
                .cfg_we_b(brd_cfg_we_b), .cfg_addr_b(brd_cfg_addr_b), .cfg_data_b(brd_cfg_data_b),
                
                // --- 被 Accumulator 接管读取权 ---
                .out_rd_en(internal_rd_en),
                .out_rd_addr(internal_rd_addr),
                .out_data(internal_out_wide[i*OUT_WIDTH +: OUT_WIDTH]),
                
                .matrix_done(cu_matrix_done_arr[i]), .batch_done(cu_batch_done_arr[i]) 
            );
        end
    endgenerate

    // --- 例化累加器，收尾工作 ---
    Array_Output_Accumulation #(
        .COL_NUM(CU_C_NUMS), .L(M), .OUT_WIDTH(OUT_WIDTH)
    ) u_accumulator (
        .clk(clk),
        .rst_n(rst_n),
        .start(start_accum),
        .done(accum_done),
        .cu_rd_en(internal_rd_en),
        .cu_rd_addr(internal_rd_addr),
        .cu_out_data_flat(internal_out_wide),
        
        .ext_rd_en(out_rd_en),
        .ext_rd_addr(out_rd_addr),
        .ext_rd_data(out_data)
    );

endmodule