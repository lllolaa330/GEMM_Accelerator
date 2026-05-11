`timescale 1ns / 1ps

module Compute_Core_Array #(
    parameter T_WIDTH           = 8,
    parameter M                 = 256,
    parameter N                 = 256,
    parameter B_VEC_LEN         = 4096,
    parameter OUT_WIDTH         = 32,
    parameter CU_NUMS           = 16,        // B_VEC_LEN / N
    parameter CU_TYPE           = 0,         // 0:Static, 1:Dynamic, 2:SlidingWindow
    parameter A_PACKED_WIDTH    = 10 + T_WIDTH,
    parameter BATCH_NUMS        = N / T_WIDTH,
    parameter BATCH_SIZE        = M * 4,
    parameter WIDE_BUS_WIDTH    = CU_NUMS * A_PACKED_WIDTH
)(
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               rst_batch_n, 

    // 主机配置
    input  wire                               host_we_act,
    input  wire [$clog2(B_VEC_LEN)-1:0]       host_addr_act,
    input  wire [7:0]                         host_data_act,
    input  wire                               start_broadcast_act,
    output wire                               broadcast_done_act,

    input  wire                               host_we_si,
    input  wire [T_WIDTH-1:0]                 host_addr_si, 
    input  wire [T_WIDTH-1:0]                 host_data_si,
    input  wire                               start_broadcast_si,
    output wire                               broadcast_done_si,

    // DMA 数据
    input  wire                               ping_pong_sel,
    input  wire                               dma_wr_en,
    input  wire [$clog2(BATCH_SIZE)-1:0]      dma_wr_addr,
    input  wire [WIDE_BUS_WIDTH-1:0]          dma_wr_data_wide,

    // 计算控制
    input  wire                               start_dma,
    input  wire                               start_compute,
    input  wire [$clog2(BATCH_NUMS)-1:0]      current_k_offset,
    output wire                               all_batch_done_posedge,
    output wire                               all_matrix_done,
    output wire                               all_cu_ready,

    // 累加器控制与输出
    input  wire                               start_accum,
    output wire                               accum_done,
    input  wire                               out_rd_en,
    input  wire [$clog2(M)-1:0]               out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]        out_data        
);

    wire                          brd_cfg_we_b;
    wire [$clog2(N)-1:0]          brd_cfg_addr_b;
    wire [CU_NUMS * 8 - 1 : 0]    brd_cfg_data_b_bus; // 并行数据总线

    wire                          brd_cfg_we_si;
    wire [T_WIDTH-1:0]            brd_cfg_addr_si;
    wire [T_WIDTH-1:0]            brd_cfg_data_si;

    wire [CU_NUMS-1:0] cu_batch_done_arr;
    wire [CU_NUMS-1:0] cu_matrix_done_arr;
    wire [CU_NUMS-1:0] cu_cu_ready_arr;

    reg [CU_NUMS-1:0] cu_batch_done_arr_flag;
    reg [CU_NUMS-1:0] cu_matrix_done_arr_flag;

    reg all_batch_done_d1;
    wire all_batch_done;
    assign all_batch_done_posedge = all_batch_done && !all_batch_done_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) all_batch_done_d1 <= 1'b0;
        else        all_batch_done_d1 <= all_batch_done;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cu_batch_done_arr_flag  <= {CU_NUMS{1'b0}};
            cu_matrix_done_arr_flag <= {CU_NUMS{1'b0}};
        end else if (start_dma) begin
            cu_batch_done_arr_flag  <= {CU_NUMS{1'b0}};
        end else begin
            for (int i = 0; i < CU_NUMS; i = i + 1) begin
                if (cu_batch_done_arr[i]) begin
                    cu_batch_done_arr_flag[i]  <= 1'b1; // 一旦某个CU的batch_done_arr为1，就保持1直到复位
                end
                if (cu_matrix_done_arr[i]) begin
                    cu_matrix_done_arr_flag[i] <= 1'b1;
                end
            end
        end
    end

    assign all_batch_done  = &cu_batch_done_arr_flag;   
    assign all_matrix_done = &cu_matrix_done_arr_flag;
    assign all_cu_ready    = &cu_cu_ready_arr;

    Act_SI_Broadcast_Buffer #(
        .N(N), .B_VEC_LEN(B_VEC_LEN),
        .T_WIDTH(T_WIDTH), .B_DATA_WIDTH(8)
    ) u_broadcast_center (
        .clk(clk), .rst_n(rst_batch_n),
        .host_we_act(host_we_act), .host_addr_act(host_addr_act), .host_data_act(host_data_act),
        .host_we_si(host_we_si), .host_addr_si(host_addr_si), .host_data_si(host_data_si), 
        .start_broadcast_b(start_broadcast_act),
        .broadcast_done_b(broadcast_done_act),
        .start_broadcast_si(start_broadcast_si),
        .broadcast_done_si(broadcast_done_si),
        
        .cfg_we_b(brd_cfg_we_b),
        .cfg_addr_b(brd_cfg_addr_b),
        .cfg_data_b_bus(brd_cfg_data_b_bus),
        .cfg_we_si(brd_cfg_we_si), 
        .cfg_addr_si(brd_cfg_addr_si), 
        .cfg_data_si(brd_cfg_data_si)  
    );

    // 内部总线：连接 CU 和 Accumulator
    wire                                 internal_rd_en;
    wire [$clog2(M)-1:0]                 internal_rd_addr;
    wire [CU_NUMS*OUT_WIDTH-1:0]         internal_out_wide;

    genvar i;
    generate
        for (i = 0; i < CU_NUMS; i = i + 1) begin : CU_INST
            CU_Wrapper #(
                .M(M), .N(N), .T_WIDTH(T_WIDTH), 
                .BATCH_SIZE(BATCH_SIZE), .BATCH_NUMS(BATCH_NUMS),
                .CU_TYPE(CU_TYPE)
            ) u_cu_wrapper (
                .clk(clk), .rst_n(rst_n),
                .ping_pong_sel(ping_pong_sel), .dma_wr_en(dma_wr_en), .dma_wr_addr(dma_wr_addr),
                .dma_wr_data(dma_wr_data_wide[i*A_PACKED_WIDTH +: A_PACKED_WIDTH]),
                .start_compute(start_compute), .current_k_offset(current_k_offset),
                .cfg_we_b(brd_cfg_we_b), .cfg_addr_b(brd_cfg_addr_b), .cfg_data_b(brd_cfg_data_b_bus[i*8 +: 8]),
                .cfg_we_si(brd_cfg_we_si), .cfg_addr_si(brd_cfg_addr_si), .cfg_data_si(brd_cfg_data_si),
                
                .out_rd_en(internal_rd_en),
                .out_rd_addr(internal_rd_addr),
                .out_data(internal_out_wide[i*OUT_WIDTH +: OUT_WIDTH]),
                
                .matrix_done(cu_matrix_done_arr[i]), .batch_done(cu_batch_done_arr[i]), .cu_ready(cu_cu_ready_arr[i])
            );
        end
    endgenerate

    // 行累加器
    Array_Output_Accumulation #(
        .COL_NUM(CU_NUMS), .L(M), .OUT_WIDTH(OUT_WIDTH)
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