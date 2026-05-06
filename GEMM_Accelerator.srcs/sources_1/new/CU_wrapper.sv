`timescale 1ns / 1ps

module CU_Wrapper #(
    parameter CU_TYPE           = 0,    // 0:Sliding, 1:Dynamic, 2:Static, 3:DSP
    parameter M                 = 64,
    parameter N                 = 64,
    parameter T_WIDTH           = 8,
    parameter A_PACKED_WIDTH    = 10 + T_WIDTH,
    parameter B_DATA_BITS       = 8,
    parameter PSUM_WIDTH        = 24,
    parameter OUT_WIDTH         = 32,
    parameter BATCH_NUMS        = N / T_WIDTH,
    parameter BATCH_SIZE        = M * 4,
    parameter OFFSET_ADDR_WIDTH = $clog2(BATCH_NUMS),
    parameter OUTPUT_ADDR_WIDTH = $clog2(M)
)(
    input  wire                                 clk,
    input  wire                                 rst_n,

    // --- 顶层/DMA 控制接口 (runtime 权重注入) ---
    input  wire                                 ping_pong_sel,    // 0/1 切换状态
    input  wire                                 dma_wr_en,
    input  wire [$clog2(BATCH_SIZE)-1:0]        dma_wr_addr,
    input  wire [A_PACKED_WIDTH-1:0]            dma_wr_data,

    // --- 运算控制接口 ---
    input  wire                                 start_compute,    // 单脉冲：启动当前 Batch 的计算
    input  wire [OFFSET_ADDR_WIDTH-1:0]         current_k_offset, // 当前是第几个 Batch (传给底层CU，用于内部选择对应的b数据)
    
    // --- Vector B 配置接口 ---
    input  wire                                 cfg_we_b,
    input  wire [7:0]                           cfg_addr_b,
    input  wire [7:0]                           cfg_data_b, 

    // --- SI 配置接口 ---
    input  wire                                 cfg_we_si,
    input  wire [7:0]                           cfg_addr_si,
    input  wire [T_WIDTH-1:0]                   cfg_data_si,

    // --- dsp 接口 ---
    input  wire                                 dsp_a_valid,
    input  wire [3:0]                           dsp_a_payload,

    // --- 结果输出接口 ---
    input  wire                                 out_rd_en,
    input  wire [OUTPUT_ADDR_WIDTH-1:0]         out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]          out_data,
    output wire                                 batch_done,
    output wire                                 matrix_done
);

    // =========================================================================
    // 参数与内部信号声明
    // =========================================================================
    localparam ADDR_W = $clog2(BATCH_SIZE);

    // 内部握手信号
    wire cu_ready;

    // BRAM 到 CU 的连线
    wire                              cu_rd_en;
    reg  [ADDR_W-1:0]                 cu_rd_addr;
    wire [A_PACKED_WIDTH-1:0]         bram_dout;

    // BRAM 读延迟对齐流水线
    reg                               read_active; // 标识地址生成器是否在工作
    reg                               a_valid_d1;  // 延迟 1 拍，对齐 BRAM 读出数据

    // =========================================================================
    // 逻辑 1: CU 自动地址发生器 (AGU)
    // =========================================================================
    // 当收到 start_compute 脉冲，开始连续生成 0 ~ BATCH_SIZE-1 的地址去读 BRAM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_active <= 1'b0;
            cu_rd_addr  <= 0;
        end else if (start_compute) begin
            read_active <= 1'b1;
            cu_rd_addr  <= 0;
        end else if (read_active && cu_ready) begin // 只有 CU ready 时才继续生成地址
            if (cu_rd_addr == BATCH_SIZE - 1) begin
                read_active <= 1'b0;
            end else begin
                cu_rd_addr <= cu_rd_addr + 1'b1;
            end
        end
    end
    assign cu_rd_en = read_active && cu_ready;


    // =========================================================================
    // 逻辑 2: 数据对齐流水线
    // =========================================================================
    // BRAM 的 READ_LATENCY_B = 1，所以 valid 信号需要打 1 拍与数据完全对齐
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_valid_d1 <= 1'b0;
        end else begin
            a_valid_d1 <= cu_rd_en;
        end
    end

    // =========================================================================
    // 例化 1: Ping-Pong BRAM (权重局部缓存)
    // =========================================================================
    PingPong_Weight_BRAM #(
        .DATA_WIDTH(A_PACKED_WIDTH),
        .DEPTH_PER_BANK(BATCH_SIZE)
    ) u_local_bram (
        .clk(clk),
        .rst_n(rst_n),
        .ping_pong_sel(ping_pong_sel),
        
        // 写端接 DMA
        .dma_wr_en(dma_wr_en),
        .dma_wr_addr(dma_wr_addr),
        .dma_wr_data(dma_wr_data),
        
        // 读端接内部 AGU 和 CU
        .cu_rd_en(cu_rd_en),
        .cu_rd_addr(cu_rd_addr),
        .cu_rd_data(bram_dout)
    );

    // =========================================================================
    // 例化 2: 核心计算逻辑 SlidingWindow_CU
    // =========================================================================
    generate
        if (CU_TYPE == 0) begin : GEN_SLIDING
            assign cu_ready = 1'b1;
            SlidingWindow_CU #(
              .M(M), .N(N), .T_WIDTH(T_WIDTH), .A_PACKED_WIDTH(A_PACKED_WIDTH),
              .B_DATA_BITS(B_DATA_BITS), .PSUM_WIDTH(PSUM_WIDTH), .OUT_WIDTH(OUT_WIDTH),
              .WINDOW_SIZE(16)
            ) u_cu (
                .clk(clk), .rst_n(rst_n),
                .cfg_we_b(cfg_we_b), .cfg_addr_b(cfg_addr_b), .cfg_data_b(cfg_data_b),
                .a_valid(a_valid_d1), .a_payload(bram_dout), .k_offset(current_k_offset),
                .batch_done(batch_done), .matrix_done(matrix_done),
                .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
            );
        end 
        else if (CU_TYPE == 1) begin : GEN_DYNAMIC
            Dynamic_Scoreboard_CU #(
              .M(M), .N(N), .T_WIDTH(T_WIDTH), .A_PACKED_WIDTH(A_PACKED_WIDTH),
              .B_DATA_BITS(B_DATA_BITS), .PSUM_WIDTH(PSUM_WIDTH), .OUT_WIDTH(OUT_WIDTH)
            ) u_cu (
                .clk(clk), .rst_n(rst_n),
                .cfg_we_b(cfg_we_b), .cfg_addr_b(cfg_addr_b), .cfg_data_b(cfg_data_b),
                .a_valid(a_valid_d1), .a_payload(bram_dout), .k_offset(current_k_offset),
                .unit_ready(cu_ready),
                .matrix_done(matrix_done),
                .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
            );
            assign batch_done = 1'b0; //无此信号, 暂时置0
        end 
        else if (CU_TYPE == 2) begin : GEN_STATIC
            Static_Scoreboard_CU #(
              .M(M), .N(N), .T_WIDTH(T_WIDTH), .A_PACKED_WIDTH(A_PACKED_WIDTH),
              .B_DATA_BITS(B_DATA_BITS), .PSUM_WIDTH(PSUM_WIDTH), .OUT_WIDTH(OUT_WIDTH) 
            )u_cu (
                .clk(clk), .rst_n(rst_n),
                .cfg_we_b(cfg_we_b), .cfg_addr_b(cfg_addr_b), .cfg_data_b(cfg_data_b),
                .cfg_we_si(cfg_we_si), .cfg_addr_si(cfg_addr_si), .cfg_data_si(cfg_data_si),
                .a_valid(a_valid_d1), .a_payload(bram_dout), .k_offset(current_k_offset),
                .unit_ready(cu_ready),
                .matrix_done(matrix_done),
                .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
            );
            assign batch_done = 1'b0; //无此信号, 暂时置0
        end 
        else if (CU_TYPE == 3) begin : GEN_DSP
            assign cu_ready = 1'b1;
            DSP_Baseline_CU #(
              .M(M), .N(N), 
              .A_DATA_BITS(4),.B_DATA_BITS(B_DATA_BITS), 
              .PSUM_WIDTH(PSUM_WIDTH), .OUT_WIDTH(OUT_WIDTH) 
            )u_cu (
                .clk(clk), .rst_n(rst_n),
                .cfg_we_b(cfg_we_b), .cfg_addr_b(cfg_addr_b), .cfg_data_b(cfg_data_b),
                .a_valid(dsp_a_valid), .a_payload(dsp_a_payload), 
                .matrix_done(matrix_done),
                .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
            );
            assign batch_done = 1'b0; 
        end
    endgenerate

endmodule