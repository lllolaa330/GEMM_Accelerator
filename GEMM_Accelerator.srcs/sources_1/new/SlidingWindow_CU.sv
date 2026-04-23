`timescale 1ns / 1ps

module SlidingWindow_CU #(
    parameter M                 = 64,
    parameter N                 = 64,                 // 权重形状：M*N
    parameter T_WIDTH           = 8,                  // Bit-reuse 行模式位宽
    parameter WINDOW_SIZE       = 16,                 // 窗口大小
    parameter B_DATA_BITS       = 8,
    parameter PSUM_WIDTH        = 24,
    parameter OUT_WIDTH         = 32,
    parameter A_PACKED_WIDTH    = 10 + T_WIDTH,       // node:[17:10] row_idx:[9:2] bitplane_id:[1:0]            
    parameter BATCH_NUMS        = N / T_WIDTH,
    parameter BATCH_SIZE        = M * 4
)(
    input  wire                                 clk,
    input  wire                                 rst_n,

    // --- Configuration Interface B (Vector B) ---
    input  wire                                 cfg_we_b,
    input  wire [7:0]                           cfg_addr_b,
    input  wire [B_DATA_BITS-1:0]               cfg_data_b, 

    // --- Runtime Data Stream ---
    input  wire                                 a_valid,
    input  wire [A_PACKED_WIDTH-1:0]            a_payload,
    input  wire [$clog2(BATCH_NUMS)-1:0]        k_offset,
    output wire                                 batch_done,
    output wire                                 matrix_done,

    // --- Output Interface ---
    input  wire                                 out_rd_en,
    input  wire [$clog2(M)-1:0]                 out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]          out_data
);

    // =========================================================================
    // 内部信号声明
    // =========================================================================

    // --- RAM/BANK 预定义参数 ---
    localparam NUM_B_BANKS    = T_WIDTH;                   // Bank数量必须等于T_WIDTH以保证并行读
    localparam B_BANK_DEPTH   = N / T_WIDTH;               // 每个Bank的深度
    localparam B_BANK_BITS    = $clog2(NUM_B_BANKS);       // Bank选择位宽
    localparam B_DEPTH_BITS   = $clog2(B_BANK_DEPTH);      // 深度地址位宽
    localparam TOTAL_PAYLOADS = BATCH_NUMS * BATCH_SIZE;   // 理论有效载荷总数

    // --- Stage 1 信号 (Window Buffer) ---
    wire                          s1_valid;
    wire [T_WIDTH-1:0]            s1_current_node;
    wire [T_WIDTH-1:0]            s2_best_node; 
    wire                          s2_best_node_valid; 

    // --- Stage 1 -> Stage 2 寄存器 ---
    reg                           r_s2_valid;
    reg [T_WIDTH-1:0]             r_s2_current_node;
    reg [$clog2(BATCH_NUMS)-1:0]   r_s2_k_offset;
    reg [7:0]                     r_s2_row_idx;            
    reg [1:0]                     r_s2_bitplane_id;

    // --- Stage 2 信号 (PPE 计算与 RAM 读取) ---
    wire [T_WIDTH*8-1 : 0]        s2_b_vec_slice;
    wire [7:0]                    b_bank_dout[0:NUM_B_BANKS-1]; 
    wire [T_WIDTH-1:0]            s2_mask;
    wire [T_WIDTH-1:0]            s2_sign_flag;
    wire                          s2_prefix_data_valid;
    wire signed [PSUM_WIDTH-1:0]  s2_prefix_data;
    wire signed [PSUM_WIDTH-1:0]  s2_PPE_in;
    wire signed [PSUM_WIDTH-1:0]  s2b_PPE_result; 
    reg  signed [OUT_WIDTH-1:0]   s2b_PPE_result_shifted;

    // --- Stage 2.5: Pipeline Registers (旁路前递与时序切断) ---
    reg                           r_s2b_valid;
    reg [T_WIDTH*8-1 : 0]         r_s2b_b_vec_slice;
    reg [T_WIDTH-1:0]             r_s2b_mask;
    reg [T_WIDTH-1:0]             r_s2b_sign_flag;
    reg signed [PSUM_WIDTH-1:0]   r_s2b_PPE_in;
    reg [T_WIDTH-1:0]             r_s2b_current_node;
    reg [7:0]                     r_s2b_row_idx;
    reg [1:0]                     r_s2b_bitplane_id;

    wire fwd_from_s2b;
    wire fwd_from_s3;

    // --- Stage 2.5 -> Stage 3 寄存器 ---
    reg                           r_s3_valid;
    reg [T_WIDTH-1:0]             r_s3_current_node;
    reg [7:0]                     r_s3_row_idx;            
    reg signed [OUT_WIDTH-1:0]    r_s3_PPE_shifted;
    reg signed [PSUM_WIDTH-1:0]   r_s3_PPE_result; 

    // --- Stage 3 信号 (APE 计算) ---
    wire                          s3_batch_last_data;
    wire                          s3_matrix_last_data;
    wire signed [PSUM_WIDTH-1:0]  s3_APE_result;

    // --- Batch Done 计算逻辑 ---
    (* dont_touch = "true" *) reg [$clog2(BATCH_SIZE+1)-1:0] s3_batch_cnt;
    reg [7:0] batch_done_shift;

    // --- Matrix Done 计算逻辑 ---
    (* dont_touch = "true" *) reg [$clog2(TOTAL_PAYLOADS+1)-1:0] s3_matrix_cnt;
    reg [7:0] matrix_done_shift;

    // =========================================================================
    // Stage 1: Window Buffer
    // =========================================================================
    assign s1_valid        = a_valid;
    assign s1_current_node = a_payload[A_PACKED_WIDTH-1:10];

    Window_Buffer #(
        .T_WIDTH(T_WIDTH), 
        .WINDOW_SIZE(WINDOW_SIZE),
        .BATCH_SIZE(BATCH_SIZE)
    ) u_window_buf (
        .clk(clk), 
        .rst_n(rst_n),
        .valid_in(s1_valid),
        .current_node(s1_current_node),
        .best_node(s2_best_node),
        .valid_out(s2_best_node_valid)
    );

    // =========================================================================
    // STAGE 1 -> Stage 2: Pipeline Registers
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin 
            r_s2_valid        <= 0;
            r_s2_current_node <= 0;
            r_s2_k_offset     <= 0;
            r_s2_row_idx      <= 0;
            r_s2_bitplane_id  <= 0;
        end else begin
            r_s2_valid <= s1_valid;
            if (s1_valid) begin
                r_s2_current_node <= a_payload[A_PACKED_WIDTH-1:10];
                r_s2_k_offset     <= k_offset;
                r_s2_row_idx      <= a_payload[9:2];
                r_s2_bitplane_id  <= a_payload[1:0];
            end 
        end     
    end

    // =========================================================================
    // Stage 2 : Vector B Storage (LUTRAM)
    // =========================================================================
    genvar gi;
    generate
        for (gi = 0; gi < NUM_B_BANKS; gi = gi + 1) begin : gen_b_banks
            xpm_memory_sdpram #(
                .ADDR_WIDTH_A        (B_DEPTH_BITS),
                .ADDR_WIDTH_B        (B_DEPTH_BITS),
                .MEMORY_SIZE         (B_BANK_DEPTH * 8),  
                .WRITE_DATA_WIDTH_A  (8),                 
                .READ_DATA_WIDTH_B   (8),                 
                .READ_LATENCY_B      (0),                 // 0延迟以支持组合逻辑穿透
                .WRITE_MODE_B        ("read_first"),
                .MEMORY_PRIMITIVE    ("distributed"),     // 强制使用LUTRAM
                .CLOCKING_MODE       ("common_clock"),
                .MEMORY_INIT_PARAM   ("0")
            ) u_b_ram (
                .clka           (clk),
                .ena            (1'b1),
                .wea            (cfg_we_b && (cfg_addr_b[B_BANK_BITS-1:0] == gi)),
                .addra          (cfg_addr_b[B_BANK_BITS + B_DEPTH_BITS - 1 : B_BANK_BITS]),
                .dina           (cfg_data_b),
                .clkb           (clk),
                .enb            (r_s2_valid || 1'b1), 
                .addrb          (r_s2_k_offset[B_DEPTH_BITS-1:0]),
                .doutb          (b_bank_dout[gi]),
                .regceb         (1'b1),
                .rstb           (1'b0),
                .sleep          (1'b0),
                .injectsbiterra (1'b0),
                .injectdbiterra (1'b0),
                .sbiterrb       (),
                .dbiterrb       ()
            );
            // 拼接输出，送给 Prefix_PE
            assign s2_b_vec_slice[((NUM_B_BANKS - gi) * 8) - 1 : ((NUM_B_BANKS - gi - 1) * 8)] = b_bank_dout[gi];
        end
    endgenerate

    // =========================================================================
    // Stage 2: Prefix Buffer (Dual Port Read LUTRAM)
    // =========================================================================
    Prefix_Buffer_WINDOW #(
        .ADDR_WIDTH(T_WIDTH),
        .DATA_WIDTH(PSUM_WIDTH)
    ) u_prefix_buffer_window (
        .clk(clk), 
        .rst_n(rst_n), 
        .batch_rst(s3_batch_last_data),
        .we(r_s3_valid), 
        .waddr(r_s3_current_node[T_WIDTH-1:0]), 
        .wdata(r_s3_PPE_result), 
        .raddr(s2_best_node[T_WIDTH-1:0]), 
        .rdata(s2_prefix_data),
        .rvalid(s2_prefix_data_valid)
    );

    // =========================================================================
    // Stage 2: 前递判断与 PPE 预处理
    // =========================================================================
    assign s2_mask      = r_s2_current_node ^ s2_best_node;
    assign s2_sign_flag = r_s2_current_node;
    
    // 流水线数据前递 (Bypass) 逻辑
    assign fwd_from_s2b = r_s2b_valid && (r_s2b_current_node == s2_best_node);
    assign fwd_from_s3  = r_s3_valid  && (r_s3_current_node  == s2_best_node);
    
    assign s2_PPE_in = (!s2_best_node_valid) ? {PSUM_WIDTH{1'b0}} :
                       (fwd_from_s2b)        ? s2b_PPE_result :
                       (fwd_from_s3)         ? r_s3_PPE_result :
                                               s2_prefix_data;

    // =========================================================================
    // STAGE 2 -> Stage 2.5: Pipeline Registers
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            r_s2b_valid        <= 0; 
            r_s2b_b_vec_slice  <= 0; 
            r_s2b_mask         <= 0;
            r_s2b_sign_flag    <= 0; 
            r_s2b_PPE_in       <= 0; 
            r_s2b_current_node <= 0;
            r_s2b_row_idx      <= 0; 
            r_s2b_bitplane_id  <= 0;
        end else begin
            r_s2b_valid <= r_s2_valid;
            if (r_s2_valid) begin
                r_s2b_b_vec_slice  <= s2_b_vec_slice;
                r_s2b_mask         <= s2_mask;
                r_s2b_sign_flag    <= s2_sign_flag;
                r_s2b_PPE_in       <= s2_PPE_in;
                r_s2b_row_idx      <= r_s2_row_idx;
                r_s2b_bitplane_id  <= r_s2_bitplane_id;
                r_s2b_current_node <= r_s2_current_node;
            end
        end
    end
    
    (* dont_touch = "true"*) 
    Prefix_PE_Window #(
        .T(T_WIDTH), 
        .D_W(8), 
        .PSUM_W(PSUM_WIDTH)
    ) u_ppe_window (
        .vec_b_slice(r_s2b_b_vec_slice),
        .mask_xor(r_s2b_mask),
        .sign_flag(r_s2b_sign_flag),
        .prefix_in(r_s2b_PPE_in),
        .result_out(s2b_PPE_result)
    );

    always @(*) begin
        case (r_s2b_bitplane_id)
            2'd0: s2b_PPE_result_shifted = $signed(s2b_PPE_result);
            2'd1: s2b_PPE_result_shifted = $signed(s2b_PPE_result) <<< 1; 
            2'd2: s2b_PPE_result_shifted = $signed(s2b_PPE_result) <<< 2;
            2'd3: s2b_PPE_result_shifted = $signed(s2b_PPE_result) <<< 3;
            default: s2b_PPE_result_shifted = $signed(s2b_PPE_result);
        endcase
    end

    // =========================================================================
    // STAGE 2.5 -> Stage 3: Pipeline Registers
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            r_s3_valid       <= 0;
            r_s3_PPE_result  <= 0;
            r_s3_row_idx     <= 0;
            r_s3_PPE_shifted <= 0;
            r_s3_current_node<= 0;
        end else begin
            r_s3_valid <= r_s2b_valid;
            if (r_s2b_valid) begin
                r_s3_PPE_result  <= s2b_PPE_result;
                r_s3_PPE_shifted <= s2b_PPE_result_shifted;
                r_s3_row_idx     <= r_s2b_row_idx;
                r_s3_current_node<= r_s2b_current_node;
            end
        end
    end

    // =========================================================================
    // STAGE 3: APE
    // =========================================================================
    APE_Window #(
        .PSUM_W(PSUM_WIDTH), 
        .OUT_W(OUT_WIDTH),
        .MAX_ROWS(M)
    ) u_ape (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(r_s3_valid),
        .shifted_psum_in(r_s3_PPE_shifted),
        .row_idx(r_s3_row_idx),
        .ape_result(s3_APE_result),
        .res_rd_en(out_rd_en),
        .res_rd_addr(out_rd_addr),
        .res_rd_data(out_data)
    );

    // =========================================================================
    // Batch & Matrix Done Indicator Logic (脉冲产生与自动归零逻辑)
    // =========================================================================
    
    // 判定当前写入的是否是该批次/矩阵的"最后一个"数据
    assign s3_batch_last_data  = (r_s3_valid && (s3_batch_cnt == BATCH_SIZE - 1));
    assign s3_matrix_last_data = (r_s3_valid && (s3_matrix_cnt == TOTAL_PAYLOADS - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_batch_cnt      <= 0;
            batch_done_shift  <= 8'd0;            
            s3_matrix_cnt     <= 0;
            matrix_done_shift <= 8'd0;
        end else begin
            // --- 1. 批次计数器 (Batch) ---
            if (r_s3_valid) begin
                if (s3_batch_cnt == BATCH_SIZE - 1)
                    s3_batch_cnt <= 0; // 到达批次末尾，自动归零，迎接下一批次
                else
                    s3_batch_cnt <= s3_batch_cnt + 1;
            end

            // --- 2. 矩阵计数器 (Matrix) ---
            if (r_s3_valid) begin
                if (s3_matrix_cnt == TOTAL_PAYLOADS - 1)
                    s3_matrix_cnt <= 0; // 整个矩阵算完，自动归零
                else
                    s3_matrix_cnt <= s3_matrix_cnt + 1;
            end

            // --- 3. 完成信号打拍延迟 (等待 APE 落盘) ---
            // 每次最后一个数据进入 APE 时，向移位寄存器推入一个 '1'
            // 8个周期后，顶层会产生一个维持仅仅 1 个时钟周期的脉冲
            batch_done_shift  <= {batch_done_shift[6:0],  s3_batch_last_data};
            matrix_done_shift <= {matrix_done_shift[6:0], s3_matrix_last_data};
        end
    end

    // 提取移位寄存器的最高位，产生单拍脉冲 (Pulse)
    assign batch_done  = batch_done_shift[7];
    assign matrix_done = matrix_done_shift[7];

endmodule