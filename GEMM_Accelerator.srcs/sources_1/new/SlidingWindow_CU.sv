`timescale 1ns / 1ps

module SlidingWindow_CU #(
    parameter M = 64,
    parameter N = 64,                         // 权重形状：M*N
    parameter T_WIDTH = 8,
    parameter A_PACKED_WIDTH = 10 + T_WIDTH,
    parameter B_DATA_BITS = 8,
    parameter PSUM_WIDTH = 24,
    parameter OUT_WIDTH = 32,
    parameter BATCH_NUMS = N / T_WIDTH,
    parameter BATCH_SIZE = M * 4,
    parameter WINDOW_SIZE = 16,                // 窗口大小
    parameter OFFSET_ADDR_WIDTH = $clog2(BATCH_NUMS),
    parameter OUTPUT_ADDR_WIDTH = $clog2(M),
    parameter SI_SIZE = 2 ** T_WIDTH
)(
    input  wire clk,
    input  wire rst_n,

    // --- Configuration Interface B (Vector B) ---
    input  wire        cfg_we_b,
    input  wire [7:0]  cfg_addr_b,
    input  wire [7:0]  cfg_data_b, 

    // --- Runtime Data Stream ---
    input  wire                          a_valid,
    input  wire [A_PACKED_WIDTH-1:0]     a_payload,
    input  wire [OFFSET_ADDR_WIDTH-1:0]  k_offset,
    output wire                          matrix_done,

    // --- Output Interface ---
    input  wire                                 out_rd_en,
    input  wire [OUTPUT_ADDR_WIDTH-1:0]         out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]          out_data
);

    // =========================================================================
    // 信号预声明
    // =========================================================================
    // Stage 1: 查 Window Buffer
    wire                          s1_valid;
    wire [T_WIDTH-1:0]            s1_current_node;
    wire [T_WIDTH-1:0]            s2_best_node; 
    wire                          s2_best_node_valid;    // 每个batch第一个数找不到prefix

    // Stage 1 -> Stage 2 寄存器
    reg                           r_s2_valid;
    reg [T_WIDTH-1:0]             r_s2_current_node;
    reg [OFFSET_ADDR_WIDTH-1:0]   r_s2_k_offset;
    reg [7:0]                     r_s2_row_idx;            // 7 与参数 M 有关
    reg [1:0]                     r_s2_bitplane_id;

    // Stage 2：PPE 计算
    wire [T_WIDTH*8-1 : 0]        s2_b_vec_slice;
    wire [7:0]                    b_bank_dout[0:NUM_B_BANKS-1]; // 用于接收每个Bank读出的数据
    wire [T_WIDTH-1:0]            s2_mask;
    wire [T_WIDTH-1:0]            s2_sign_flag;
    wire                          s2_prefix_data_valid;
    wire signed [PSUM_WIDTH-1:0]  s2_prefix_data;
    wire signed [PSUM_WIDTH-1:0]  s2_PPE_in;
    wire signed [PSUM_WIDTH-1:0]  s2_PPE_result; 
    reg signed [OUT_WIDTH-1:0]    s2_PPE_result_shifted;

    // STAGE 2.5: Pipeline Registers (切断 RAM读取 和 PPE加法树 的时序)
    reg                           r_s2b_valid;
    reg [T_WIDTH*8-1 : 0]         r_s2b_b_vec_slice;
    reg [T_WIDTH-1:0]             r_s2b_mask;
    reg [T_WIDTH-1:0]             r_s2b_sign_flag;
    reg signed [PSUM_WIDTH-1:0]   r_s2b_PPE_in;

    // 以下信号也要跟着打一拍，保证与数据对齐
    reg [T_WIDTH-1:0]             r_s2b_current_node;
    reg [7:0]                     r_s2b_row_idx;
    reg [1:0]                     r_s2b_bitplane_id;

    // Stage 2 -> Stage 3 寄存器
    reg                           r_s3_valid;
    reg [T_WIDTH-1:0]             r_s3_current_node;
    reg [7:0]                     r_s3_row_idx;            // 7 与参数 M 有关
    reg signed [OUT_WIDTH-1:0]    r_s3_PPE_shifted;
    reg signed [PSUM_WIDTH-1:0]   r_s3_PPE_result; 

    // Stage 3: APE 计算
    wire signed [PSUM_WIDTH-1:0]  s3_APE_result;

    // =========================================================================
    // Stage 1: Window Buffer
    // =========================================================================
    assign s1_valid        = a_valid;
    assign s1_current_node = a_payload[A_PACKED_WIDTH-1:10];
    assign s1_k_offset     = k_offset;

    // Window_Buffer 会消耗一拍
    Window_Buffer #(.T_WIDTH(T_WIDTH), .WINDOW_SIZE(WINDOW_SIZE),.BATCH_SIZE(BATCH_SIZE)) 
    u_window_buf (
        .clk(clk), .rst_n(rst_n),
        .valid_in(a_valid),
        .current_node(a_payload[A_PACKED_WIDTH-1:10]),
        .best_node(s2_best_node),
        .valid_out(s2_best_node_valid)
    );

    // =========================================================================
    // STAGE 1 -> Stage 2: Reg
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin 
            r_s2_valid                 <= 0;
            r_s2_current_node          <= 0;
            r_s2_k_offset              <= 0;
            r_s2_row_idx               <= 0;
            r_s2_bitplane_id           <= 0;
        end else begin
            r_s2_valid <= s1_valid;
            if (s1_valid) begin
                r_s2_current_node          <= a_payload[A_PACKED_WIDTH-1:10];
                r_s2_k_offset              <= k_offset;
                r_s2_row_idx               <= a_payload[9:2];
                r_s2_bitplane_id           <= a_payload[1:0];
            end 
        end     
    end

    // =========================================================================
    // Stage 2 : Vector B Storage
    // =========================================================================
    localparam MAX_B_ELEMENTS = N;                         // B向量最大容量
    localparam NUM_B_BANKS    = T_WIDTH;                   // Bank数量必须等于T_WIDTH以保证并行读
    localparam B_BANK_DEPTH   = MAX_B_ELEMENTS / T_WIDTH;  // 每个Bank的深度
    localparam B_DEPTH_BITS   = $clog2(B_BANK_DEPTH);      // 深度地址位宽 (比如深度8 -> 3位)
    localparam B_BANK_BITS    = $clog2(NUM_B_BANKS);       // Bank选择位宽 (比如8个Bank -> 3位)
    
    genvar gi;
    generate
        for (gi = 0; gi < NUM_B_BANKS; gi = gi + 1) begin : gen_b_banks
            xpm_memory_sdpram #(
                .ADDR_WIDTH_A        (B_DEPTH_BITS),
                .ADDR_WIDTH_B        (B_DEPTH_BITS),
                .MEMORY_SIZE         (B_BANK_DEPTH * 8),  // 单个Bank的总比特数
                .WRITE_DATA_WIDTH_A  (8),                 // 每次写入1个int8
                .READ_DATA_WIDTH_B   (8),                 // 每次读出1个int8
                .READ_LATENCY_B      (0),                 // 0延迟以支持组合逻辑穿透
                .WRITE_MODE_B        ("read_first"),
                .MEMORY_PRIMITIVE    ("distributed"),     // 强制使用LUTRAM
                .CLOCKING_MODE       ("common_clock"),
                .MEMORY_INIT_PARAM   ("0")
            ) u_b_ram (
                .clka           (clk),
                .ena            (1'b1),
                
                // --- 【写逻辑】 ---
                // cfg_addr 的低位用于选择进入哪个 Bank，高位用于 Bank 内的深度寻址
                .wea            (cfg_we_b && (cfg_addr_b[B_BANK_BITS-1:0] == gi)),
                .addra          (cfg_addr_b[B_BANK_BITS + B_DEPTH_BITS - 1 : B_BANK_BITS]),
                .dina           (cfg_data_b),

                .clkb           (clk),
                .enb            (r_s2_valid || 1'b1), // 运行时读使能
                
                // --- 【读逻辑】 ---
                // 因为 k_offset 的定义现在是块索引 (0, 1, 2...)
                // k_offset=0时，所有Bank读深度0的数据；k_offset=1时读深度1的数据
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
            // --- 【拼接输出】 ---
            // 把每个 Bank 读出的 8-bit 拼接到 64/32-bit 的 s2_b_vec_slice 里面，送给 Prefix_PE
            assign s2_b_vec_slice[((NUM_B_BANKS - gi) * 8) - 1 : ((NUM_B_BANKS - gi - 1) * 8)] = b_bank_dout[gi];
        end
    endgenerate

    // =========================================================================
    // Stage 2: Prefix Buffer (Dual Port Read)
    // =========================================================================
    Prefix_Buffer_WINDOW #(
        .ADDR_WIDTH(T_WIDTH),
        .DATA_WIDTH(PSUM_WIDTH),
        .BATCH_SIZE(BATCH_SIZE)
    ) u_prefix_buffer_window (
        .clk(clk), .rst_n(rst_n), 
        // --- Write Port (来自 Stage 3) ---
        .we(r_s3_valid), 
        .waddr(r_s3_current_node[T_WIDTH-1:0]), 
        .wdata(r_s3_PPE_result), 
        // Port 1: 查 Prefix 
        .raddr(s2_best_node[T_WIDTH-1:0]), 
        .rdata(s2_prefix_data),
        .rvalid(s2_prefix_data_valid)
    );

    // =========================================================================
    // Stage 2: PPE
    // =========================================================================
    assign s2_mask      = r_s2_current_node ^ s2_best_node;
    assign s2_sign_flag = r_s2_current_node;
    // --- 【新增：流水线数据前递 (Bypass) 逻辑】 ---
    // 检查当前需要的 best_node 是否正在后续的流水线中还没落盘
    wire fwd_from_s2b = r_s2b_valid && (r_s2b_current_node == s2_best_node);
    wire fwd_from_s3  = r_s3_valid  && (r_s3_current_node  == s2_best_node);
    // 数据选择优先级：如果无效直接给0 -> 刚刚算完的(S2b) -> 准备写入RAM的(S3) -> RAM里读出来的旧数据
    assign s2_PPE_in = (!s2_best_node_valid) ? {PSUM_WIDTH{1'b0}} :
                       (fwd_from_s2b)        ? s2_PPE_result :
                       (fwd_from_s3)         ? r_s3_PPE_result :
                                               s2_prefix_data;
    // =========================================================================
    // STAGE 2 -> Stage 2.5: Reg
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            r_s2b_valid <= 0; r_s2b_b_vec_slice <= 0; r_s2b_mask <= 0;
            r_s2b_sign_flag <= 0; r_s2b_PPE_in <= 0; r_s2b_current_node <= 0;
            r_s2b_row_idx <= 0; r_s2b_bitplane_id <= 0;
        end else begin
            r_s2b_valid <= r_s2_valid;
            if (r_s2_valid) begin
                // 把异步读出的 RAM 数据和组合逻辑掩码在这里先存下来
                r_s2b_b_vec_slice  <= s2_b_vec_slice;
                r_s2b_mask         <= s2_mask;
                r_s2b_sign_flag    <= s2_sign_flag;
                r_s2b_PPE_in       <= s2_PPE_in;
                // 同步传递控制信号
                r_s2b_row_idx      <= r_s2_row_idx;
                r_s2b_bitplane_id  <= r_s2_bitplane_id;
                r_s2b_current_node <= r_s2_current_node;
            end
        end
    end
    
    (* dont_touch = "true"*) 
    Prefix_PE_Window #(
      .T(T_WIDTH), 
      .D_W(B_DATA_BITS), 
      .PSUM_W(PSUM_WIDTH)
      ) u_ppe_window (
        .vec_b_slice(r_s2b_b_vec_slice),
        .mask_xor(r_s2b_mask),
        .sign_flag(r_s2b_sign_flag),
        .prefix_in(r_s2b_PPE_in),
        .result_out(s2_PPE_result)
    );

    always @(*) begin
        case (r_s2b_bitplane_id)
            2'd0: s2_PPE_result_shifted = $signed(s2_PPE_result);
            2'd1: s2_PPE_result_shifted = $signed(s2_PPE_result) <<< 1; 
            2'd2: s2_PPE_result_shifted = $signed(s2_PPE_result) <<< 2;
            2'd3: s2_PPE_result_shifted = $signed(s2_PPE_result) <<< 3;
            default: s2_PPE_result_shifted = $signed(s2_PPE_result);
        endcase
    end

    // =========================================================================
    // STAGE 2.5 -> Stage 3: Reg
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
                r_s3_PPE_result  <= s2_PPE_result;
                r_s3_PPE_shifted <= s2_PPE_result_shifted;
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
    // Matrix Done Indicator
    // =========================================================================
    // 整个矩阵的理论有效载荷总数
    localparam TOTAL_PAYLOADS = BATCH_NUMS * BATCH_SIZE;
    
    (* dont_touch = "true" *) 
    reg[$clog2(TOTAL_PAYLOADS+1)-1:0] s3_processed_cnt;
    reg [7:0] done_shift;
    wire is_all_processed = (s3_processed_cnt >= TOTAL_PAYLOADS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s3_processed_cnt <= 0;
            done_shift       <= 8'd0;
        end else begin
            // 统计成功进入 Stage 3 (APE) 的有效数据个数
            if (r_s3_valid && !is_all_processed) begin
                s3_processed_cnt <= s3_processed_cnt + 1;
            end

            // 当全部数据都送入 APE 后，启动移位打拍，等待 APE 计算并写入 RAM 完全结束
            if (is_all_processed) begin
                done_shift <= {done_shift[6:0], 1'b1};
            end else begin
                done_shift <= 8'd0;
            end
        end
    end

    // 延迟 8 个周期后拉高，绝对确保流水线的最后一拍已经安全落盘
    assign matrix_done = done_shift[7];


endmodule