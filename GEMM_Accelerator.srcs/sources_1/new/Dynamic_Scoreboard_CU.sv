`timescale 1ns / 1ps

module Dynamic_Scoreboard_CU #(
    parameter M = 64,
    parameter N = 64,
    parameter T_WIDTH = 8,
    parameter A_PACKED_WIDTH = 10 + T_WIDTH,
    parameter B_DATA_BITS = 8,
    parameter PSUM_WIDTH = 24,
    parameter OUT_WIDTH = 32,
    parameter BATCH_NUMS = N / T_WIDTH,
    parameter BATCH_SIZE = M * 4,
    parameter OFFSET_ADDR_WIDTH = $clog2(BATCH_NUMS),
    parameter OUTPUT_ADDR_WIDTH = $clog2(M),
    parameter SI_SIZE = 2 ** T_WIDTH
)(
    input  wire clk,
    input  wire rst_n,

    // --- Configuration Interface B (Vector B) ---
    input  wire                  cfg_we_b,
    input  wire [$clog2(N)-1:0]  cfg_addr_b,
    input  wire [7:0]            cfg_data_b, 
    
    // --- Runtime Data Stream ---
    input  wire                          a_valid,
    input  wire [A_PACKED_WIDTH-1:0]     a_payload,
    input  wire [OFFSET_ADDR_WIDTH-1:0]  k_offset,
    output wire                          unit_ready,
    output wire                          batch_done,
    output wire                          matrix_done,

    // --- Output Interface ---
    input  wire                                 out_rd_en,
    input  wire [OUTPUT_ADDR_WIDTH-1:0]         out_rd_addr,
    output wire [OUT_WIDTH-1:0]                 out_data
);

    // =========================================================================
    // 信号预声明
    // =========================================================================

    // --- RAM/BANK 预定义参数 ---
    localparam NUM_B_BANKS    = T_WIDTH;                   // Bank数量必须等于T_WIDTH以保证并行读
    localparam B_BANK_DEPTH   = N / T_WIDTH;               // 每个Bank的深度
    localparam B_BANK_BITS    = $clog2(NUM_B_BANKS);       // Bank选择位宽
    localparam B_DEPTH_BITS   = $clog2(B_BANK_DEPTH);      // 深度地址位宽
    localparam TOTAL_PAYLOADS = BATCH_NUMS * BATCH_SIZE;   // 理论有效载荷总数

    ///// Stage 1: Dynamic SI & Sorter & 查 SI /////
    wire                          s1_valid;
    wire [A_PACKED_WIDTH-1:0]     s1_payload;
    wire [T_WIDTH-1:0]            s1_current_node;
    wire [PSUM_WIDTH-1:0]         s1_si_rdata_prefix;
    wire [OFFSET_ADDR_WIDTH-1:0]  s1_k_offset;
    wire                          s1_rst_batch;

    // Stage 1 -> Stage 2 寄存器
    reg                           r_s2_valid;
    reg [T_WIDTH-1:0]             r_s2_current_node;
    reg [T_WIDTH-1:0]             r_s2_prefix_node; 
    reg [OFFSET_ADDR_WIDTH-1:0]   r_s2_k_offset;
    reg [7:0]                     r_s2_row_idx;            // 7 与参数 M 有关
    reg [1:0]                     r_s2_bitplane_id;
    reg                           r_s2_rst_batch;
    
    /////// Stage 2 : PPE 计算 /////
    wire [T_WIDTH*8-1 : 0]        s2_b_vec_slice;
    wire [7:0]                    b_bank_dout[0:NUM_B_BANKS-1]; // 用于接收每个Bank读出的数据
    wire                          s2_is_direct_reuse;    // 是否 Self 命中
    wire                          s2_hit;                // 是否 Prefix 命中
    wire                          s2_pb_rvalid;
    wire signed [PSUM_WIDTH-1:0]  s2_pb_rdata_prefix;
    wire                          s2_pb_rvalid_self;
    wire signed [PSUM_WIDTH-1:0]  s2_pb_rdata_self;
    wire [T_WIDTH-1:0]            s2_mask; 
    wire signed [PSUM_WIDTH-1:0]  s2_PPE_in;
    wire signed [PSUM_WIDTH-1:0]  s2_PPE_result; 

    // Stage 2 -> Stage 3 寄存器
    reg                           r_s3_valid;
    reg [T_WIDTH-1:0]             r_s3_node_id;
    reg [7:0]                     r_s3_row_idx;            // 7 与参数 M 有关
    reg [1:0]                     r_s3_bitplane_id;
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
    // Stage 1: Ping-Pong Sorter & Dynamic Scoreboard Core
    // =========================================================================
    Sorter_and_SI #(
      .T_WIDTH(T_WIDTH),
      .DATA_WIDTH(A_PACKED_WIDTH),
      .PSUM_WIDTH(PSUM_WIDTH),
      .BATCH_SIZE(BATCH_SIZE),
      .BATCH_NUMS(BATCH_NUMS),
      .OFFSET_ADDR_WIDTH(OFFSET_ADDR_WIDTH),
      .SI_SIZE(SI_SIZE)
    ) u_sorter_and_si (
      // input
      .clk(clk), .rst_n(rst_n),
      .in_valid(a_valid),
      .in_data(a_payload),
      .in_k_offset(k_offset),
      // output
      .in_ready(unit_ready),    // 准备好接收 a_payload 
      .out_valid(s1_valid),
      .out_data(s1_payload),
      .out_prefix_data(s1_si_rdata_prefix),
      .out_k_offset(s1_k_offset),
      .batch_done(s1_rst_batch)  // 完成输出 s1_payload
    );

    assign s1_current_node = s1_payload [A_PACKED_WIDTH-1:A_PACKED_WIDTH-T_WIDTH];

    // =========================================================================
    // STAGE 1 -> Stage 2: Reg
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin 
            r_s2_valid                 <= 0;
            r_s2_current_node          <= 0;
            r_s2_prefix_node           <= 0;
            r_s2_k_offset              <= 0;
            r_s2_row_idx               <= 0;
            r_s2_bitplane_id           <= 0;
            r_s2_rst_batch             <= 0;
        end else begin
            r_s2_valid     <= s1_valid;
            r_s2_rst_batch <= s1_rst_batch;
            if (s1_valid) begin
                r_s2_current_node          <= s1_current_node;
                r_s2_prefix_node           <= s1_si_rdata_prefix;
                r_s2_k_offset              <= s1_k_offset;
                r_s2_row_idx               <= s1_payload[9:2];
                r_s2_bitplane_id           <= s1_payload[1:0];
            end 
        end     
    end

    // =========================================================================
    // STAGE 2 -> Stage 2b: RAM 读出对齐流水线 (打拍)
    // =========================================================================
    reg                           r_s2b_valid;
    reg [T_WIDTH-1:0]             r_s2b_current_node;
    reg [T_WIDTH-1:0]             r_s2b_prefix_node; 
    reg [7:0]                     r_s2b_row_idx;            
    reg [1:0]                     r_s2b_bitplane_id;

    always @(posedge clk) begin
        if (!rst_n) begin
            r_s2b_valid        <= 0;
            r_s2b_current_node <= 0;
            r_s2b_prefix_node  <= 0;
            r_s2b_row_idx      <= 0;
            r_s2b_bitplane_id  <= 0;
        end else begin
            r_s2b_valid <= r_s2_valid;
            if (r_s2_valid) begin
                r_s2b_current_node <= r_s2_current_node;
                r_s2b_prefix_node  <= r_s2_prefix_node;
                r_s2b_row_idx      <= r_s2_row_idx;
                r_s2b_bitplane_id  <= r_s2_bitplane_id;
            end
        end
    end
    
    // =========================================================================
    // Stage 2: Prefix Buffer (Dual Port Read)
    // =========================================================================
    Prefix_Buffer_LUTRAM #(
        .ADDR_WIDTH(T_WIDTH),
        .DATA_WIDTH(PSUM_WIDTH)
    ) u_prefix_buffer (
        .clk(clk), .rst_n(rst_n), 
        .soft_clear(r_s2_rst_batch),
        // --- Write Port (来自 Stage 3) ---
        .we(r_s3_valid), 
        .waddr(r_s3_node_id[T_WIDTH-1:0]), 
        .wdata(r_s3_PPE_result), 
        // Port 1: 查 Prefix (由 Scoreboard 提供)
        .raddr(r_s2b_prefix_node[T_WIDTH-1:0]), 
        .rdata(s2_pb_rdata_prefix),
        .rvalid(s2_pb_rvalid),
        // Port 2: 查 Current Node (自检)
        .raddr_check(r_s2b_current_node[T_WIDTH-1:0]),
        .rdata_check(s2_pb_rdata_self),
        .rvalid_check(s2_pb_rvalid_self)
    );

    assign s2_is_direct_reuse  = s2_pb_rvalid_self;
    assign s2_hit              = (!s2_pb_rvalid_self) && s2_pb_rvalid;

    // =========================================================================
    // Stage 2 : Vector B Storage
    // =========================================================================
    
    genvar gi;
    generate
        for (gi = 0; gi < NUM_B_BANKS; gi = gi + 1) begin : gen_b_banks
            xpm_memory_sdpram #(
                .ADDR_WIDTH_A        (B_DEPTH_BITS),
                .ADDR_WIDTH_B        (B_DEPTH_BITS),
                .MEMORY_SIZE         (B_BANK_DEPTH * 8),  // 单个Bank的总比特数
                .WRITE_DATA_WIDTH_A  (8),                 // 每次写入1个int8
                .READ_DATA_WIDTH_B   (8),                 // 每次读出1个int8
                .READ_LATENCY_B      (1),                 // 开启1拍延迟切断组合逻辑
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

    // 1. Mask 逻辑
    // Direct Reuse -> Mask=0; Hit -> Mask=XOR; Miss -> Mask=Current
    assign s2_mask = s2_is_direct_reuse ? {T_WIDTH{1'b0}} : 
                     (s2_hit            ? (r_s2b_current_node ^ r_s2b_prefix_node) : 
                                           r_s2b_current_node);
    
    // 2. Prefix MUX 逻辑
    // Direct Reuse -> Use self_Val;Hit -> Use prefix_Val; Miss -> Use 0
    assign s2_PPE_in = s2_is_direct_reuse ? s2_pb_rdata_self : 
                      (s2_hit             ? s2_pb_rdata_prefix : 
                                            {PSUM_WIDTH{1'b0}});
    
    Prefix_PE #(
        .T(T_WIDTH), 
        .D_W(B_DATA_BITS),
        .PSUM_W(PSUM_WIDTH)
    ) u_prefix_pe (
        .vec_b_slice(s2_b_vec_slice),
        .mask(s2_mask), 
        .prefix_in(s2_PPE_in), 
        .result_out(s2_PPE_result)  
    );

    // =========================================================================
    // STAGE 2 -> Stage 3: Reg
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            r_s3_valid       <= 0;
            r_s3_PPE_result  <= 0;
            r_s3_node_id     <= 0;
            r_s3_row_idx     <= 0;
            r_s3_bitplane_id <= 0;
        end else begin
            r_s3_valid <= r_s2b_valid;
            if (r_s2b_valid) begin
                r_s3_PPE_result  <= s2_PPE_result;
                r_s3_node_id     <= r_s2b_current_node;
                r_s3_row_idx     <= r_s2b_row_idx;
                r_s3_bitplane_id <= r_s2b_bitplane_id;
            end
        end
    end

    // =========================================================================
    // STAGE 3: APE
    // =========================================================================
    Accumulation_PE #(
        .PSUM_W(PSUM_WIDTH), 
        .OUT_W(OUT_WIDTH),
        .MAX_ROWS(M)
    ) u_ape (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(r_s3_valid),
        .partial_sum(r_s3_PPE_result),
        .row_idx(r_s3_row_idx),
        .bitplane_id(r_s3_bitplane_id),
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