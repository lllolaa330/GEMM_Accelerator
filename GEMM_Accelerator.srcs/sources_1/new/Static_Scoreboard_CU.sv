`timescale 1ns / 1ps

module Static_Scoreboard_CU #(
    parameter M = 256,
    parameter N = 256,                         // 权重形状：M*N
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
    input  wire        cfg_we_b,
    input  wire [7:0]  cfg_addr_b,
    input  wire [7:0]  cfg_data_b, 
    
    // --- Configuration Interface SI (Scoreboard) ---
    input  wire               cfg_we_si,
    input  wire [T_WIDTH-1:0] cfg_addr_si,
    input  wire [T_WIDTH-1:0] cfg_data_si,

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

    reg                         h_input_sel;
    reg                         h_output_sel; 
    reg [OFFSET_ADDR_WIDTH-1:0] h0_k_offset_reg;
    reg [OFFSET_ADDR_WIDTH-1:0] h1_k_offset_reg;
    wire                        h0_in_ready, h1_in_ready;
    wire                        h0_out_valid, h1_out_valid;
    wire [A_PACKED_WIDTH-1:0]   h0_out_data, h1_out_data;
    wire                        h0_batch_done, h1_batch_done;
    reg                         h0_batch_done_q,h1_batch_done_q;
    wire                        h0_done_pulse;
    wire                        h1_done_pulse;
    wire                        h0_valid_in, h1_valid_in;
    wire                        h0_accepting_first_data;
    wire                        h1_accepting_first_data;

    // Stage 1: Sorter & 查 SI
    wire [OFFSET_ADDR_WIDTH-1:0]  s1_dispatch_k_offset;
    wire                          s1_sorted_valid;
    wire [A_PACKED_WIDTH-1:0]     s1_sorted_payload;
    wire                          s1_sorter_soft_rst_batch;
    wire [T_WIDTH-1:0]            s1_current_node;
    wire [T_WIDTH-1:0]            s1_si_rdata_prefix;       // 来自 SI RAM

    // Stage 1 -> Stage 2 寄存器
    reg                           r_s2_valid;
    reg [T_WIDTH-1:0]             r_s2_current_node;
    reg [T_WIDTH-1:0]             r_s2_prefix_node; 
    reg [OFFSET_ADDR_WIDTH-1:0]   r_s2_k_offset;
    reg [7:0]                     r_s2_row_idx;            // 7 与参数 M 有关
    reg [1:0]                     r_s2_bitplane_id;
    reg                           r_s2_sorter_soft_rst_batch;
    
    // Stage 2 : PPE 计算
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

    // Stage 3: APE 计算
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
    // Sorter 的 Ping-Pong 控制逻辑
    // =========================================================================
    assign h0_valid_in = a_valid && (h_input_sel == 1'b0);
    assign h1_valid_in = a_valid && (h_input_sel == 1'b1);
    assign h0_done_pulse = h0_batch_done && !h0_batch_done_q;
    assign h1_done_pulse = h1_batch_done && !h1_batch_done_q;
    // k_offset
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h0_k_offset_reg <= {OFFSET_ADDR_WIDTH{1'b0}};
            h1_k_offset_reg <= {OFFSET_ADDR_WIDTH{1'b0}};
        end 
        else begin
            // 直接判断有效、就绪，并且恰好是第一笔数据时，锁存 k_offset
            if (h0_valid_in && h0_in_ready && h0_accepting_first_data) 
                h0_k_offset_reg <= k_offset;
            if (h1_valid_in && h1_in_ready && h1_accepting_first_data)
                h1_k_offset_reg <= k_offset;
        end
    end
    // h_input_sel
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_input_sel <= 1'b0;
        end else begin
            // 当当前选中的 Sorter 满了（ready变低），且另一个 Sorter 空闲时切换
            if (h_input_sel == 1'b0) begin
              if (!h0_in_ready && h1_in_ready) h_input_sel <= 1'b1;
              else h_input_sel <= 1'b0;
            end
            else if (h_input_sel == 1'b1) begin
              if(!h1_in_ready && h0_in_ready) h_input_sel <= 1'b0;
              else h_input_sel <= 1'b1;
            end 
            else h_input_sel <= 1'b0;
        end
    end
    // h0_batch_done, h1_batch_done
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h0_batch_done_q <= 1'b0;
            h1_batch_done_q <= 1'b0;
        end else begin
            h0_batch_done_q <= h0_batch_done; // 延迟一拍
            h1_batch_done_q <= h1_batch_done; // 延迟一拍
        end
    end
    // h_output_sel
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_output_sel <= 1'b0;
        end else begin
            // 当 h0 彻底完成这一批次的输出时，切到 h1
            if (h_output_sel == 1'b0) begin
              if (h0_done_pulse) h_output_sel <= 1'b1;
              else h_output_sel <= 1'b0;
            end
            // 当 h1 彻底完成这一批次的输出时，切回 h0
            else if (h_output_sel == 1'b1) begin
              if(h1_done_pulse) h_output_sel <= 1'b0;
              else h_output_sel <= 1'b1;
            end 
        end
    end
    // unit_ready(对外 ready 信号)
    assign unit_ready = (h_input_sel == 1'b0) ? h0_in_ready : h1_in_ready;

    // =========================================================================
    // Module: Hamming Sorter (Input Processing)
    // =========================================================================
    Hamming_Sorter_Static #(
        .T_WIDTH(T_WIDTH), .DATA_WIDTH(A_PACKED_WIDTH), .BATCH_SIZE(BATCH_SIZE)
    ) u_h0_sorter (
        .clk(clk), .rst_n(rst_n),
        .in_valid(h0_valid_in), .in_data(a_payload), .in_ready(h0_in_ready),
        .out_valid(h0_out_valid), .out_data(h0_out_data), .out_ready(h_output_sel == 1'b0),
        .soft_rst_batch_done(h0_batch_done),
        .accepting_first_data(h0_accepting_first_data)
    );

    Hamming_Sorter_Static #(
        .T_WIDTH(T_WIDTH), .DATA_WIDTH(A_PACKED_WIDTH), .BATCH_SIZE(BATCH_SIZE)
    ) u_h1_sorter (
        .clk(clk), .rst_n(rst_n),
        .in_valid(h1_valid_in), .in_data(a_payload), .in_ready(h1_in_ready),
        .out_valid(h1_out_valid), .out_data(h1_out_data), .out_ready(h_output_sel == 1'b1),
        .soft_rst_batch_done(h1_batch_done),
        .accepting_first_data(h1_accepting_first_data)
    );

    // 输出汇聚逻辑
    assign s1_sorted_valid   = (h_output_sel == 1'b0) ? h0_out_valid : h1_out_valid;
    assign s1_sorted_payload = (h_output_sel == 1'b0) ? h0_out_data  : h1_out_data;
    assign s1_sorter_soft_rst_batch = h0_batch_done | h1_batch_done;
    assign s1_dispatch_k_offset = (h_output_sel == 1'b0) ? h0_k_offset_reg : h1_k_offset_reg;
    assign s1_current_node = s1_sorted_payload[A_PACKED_WIDTH-1:10];

    // =========================================================================
    // Stage 1: Scoreboard Info (SI) RAM
    // =========================================================================
    xpm_memory_sdpram #(
        .ADDR_WIDTH_A        (T_WIDTH),         // 深度
        .ADDR_WIDTH_B        (T_WIDTH),
        .MEMORY_SIZE         (SI_SIZE * T_WIDTH),// 总比特数 (例如 256*8 = 2048)
        .WRITE_DATA_WIDTH_A  (T_WIDTH),         // 显式指定写位宽，防止默认为 32
        .READ_DATA_WIDTH_B   (T_WIDTH),         // 显式指定读位宽，防止默认为 32
        .BYTE_WRITE_WIDTH_A  (T_WIDTH),         // 不使用字节使能，设为与位宽一致
        .READ_LATENCY_B      (0),               // 0 延迟，配合 distributed
        .WRITE_MODE_B        ("read_first"),    // 分布式 RAM 必须设为 read_first
        .MEMORY_PRIMITIVE    ("distributed"),   // 使用 LUT 资源
        .CLOCKING_MODE       ("common_clock"),
        .MEMORY_INIT_PARAM   ("0")              // 初始化为 0
    ) u_si_ram (
        .clka           (clk),
        .ena            (1'b1), 
        .wea            (cfg_we_si),
        .addra          (cfg_addr_si),
        .dina           (cfg_data_si),
        
        .clkb           (clk),
        .enb            (s1_sorted_valid),
        .addrb          (s1_current_node),
        .doutb          (s1_si_rdata_prefix),
        
        .regceb         (1'b1),
        .rstb           (1'b0),
        .sleep          (1'b0),
        .injectsbiterra (1'b0),
        .injectdbiterra (1'b0),
        .sbiterrb       (),
        .dbiterrb       ()
    );

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
            r_s2_sorter_soft_rst_batch <= 0;
        end else begin
            r_s2_valid <= s1_sorted_valid;
            r_s2_sorter_soft_rst_batch <= s1_sorter_soft_rst_batch;
            if (s1_sorted_valid) begin
                r_s2_current_node          <= s1_current_node;
                r_s2_prefix_node           <= s1_si_rdata_prefix;
                r_s2_k_offset              <= s1_dispatch_k_offset;
                r_s2_row_idx               <= s1_sorted_payload[9:2];
                r_s2_bitplane_id           <= s1_sorted_payload[1:0];
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
        .soft_clear(r_s2_sorter_soft_rst_batch),
        // --- Write Port (来自 Stage 3) ---
        .we(r_s3_valid), 
        .waddr(r_s3_node_id[T_WIDTH-1:0]), 
        .wdata(r_s3_PPE_result), 
        // Port 1: 查 Prefix (由 Scoreboard 提供)
        .raddr(r_s2_prefix_node[T_WIDTH-1:0]), 
        .rdata(s2_pb_rdata_prefix),
        .rvalid(s2_pb_rvalid),
        // Port 2: 查 Current Node (自检)
        .raddr_check(r_s2_current_node[T_WIDTH-1:0]),
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

    // 1. Mask 逻辑
    // Direct Reuse -> Mask=0; Hit -> Mask=XOR; Miss -> Mask=Current
    assign s2_mask = s2_is_direct_reuse ? {T_WIDTH{1'b0}} : 
                     (s2_hit            ? (r_s2_current_node ^ r_s2_prefix_node) : 
                                           r_s2_current_node);
    
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
            r_s3_valid <= r_s2_valid;
            if (r_s2_valid) begin
                r_s3_PPE_result  <= s2_PPE_result;
                r_s3_node_id     <= r_s2_current_node;
                r_s3_row_idx     <= r_s2_row_idx;
                r_s3_bitplane_id <= r_s2_bitplane_id;
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