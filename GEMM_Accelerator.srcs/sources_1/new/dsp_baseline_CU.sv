`timescale 1ns / 1ps

module dsp_baseline_CU #(
    parameter M = 256,
    parameter N = 256,                         
    parameter T_WIDTH = 8,               
    parameter A_PACKED_WIDTH = 10 + T_WIDTH, 
    parameter B_DATA_BITS = 8,           // B的数据位宽 (8-bit)
    parameter PSUM_WIDTH = 24,           // 内部累加器位宽
    parameter OUT_WIDTH = 32,            // 输出结果位宽
    parameter OUTPUT_ADDR_WIDTH = $clog2(M),
    parameter BATCH_NUMS = N / T_WIDTH,
    parameter BATCH_SIZE = M * 4
  )(
    input  wire                                 clk,
    input  wire                                 rst_n,

    // --- Configuration Interface B (Vector B) ---
    input  wire                                 cfg_we_b,
    input  wire [7:0]                           cfg_addr_b,
    input  wire signed [B_DATA_BITS-1:0]        cfg_data_b, 

    // --- Runtime Data Stream ---
    input  wire                                 a_valid,
    input  wire unsigned [A_PACKED_WIDTH-1:0]   a_payload,
    input  wire [7:0]                           k_offset,
    output wire                                 cu_ready,
    output wire                                 batch_done,
    output wire                                 matrix_done,

    // --- Output Interface ---
    input  wire                                 out_rd_en,
    input  wire [OUTPUT_ADDR_WIDTH-1:0]         out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]          out_data
);
    assign cu_ready = 1'b1; // 始终准备好接受数据
    // =========================================================================
    // 参数声明
    // =========================================================================
    // --- RAM/BANK 预定义参数 ---
    localparam NUM_B_BANKS    = T_WIDTH;                   // Bank数量必须等于T_WIDTH以保证并行读
    localparam B_BANK_DEPTH   = N / T_WIDTH;               // 每个Bank的深度
    localparam B_BANK_BITS    = $clog2(NUM_B_BANKS);       // Bank选择位宽
    localparam B_DEPTH_BITS   = $clog2(B_BANK_DEPTH);      // 深度地址位宽
    localparam TOTAL_PAYLOADS = BATCH_NUMS * BATCH_SIZE;   

    // --- Stage 1: DSP 计算 ---
    wire                          s1_valid;
    wire [7:0]                    s1_k_offset;
    wire [T_WIDTH-1:0]            s1_current_node; 
    wire [7:0]                    s1_row_idx;   
    wire [1:0]                    s1_bitplane_id;
    wire [7:0]                    b_bank_dout[0:NUM_B_BANKS-1];
    wire [T_WIDTH*8-1 : 0]        s1_b_vec_slice;

    wire signed [PSUM_WIDTH-1:0]  s12_DSP_result;  // dsp 产生的结果有两拍延时

    reg                           s10_valid;         // 第1级
    reg  [T_WIDTH-1:0]            s10_current_node; 
    reg  [7:0]                    s10_row_idx;   
    reg  [1:0]                    s10_bitplane_id;

    reg                           s11_valid;         // 第2级
    reg  [T_WIDTH-1:0]            s11_current_node; 
    reg  [7:0]                    s11_row_idx;   
    reg  [1:0]                    s11_bitplane_id;
    
    reg                           s12_valid;         // 第3级对齐DSP结果
    reg  [T_WIDTH-1:0]            s12_current_node; 
    reg  [7:0]                    s12_row_idx;   
    reg  [1:0]                    s12_bitplane_id;

    reg  signed [PSUM_WIDTH-1:0]  s12_DSP_shifted;
    
    // --- Stage 1 -> Stage 2 寄存器 ---
    reg                           r_s2_valid;
    reg [T_WIDTH-1:0]             r_s2_current_node;
    reg [7:0]                     r_s2_row_idx;  
    reg signed [OUT_WIDTH-1:0]    r_s2_DSP_shifted; 

    // --- Stage 2: APE 计算 ---
    wire signed [OUT_WIDTH-1:0]   s2_APE_result;
    reg  [$clog2(BATCH_SIZE)-1:0] s2_batch_cnt;
    reg  [$clog2(TOTAL_PAYLOADS)-1:0] s2_matrix_cnt;
    wire                          s2_batch_last_data;
    wire                          s2_matrix_last_data;
    reg  [7:0]                    batch_done_shift;
    reg  [7:0]                    matrix_done_shift;

    // =========================================================================
    // Stage 1: 输入数据解包与预处理
    // =========================================================================
    assign s1_valid        = a_valid;
    assign s1_current_node = a_payload[A_PACKED_WIDTH-1:10];
    assign s1_row_idx      = a_payload[9:2];
    assign s1_bitplane_id  = a_payload[1:0];
    assign s1_k_offset     = k_offset;

    // =========================================================================
    // Stage 1 : Vector B Storage (LUTRAM)
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
                .enb            (s1_valid || 1'b1), 
                .addrb          (s1_k_offset[B_DEPTH_BITS-1:0]),
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
            assign s1_b_vec_slice[((NUM_B_BANKS - gi) * 8) - 1 : ((NUM_B_BANKS - gi - 1) * 8)] = b_bank_dout[gi];
        end
    endgenerate
    
    // =========================================================================
    // Stage 1: DSP 计算 (点积)
    // =========================================================================
    dot_product_dsp u_dot_product_8x8_dsp (
        .clk(clk),
        .rst_n(rst_n),
        .node(s1_current_node),
        .int8_vec(s1_b_vec_slice),
        .dot_result(s12_DSP_result)
    );
    
    always @(posedge clk) begin
        if (!rst_n) begin
            s10_valid        <= 0;
            s10_current_node <= 0;
            s10_row_idx      <= 0;
            s10_bitplane_id  <= 0;
            s11_valid        <= 0;
            s11_current_node <= 0;
            s11_row_idx      <= 0;
            s11_bitplane_id  <= 0;
            s12_valid        <= 0;
            s12_current_node <= 0;
            s12_row_idx      <= 0;
            s12_bitplane_id  <= 0;
        end else begin
            // DSP计算结果实际有三拍延时，控制信号增加到打三拍
            s10_valid        <= s1_valid;
            s10_current_node <= s1_current_node; 
            s10_row_idx      <= s1_row_idx;   
            s10_bitplane_id  <= s1_bitplane_id;

            s11_valid        <= s10_valid;
            s11_current_node <= s10_current_node; 
            s11_row_idx      <= s10_row_idx;   
            s11_bitplane_id  <= s10_bitplane_id;

            s12_valid        <= s11_valid;
            s12_current_node <= s11_current_node; 
            s12_row_idx      <= s11_row_idx;   
            s12_bitplane_id  <= s11_bitplane_id;
        end
    end

    // =========================================================================
    // Stage 1: dsp 计算结果移位 
    // =========================================================================
    always @(*) begin
        case (s12_bitplane_id)
            2'd0: s12_DSP_shifted = $signed(s12_DSP_result);
            2'd1: s12_DSP_shifted = $signed(s12_DSP_result) <<< 1;
            2'd2: s12_DSP_shifted = $signed(s12_DSP_result) <<< 2;
            2'd3: s12_DSP_shifted = $signed(s12_DSP_result) <<< 3;
            default: s12_DSP_shifted = $signed(s12_DSP_result);
        endcase
    end

    // =========================================================================
    // STAGE 1 -> Stage 2: Pipeline Registers
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin 
            r_s2_valid        <= 0;
            r_s2_current_node <= 0;
            r_s2_row_idx      <= 0;
            r_s2_DSP_shifted  <= 0;
        end else begin
            r_s2_valid <= s12_valid;
            if (s12_valid) begin
                r_s2_current_node <= s12_current_node;
                r_s2_row_idx      <= s12_row_idx; 
                r_s2_DSP_shifted  <= s12_DSP_shifted;
            end 
        end     
    end

    // =========================================================================
    // STAGE 2: APE
    // =========================================================================
    APE_Window #(
        .PSUM_W(PSUM_WIDTH), 
        .OUT_W(OUT_WIDTH),
        .MAX_ROWS(M)
    ) u_ape (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(r_s2_valid),
        .shifted_psum_in(r_s2_DSP_shifted),
        .row_idx(r_s2_row_idx),
        .ape_result(s2_APE_result),
        .res_rd_en(out_rd_en),
        .res_rd_addr(out_rd_addr),
        .res_rd_data(out_data)
    );
    
    // =========================================================================
    // Batch & Matrix Done Indicator Logic (脉冲产生与自动归零逻辑)
    // =========================================================================
    // 判定当前写入的是否是该批次/矩阵的"最后一个"数据
    assign s2_batch_last_data  = (r_s2_valid && (s2_batch_cnt == BATCH_SIZE - 1));
    assign s2_matrix_last_data = (r_s2_valid && (s2_matrix_cnt == TOTAL_PAYLOADS - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_batch_cnt      <= 0;
            batch_done_shift  <= 8'd0;            
            s2_matrix_cnt     <= 0;
            matrix_done_shift <= 8'd0;
        end else begin
            // --- 1. 批次计数器 (Batch) ---
            if (r_s2_valid) begin
                if (s2_batch_cnt == BATCH_SIZE - 1)
                    s2_batch_cnt <= 0;
                else
                    s2_batch_cnt <= s2_batch_cnt + 1;
            end

            // --- 2. 矩阵计数器 (Matrix) ---
            if (r_s2_valid) begin
                if (s2_matrix_cnt == TOTAL_PAYLOADS - 1)
                    s2_matrix_cnt <= 0; 
                else
                    s2_matrix_cnt <= s2_matrix_cnt + 1;
            end

            // --- 3. 完成信号打拍延迟 (等待 APE 落盘) ---
            batch_done_shift  <= {batch_done_shift[6:0],  s2_batch_last_data};
            matrix_done_shift <= {matrix_done_shift[6:0], s2_matrix_last_data};
        end
    end

    // 提取移位寄存器的最高位，产生单拍脉冲 (Pulse)
    assign batch_done  = batch_done_shift[7];
    assign matrix_done = matrix_done_shift[7];

endmodule