`timescale 1ns / 1ps

// 强制综合工具将模块内部的乘法和加法树尽可能映射到 DSP 切片中
(* use_dsp = "yes" *)
module dot_product_dsp #(
    parameter T_WIDTH     = 8,   // 向量长度（同时决定 node 位宽）
    parameter B_DATA_BITS = 8,   // INT8 的位宽
    parameter PSUM_WIDTH  = 24   // 点积结果位宽
)(
    input  wire                               clk,
    input  wire                               rst_n,
    
    // 输入接口
    input  wire [T_WIDTH-1:0]                 node,       // T_WIDTH位的二进制 flag (如 00001011)
    input  wire [T_WIDTH*B_DATA_BITS-1:0]     int8_vec,   // 拼接的 B 向量数据
    
    // 输出接口
    output wire signed [PSUM_WIDTH-1:0]       dot_result  // 点积结果
);

    // =========================================================================
    // 1. 数据解包 (将平铺的位宽切片解包为 T_WIDTH 个独立的有符号数)
    // =========================================================================
    wire signed [B_DATA_BITS-1:0] b_in [0:T_WIDTH-1];
    genvar i;
    generate
        for (i = 0; i < T_WIDTH; i = i + 1) begin : gen_unpack
            assign b_in[i] = int8_vec[i * B_DATA_BITS +: B_DATA_BITS];
        end
    endgenerate

    // =========================================================================
    // Stage 1: 输入寄存器打拍 (映射到 DSP 的 A 和 B 寄存器，阻断输入毛刺)
    // =========================================================================
    reg [T_WIDTH-1:0]            node_r;
    reg signed [B_DATA_BITS-1:0] b_r [0:T_WIDTH-1];
    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            node_r <= {T_WIDTH{1'b0}};
            for (j = 0; j < T_WIDTH; j = j + 1) begin
                b_r[j] <= 0;
            end
        end else begin
            node_r <= node;
            for (j = 0; j < T_WIDTH; j = j + 1) begin
                b_r[j] <= b_in[j];
            end
        end
    end

    // =========================================================================
    // Stage 2: 并行乘法层 (强制映射到 DSP 的乘法器 M Reg)
    // =========================================================================
    // 乘积直接放入 PSUM_WIDTH 位宽的寄存器中，让综合工具自动完成位宽扩展
    reg signed [PSUM_WIDTH-1:0] prod [0:T_WIDTH-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (j = 0; j < T_WIDTH; j = j + 1) begin
                prod[j] <= 0;
            end
        end else begin
            for (j = 0; j < T_WIDTH; j = j + 1) begin
                // 【核心细节】: {1'b0, node_r[j]} 强制将单bit转为正有符号数 (0或1)
                // 确保有符号乘法正确执行，并且能够无缝塞进 DSP 的预加器/乘法器
                prod[j] <= $signed({1'b0, node_r[j]}) * b_r[j];
            end
        end
    end

    // =========================================================================
    // Stage 3: 参数化加法树与流水线输出 (映射到 P Reg 及 LUT 加法树)
    // =========================================================================
    reg signed [PSUM_WIDTH-1:0] sum_comb;
    reg signed [PSUM_WIDTH-1:0] sum_reg;

    // 参数化的组合逻辑加法树：Vivado 能够自动将其优化为高度平衡的加法树
    always @(*) begin
        sum_comb = 0;
        for (j = 0; j < T_WIDTH; j = j + 1) begin
            sum_comb = sum_comb + prod[j];
        end
    end

    // 最终结果寄存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_reg <= 0;
        end else begin
            sum_reg <= sum_comb;
        end
    end

    // 赋值输出
    assign dot_result = sum_reg;

endmodule