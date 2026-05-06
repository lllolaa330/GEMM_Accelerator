`timescale 1ns / 1ps

module Prefix_PE #(
    parameter T = 8,
    parameter D_W = 8,
    parameter PSUM_W = 24 
)(
    input  wire [T*D_W-1:0]          vec_b_slice, 
    input  wire [T-1:0]              mask,
    input  wire signed[PSUM_W-1:0]   prefix_in,  
    output wire signed[PSUM_W-1:0]   result_out      
);

    // 定义组合逻辑中使用的内部寄存器变量
    reg signed [PSUM_W-1:0] sum;
    reg signed[D_W-1:0]    temp_b;
    integer i;

    // 纯组合逻辑块：行为级描述累加过程
    always @(*) begin
        // 1. 初始化累加器为传入的 prefix_in
        sum = prefix_in;
        
        // 2. 遍历所有的 T 个元素
        for (i = 0; i < T; i = i + 1) begin
            
            // 提取出对应的 D_W 位数据，并强制转换为有符号数
            temp_b = $signed(vec_b_slice[i*D_W +: D_W]);
            
            // 3. MASK 判断逻辑
            if (mask[i] == 1'b1) begin
                // 如果掩码命中，累加到 sum 中。
                // 注意：temp_b 是 D_W 位(如 8-bit)，sum 是 PSUM_W 位(如 24-bit)
                // 因为两者都声明了 signed，Verilog 会自动进行完美的符号位扩展！
                sum = sum + temp_b;
            end
        end
    end

    // 4. 输出最终结果
    assign result_out = sum;

endmodule