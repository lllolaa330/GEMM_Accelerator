`timescale 1ns / 1ps

module Prefix_PE_Window #(
    parameter T = 8,
    parameter D_W = 8,
    parameter PSUM_W = 24 
)(
    input  wire [T*D_W-1:0]          vec_b_slice, 
    input  wire [T-1:0]              mask_xor,    // 差异位
    input  wire [T-1:0]              sign_flag,   // 1: 减法(1->0), 0: 加法(0->1)
    input  wire signed[PSUM_W-1:0]   prefix_in,  
    output wire signed[PSUM_W-1:0]   result_out      
);

    reg signed [PSUM_W-1:0] sum;
    reg signed [D_W-1:0]    temp_b;
    integer i;

    always @(*) begin
        sum = prefix_in;
        for (i = 0; i < T; i = i + 1) begin
            // 提取对应权重
            temp_b = $signed(vec_b_slice[i*D_W +: D_W]);
            
            // 如果该位有差异
            if (mask_xor[i] == 1'b1) begin
                if (sign_flag[i] == 1'b1) 
                    sum = sum + temp_b; // 减法
                else 
                    sum = sum - temp_b; // 加法
            end
        end
    end

    assign result_out = sum;
endmodule