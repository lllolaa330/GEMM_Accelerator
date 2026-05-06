`timescale 1ns / 1ps

module Accumulation_PE #(
    parameter PSUM_W = 24,
    parameter OUT_W = 32,
    parameter MAX_ROWS = 256  // 8位 row_idx
)(
    input  wire clk,
    input  wire rst_n,        // 全局复位

    // --- 计算输入 (来自 Stage 2) ---
    input  wire                      valid_in,
    input  wire signed [PSUM_W-1:0]  partial_sum,
    input  wire [7:0]                row_idx,      
    input  wire [1:0]                bitplane_id,  

    // --- 结果输出 ---
    output wire signed [OUT_W-1:0]   ape_result,

    // --- 结果读出接口 ---
    input  wire                                   res_rd_en,    
    input  wire [$clog2(MAX_ROWS)-1:0]            res_rd_addr,  
    output reg signed [OUT_W-1:0]                 res_rd_data    // 同步读，提高时序性能
);

    localparam ADDR_W = $clog2(MAX_ROWS);

    // 1. 定义存储器 (LUTRAM)
    (* ram_style = "distributed" *) reg signed [OUT_W-1:0] acc_mem [0:MAX_ROWS-1];
    
    // 有效位图：记录哪些行在本轮计算中被“污染”过
    reg [MAX_ROWS-1:0] vld_map;

    // --- 旁路寄存器 (Forwarding Registers) ---
    reg [7:0]            fwd_row_idx; // 记录上一拍写入的行地址
    reg [OUT_W-1:0]      fwd_data;    // 记录上一拍计算出的新结果
    reg                  fwd_valid;   // 记录上一拍是否发生了有效写入

    // 2. 移位逻辑
    reg signed [OUT_W-1:0] shifted_val;
    always @(*) begin
            case (bitplane_id)
                2'd0: shifted_val = $signed(partial_sum);
                2'd1: shifted_val = $signed(partial_sum) <<< 1; 
                2'd2: shifted_val = $signed(partial_sum) <<< 2;
                2'd3: shifted_val = $signed(partial_sum) <<< 3;
                default: shifted_val = $signed(partial_sum);
            endcase
    end
    
    // 3. 快速清零逻辑
    reg is_clearing;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vld_map <= {MAX_ROWS{1'b0}};
            is_clearing <= 1'b0;
        end else begin
            is_clearing <= 1'b0;
            if (valid_in) begin
                vld_map[row_idx] <= 1'b1; // 写入时标记为有效
            end
        end
    end

    // 4. 旁路逻辑
    wire signed [OUT_W-1:0] ram_raw_val = acc_mem[row_idx];
    wire signed [OUT_W-1:0] ram_val_checked = vld_map[row_idx] ? ram_raw_val : {OUT_W{1'b0}};
    // 优先级：旁路寄存器 > 经过验证的内存值
    // 如果当前要算的 row_idx 等于上一拍刚算完还没存进 RAM 的 fwd_row_idx，则直接用 fwd_data
    wire signed [OUT_W-1:0] base_val = (fwd_valid && (row_idx == fwd_row_idx)) ? fwd_data : ram_val_checked;
    wire signed [OUT_W-1:0] next_sum = base_val + shifted_val;
    assign ape_result = base_val;

    // 5. 核心写逻辑
    always @(posedge clk) begin
        if (valid_in ) begin
            acc_mem[row_idx] <= next_sum;
            fwd_row_idx <= row_idx;
            fwd_data    <= next_sum;
            fwd_valid   <= 1'b1;
        end else begin
            fwd_valid   <= 1'b0;
        end
    end

    // 5. 结果同步读出
    always @(posedge clk) begin
        if (res_rd_en) begin
            res_rd_data <= vld_map[res_rd_addr] ? acc_mem[res_rd_addr] : {OUT_W{1'b0}};
        end
    end
endmodule