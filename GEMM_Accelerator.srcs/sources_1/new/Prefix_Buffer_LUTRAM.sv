`timescale 1ns / 1ps

module Prefix_Buffer_LUTRAM #(
    parameter ADDR_WIDTH = 8,  
    parameter DATA_WIDTH = 24  
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  soft_clear,
    
    // --- 写端口 (来自 Stage 3 Update) ---
    input  wire                  we,     // Write Enable
    input  wire [ADDR_WIDTH-1:0] waddr,  // Write Node ID
    input  wire [DATA_WIDTH-1:0] wdata,  // Calculated Result
    
    // --- 读端口 1 (Prefix 查询) ---
    input  wire [ADDR_WIDTH-1:0] raddr,  
    output wire [DATA_WIDTH-1:0] rdata,  
    output wire                  rvalid,

    // --- 读端口 2 (Self Check 查询) ---
    input  wire [ADDR_WIDTH-1:0] raddr_check,  // 连接 Current Node ID
    output wire [DATA_WIDTH-1:0] rdata_check,  // 如果命中，直接输出结果
    output wire                  rvalid_check  // 命中信号
    // LUTRAM 读是异步的，地址变数据即变
);

    // 1. 定义存储阵列
    // (* ram_style = "distributed" *) 提示综合器使用 LUTRAM
    (* ram_style = "distributed" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    reg [(1<<ADDR_WIDTH)-1:0] valid_bits; // 记录哪些地址被写过(用寄存器阵列实现，支持单拍全部清零)
    
    // 1. 清零逻辑 & 写入逻辑 (同步)
    always @(posedge clk) begin
        if (!rst_n || soft_clear) begin 
            valid_bits <= 0; // 直接赋 0 ，自动拓展到所有位
            // 因为有 valid_bits 拦截，mem 里的历史垃圾数据根本不会被读出，无需清零
        end else if (we) begin
            mem[waddr] <= wdata;
            valid_bits[waddr] <= 1'b1;
        end
    end

    // 2. 读取逻辑
    // 关键设计：解决读写冲突 (Read-After-Write Hazard)
    // --- Port 1 Logic (Prefix) ---
    assign rdata =
        (raddr == 0)             ? {DATA_WIDTH{1'b0}} :
        (we && (waddr == raddr)) ? wdata              : // 旁路：本拍正在写，直接用新数据
        (valid_bits[raddr])      ? mem[raddr]         :
                                   {DATA_WIDTH{1'b0}};
    assign rvalid = 
        (raddr == 0)             ? 1'b1 : 
        (we && (waddr == raddr)) ? 1'b1 :               // 本拍正在写的地址，valid 必须被旁路为 1
                                   valid_bits[raddr];                               

    // --- Port 2 Logic (Self Check) ---
    assign rdata_check =
        (raddr_check == 0)                   ? {DATA_WIDTH{1'b0}} :
        (we && (waddr == raddr_check))       ? wdata              : // 旁路
        (valid_bits[raddr_check])            ? mem[raddr_check]   :
                                               {DATA_WIDTH{1'b0}};                                 
    assign rvalid_check = 
        (raddr_check == 0)                   ? 1'b1 : 
        (we && (waddr == raddr_check))       ? 1'b1 :               // 旁路 valid
                                               valid_bits[raddr_check]; 
                                           
endmodule