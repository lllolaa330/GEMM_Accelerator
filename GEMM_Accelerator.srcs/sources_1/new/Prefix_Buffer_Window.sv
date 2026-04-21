`timescale 1ns / 1ps

module Prefix_Buffer_WINDOW #(
    parameter ADDR_WIDTH = 8,  
    parameter DATA_WIDTH = 24,
    parameter BATCH_SIZE = 256 * 4
)(
    input  wire                  clk,
    input  wire                  rst_n,
    
    // --- 写端口 (来自 Stage 3 Update) ---
    input  wire                  we,     
    input  wire [ADDR_WIDTH-1:0] waddr,  
    input  wire [DATA_WIDTH-1:0] wdata,  
    
    // --- 读端口 (Prefix 查询) ---
    input  wire [ADDR_WIDTH-1:0] raddr,  
    output wire [DATA_WIDTH-1:0] rdata,  
    output wire                  rvalid

    // LUTRAM 读是异步的，地址变数据即变
);

    // 1. 定义存储阵列
    (* ram_style = "distributed" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];    
    // (* ram_style = "distributed" *) 提示综合器使用 LUTRAM
    
    reg [(1<<ADDR_WIDTH)-1:0]     valid_bits;   // 记录哪些地址被写过(用寄存器阵列实现，支持单拍全部清零)
    reg [$clog2(BATCH_SIZE)-1:0]  batch_cnt; 

    // 1. 清零 & 写入 (同步)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            valid_bits <= 0; 
            batch_cnt  <= 0;
        end else if (we) begin
            mem[waddr]        <= wdata;
            if (batch_cnt == BATCH_SIZE - 1) begin
                // 【软复位触发】当前存入的是本 Batch 的最后一个数据
                valid_bits <= 0;
                batch_cnt  <= 0;
            end else begin
                valid_bits[waddr] <= 1'b1;
                batch_cnt         <= batch_cnt + 1'b1;
            end
        end
    end

    // 2. 读取
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
                                            
endmodule