`timescale 1ns / 1ps

module Prefix_Buffer_WINDOW #(
    parameter ADDR_WIDTH = 8,  
    parameter DATA_WIDTH = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  batch_rst,
    
    // --- 写端口 (来自 Stage 3 Update) ---
    input  wire                  we,     
    input  wire [ADDR_WIDTH-1:0] waddr,  
    input  wire [DATA_WIDTH-1:0] wdata,  
    
    // --- 读端口 (Prefix 查询) ---
    input  wire [ADDR_WIDTH-1:0] raddr,  
    output wire [DATA_WIDTH-1:0] rdata,  
    output wire                  rvalid
);

    // =========================================================================
    // 参数与内部信号声明置顶
    // =========================================================================
    // (* ram_style = "distributed" *) 提示综合器强制使用 LUTRAM
    (* ram_style = "distributed" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];    
    
    reg [(1<<ADDR_WIDTH)-1:0]    valid_bits;   // 记录哪些地址被写过(支持单拍清零)

    // =========================================================================
    // 逻辑实现
    // =========================================================================

    // --- 清零 & 写入逻辑 (同步) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            valid_bits <= 0; 
        end else if (batch_rst) begin
            // 收到上层的批次结束信号，单拍清空所有有效位
            valid_bits <= 0;
            // 即使是最后一个数据，依然写入RAM（但由于 valid 变 0，它不会被下一批次误用）
            if (we) mem[waddr] <= wdata; 
        end else if (we) begin
            mem[waddr] <= wdata;
            valid_bits[waddr] <= 1'b1;
        end
    end

    // --- 读取逻辑 (异步旁路) ---
    assign rdata = (raddr == 0)             ? {DATA_WIDTH{1'b0}} :
                   (we && (waddr == raddr)) ? wdata              : // 旁路：本拍正在写，直接用新数据
                   (valid_bits[raddr])      ? mem[raddr]         :
                                              {DATA_WIDTH{1'b0}};

    assign rvalid = (raddr == 0)             ? 1'b1 : 
                    (we && (waddr == raddr)) ? 1'b1 :              // 本拍正在写的地址，valid旁路为1
                                               valid_bits[raddr];                               

endmodule