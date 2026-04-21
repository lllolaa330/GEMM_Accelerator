`timescale 1ns / 1ps

module Array_Output_Accumulation #(
    parameter COL_NUM = 2,
    parameter L = 256,
    parameter OUT_WIDTH = 32,
    parameter ADDR_W = $clog2(L)
)(
    input  wire clk,
    input  wire rst_n,
    
    // --- 握手与控制 ---
    input  wire start,             // 启动累加的脉冲信号
    output wire done,              // 累加完成的标志

    // --- 与 CU 侧的读接口 ---
    output wire                           cu_rd_en,
    output wire [ADDR_W-1:0]              cu_rd_addr,
    input  wire[COL_NUM*OUT_WIDTH-1:0]    cu_out_data_flat, // 展平的 COL_NUM 个 CU 输出

    // --- 与顶层外部的读接口 (Out_Read 阶段) ---
    input  wire                           ext_rd_en,
    input  wire [ADDR_W-1:0]              ext_rd_addr,
    output wire signed [OUT_WIDTH-1:0]    ext_rd_data
);

    // 内部寄存器
    reg [ADDR_W:0] rd_addr_reg;  // 多 1 bit 防止溢出
    reg            rd_en_reg;
    
    reg [ADDR_W:0] wr_addr_reg;  
    reg            wr_en_reg;

    // 1. 发起向 CU 的读请求 (从 0 遍历到 L-1)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_addr_reg <= 0;
            rd_en_reg   <= 0;
        end else if (start) begin
            rd_addr_reg <= 0;
            rd_en_reg   <= 1'b1;
        end else if (rd_en_reg) begin
            if (rd_addr_reg == L - 1) begin
                rd_en_reg   <= 1'b0; // 读完 L 个数据后停止
            end else begin
                rd_addr_reg <= rd_addr_reg + 1;
            end
        end
    end

    assign cu_rd_en   = rd_en_reg;
    assign cu_rd_addr = rd_addr_reg[ADDR_W-1:0];

    // 2. 模拟 1 拍 SRAM 读延迟，产生写使能和写地址
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_en_reg   <= 0;
            wr_addr_reg <= 0;
        end else begin
            wr_en_reg   <= rd_en_reg;
            wr_addr_reg <= rd_addr_reg;
        end
    end

    // 3. 组合逻辑加法树 (Combinational Adder Tree)
    integer i;
    reg signed [OUT_WIDTH-1:0] sum_comb;
    always @(*) begin
        sum_comb = 0;
        for (i = 0; i < COL_NUM; i = i + 1) begin
            // 累加所有列同一行地址的数据
            sum_comb = sum_comb + $signed(cu_out_data_flat[i * OUT_WIDTH +: OUT_WIDTH]);
        end
    end

    // 4. Result RAM：保存当前行的所有 L 个最终结果
    reg signed [OUT_WIDTH-1:0] res_ram [0:L-1];
    always @(posedge clk) begin
        if (wr_en_reg) begin
            res_ram[wr_addr_reg[ADDR_W-1:0]] <= sum_comb;
        end
    end

    // 5. 完成信号
    reg done_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_reg <= 1'b0;
        end else if (start) begin
            done_reg <= 1'b0;
        end else if (wr_en_reg && (wr_addr_reg == L - 1)) begin
            done_reg <= 1'b1;
        end
    end
    assign done = done_reg;

    // 6. 处理顶层的外部读取请求 (1 拍延迟输出)
    reg signed [OUT_WIDTH-1:0] ext_rd_data_reg;
    always @(posedge clk) begin
        if (ext_rd_en) begin
            ext_rd_data_reg <= res_ram[ext_rd_addr];
        end
    end
    assign ext_rd_data = ext_rd_data_reg;

endmodule