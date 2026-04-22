`timescale 1ns / 1ps

module Array_Output_Accumulation #(
    parameter COL_NUM   = 43,
    parameter L         = 256,
    parameter OUT_WIDTH = 32,
    parameter ADDR_W    = $clog2(L)
)(
    input  wire                               clk,
    input  wire                               rst_n,
    
    // --- 握手与控制 ---
    input  wire                               start,       
    output wire                               done,        

    // --- 与 CU 阵列的读接口 ---
    output wire                               cu_rd_en,
    output wire [ADDR_W-1:0]                  cu_rd_addr,
    input  wire [COL_NUM*OUT_WIDTH-1:0]       cu_out_data_flat, 

    // --- 与顶层外部的读接口 ---
    input  wire                               ext_rd_en,
    input  wire [ADDR_W-1:0]                  ext_rd_addr,
    output wire signed [OUT_WIDTH-1:0]        ext_rd_data
);

    // =========================================================================
    // 信号定义
    // =========================================================================
    reg [ADDR_W-1:0] rd_addr_cnt;
    reg              rd_active;

    // 流水线延迟寄存器
    reg              d1_en;
    reg [ADDR_W-1:0] d1_addr;
    reg              wr_en;
    reg [ADDR_W-1:0] wr_addr;

    reg signed [OUT_WIDTH-1:0] sum_reg; // 寄存器化的求和结果
    (* ram_style = "block" *) reg signed [OUT_WIDTH-1:0] res_ram [0:L-1];

    // =========================================================================
    // 1. 发起读请求 (请求地址 T0)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_addr_cnt <= 0;
            rd_active   <= 0;
        end else if (start) begin
            rd_addr_cnt <= 0;
            rd_active   <= 1'b1;
        end else if (rd_active) begin
            if (rd_addr_cnt == L - 1)
                rd_active <= 1'b0;
            else
                rd_addr_cnt <= rd_addr_cnt + 1;
        end
    end

    assign cu_rd_en   = rd_active;
    assign cu_rd_addr = rd_addr_cnt;

    // =========================================================================
    // 2. 核心流水线对齐
    // T+1: CU 数据出现在总线上，进行求和
    // T+2: 求和结果存入 res_ram
    // =========================================================================
    integer i;
    reg signed [OUT_WIDTH-1:0] sum_v; // 临时组合逻辑变量

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d1_en   <= 0; d1_addr <= 0;
            wr_en   <= 0; wr_addr <= 0;
            sum_reg <= 0;
        end else begin
            // 第一级延迟：等待 CU 数据返回
            d1_en   <= rd_active;
            d1_addr <= rd_addr_cnt;

            // 在数据有效的那一拍进行求和并锁存
            sum_v = 0;
            for (i = 0; i < COL_NUM; i = i + 1) begin
                sum_v = sum_v + $signed(cu_out_data_flat[i * OUT_WIDTH +: OUT_WIDTH]);
            end
            sum_reg <= sum_v; 

            // 第二级延迟：等待求和寄存器结果稳定，准备写入 RAM
            wr_en   <= d1_en;
            wr_addr <= d1_addr;
        end
    end

    // =========================================================================
    // 3. 结果存储与完成信号
    // =========================================================================
    always @(posedge clk) begin
        if (wr_en) begin
            res_ram[wr_addr] <= sum_reg;
        end
    end

    // done 信号必须在最后一拍数据写入 RAM 后发出
    reg done_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) done_q <= 1'b0;
        else if (wr_en && (wr_addr == L - 1)) done_q <= 1'b1;
        else done_q <= 1'b0;
    end
    assign done = done_q;

    // =========================================================================
    // 4. 外部读取 (保持1拍延迟)
    // =========================================================================
    reg signed [OUT_WIDTH-1:0] ext_data_q;
    always @(posedge clk) begin
        if (ext_rd_en)
            ext_data_q <= res_ram[ext_rd_addr];
    end
    assign ext_rd_data = ext_data_q;

endmodule