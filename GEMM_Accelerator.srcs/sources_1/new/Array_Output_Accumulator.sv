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
    input  wire                               start,       // 启动累加的脉冲
    output wire                               done,        // 累加完成标志脉冲

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
    // 内部寄存器与信号
    // =========================================================================
    reg [ADDR_W:0] rd_addr_reg;  
    reg            rd_en_reg;
    reg [ADDR_W:0] wr_addr_reg;  
    reg            wr_en_reg;
    reg            done_reg;

    integer i;
    reg signed [OUT_WIDTH-1:0] sum_comb;
    (* ram_style = "block" *) reg signed [OUT_WIDTH-1:0] res_ram [0:L-1];
    reg signed [OUT_WIDTH-1:0] ext_rd_data_reg;

    // =========================================================================
    // 1. 发起向所有 CU 的同步读请求 (遍历 0 到 L-1)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_addr_reg <= 0;
            rd_en_reg   <= 0;
        end else if (start) begin
            rd_addr_reg <= 0;
            rd_en_reg   <= 1'b1;
        end else if (rd_en_reg) begin
            if (rd_addr_reg == L - 1) begin
                rd_en_reg   <= 1'b0; 
            end else begin
                rd_addr_reg <= rd_addr_reg + 1;
            end
        end
    end

    assign cu_rd_en   = rd_en_reg;
    assign cu_rd_addr = rd_addr_reg[ADDR_W-1:0];

    // =========================================================================
    // 2. 模拟 1 拍 SRAM 读延迟对齐 (CU RAM的延迟)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_en_reg   <= 0;
            wr_addr_reg <= 0;
        end else begin
            wr_en_reg   <= rd_en_reg;
            wr_addr_reg <= rd_addr_reg;
        end
    end

    // =========================================================================
    // 3. 组合逻辑加法树 (43个输入相加)
    // =========================================================================
    always @(*) begin
        sum_comb = 0;
        for (i = 0; i < COL_NUM; i = i + 1) begin
            sum_comb = sum_comb + $signed(cu_out_data_flat[i * OUT_WIDTH +: OUT_WIDTH]);
        end
    end

    // =========================================================================
    // 4. 写入最终结果 RAM 并产生完成信号
    // =========================================================================
    always @(posedge clk) begin
        if (wr_en_reg) begin
            res_ram[wr_addr_reg[ADDR_W-1:0]] <= sum_comb;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) done_reg <= 1'b0;
        else if (wr_en_reg && (wr_addr_reg == L - 1)) done_reg <= 1'b1; // 单拍脉冲
        else done_reg <= 1'b0;
    end
    assign done = done_reg;

    // =========================================================================
    // 5. 顶层外部读取逻辑 (1拍延迟输出)
    // =========================================================================
    always @(posedge clk) begin
        if (ext_rd_en) begin
            ext_rd_data_reg <= res_ram[ext_rd_addr];
        end
    end
    assign ext_rd_data = ext_rd_data_reg;

endmodule