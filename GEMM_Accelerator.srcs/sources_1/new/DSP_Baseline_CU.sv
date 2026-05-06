`timescale 1ns / 1ps

module DSP_Baseline_CU #(
    parameter M = 256,
    parameter N = 256,                         // 矩阵形状：M*N
    parameter A_DATA_BITS = 4,
    parameter B_DATA_BITS = 8,
    parameter PSUM_WIDTH = 24,
    parameter OUT_WIDTH = 32,            
    parameter OUTPUT_ADDR_WIDTH = $clog2(M)
)(
    input  wire clk,
    input  wire rst_n,

    // --- Configuration Interface B (Vector B) ---
    input  wire                           cfg_we_b,
    input  wire [7:0]                     cfg_addr_b,
    input  wire signed [B_DATA_BITS-1:0]  cfg_data_b, 

    // --- Runtime Data Stream ---
    input  wire                                   a_valid,
    input  wire unsigned [A_DATA_BITS-1:0]        a_payload,
    output wire                                   matrix_done,

    // --- Output Interface ---
    input  wire                                 out_rd_en,
    input  wire [OUTPUT_ADDR_WIDTH-1:0]         out_rd_addr,
    output wire signed [OUT_WIDTH-1:0]          out_data
);

    // =========================================================================
    // 1. Vector B Storage (Simple Dual Port RAM)
    // =========================================================================
    reg signed [7:0] b_ram [0:N-1];
    reg signed [7:0] b_data_out;
    
    // Config Port (Write)
    always @(posedge clk) begin
        if (cfg_we_b) begin
            b_ram[cfg_addr_b] <= cfg_data_b;
        end
    end

    // =========================================================================
    // 2. Control Logic (Row & Column Counters)
    // =========================================================================
    reg [$clog2(N)-1:0] col_cnt;
    reg [$clog2(M)-1:0] row_cnt;

    // 当输入 a_valid 时，矩阵 A 按行逐元素流入
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= 0;
            row_cnt <= 0;
        end else if (a_valid) begin
            if (col_cnt == N - 1) begin
                col_cnt <= 0;
                if (row_cnt == M - 1)
                    row_cnt <= 0;
                else
                    row_cnt <= row_cnt + 1;
            end else begin
                col_cnt <= col_cnt + 1;
            end
        end
    end

    // Runtime Port (Read Vector B) - 取数延迟 1 周期
    always @(posedge clk) begin
        if (a_valid) begin
            b_data_out <= b_ram[col_cnt];
        end
    end

    // =========================================================================
    // 3. DSP MAC Pipeline (3-Stage for maximum Fmax mapping to DSP48)
    // =========================================================================
    
    // --- Pipeline Registers ---
    reg unsigned [A_DATA_BITS-1:0]  dsp_a;
    reg signed [B_DATA_BITS-1:0]    dsp_b;
    (* use_dsp = "yes" *) reg signed [PSUM_WIDTH-1:0]     dsp_m;   // 乘法器寄存器
    (* use_dsp = "yes" *) reg signed [OUT_WIDTH-1:0]      dsp_acc; // 累加器
    
    reg valid_r1, is_first_r1, is_last_r1;
    reg valid_r2, is_first_r2, is_last_r2;
    reg valid_r3, is_last_r3;

    reg [$clog2(M)-1:0] row_cnt_r1, row_cnt_r2, row_cnt_r3;

    // --- Stage 1: Load A Data and match B RAM Latency ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_r1 <= 0; is_first_r1 <= 0; is_last_r1 <= 0; row_cnt_r1 <= 0; dsp_a <= 0;
        end else begin
            valid_r1    <= a_valid;
            if (a_valid) begin
                dsp_a       <= $unsigned(a_payload);
                is_first_r1 <= (col_cnt == 0);
                is_last_r1  <= (col_cnt == N - 1);
                row_cnt_r1  <= row_cnt;
            end else begin
                is_first_r1 <= 0;
                is_last_r1  <= 0;
            end
        end
    end

    // --- Stage 2: Multiply ---
    wire signed [A_DATA_BITS:0] signed_dsp_a = {1'b0, dsp_a}; 
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_r2 <= 0; is_first_r2 <= 0; is_last_r2 <= 0; row_cnt_r2 <= 0;
            dsp_b <= 0; dsp_m <= 0;
        end else begin
            valid_r2    <= valid_r1;
            is_first_r2 <= is_first_r1;
            is_last_r2  <= is_last_r1;
            row_cnt_r2  <= row_cnt_r1;
            
            if (valid_r1) begin
                dsp_b <= b_data_out;

                dsp_m <= signed_dsp_a * b_data_out; 
                // $display("Cycle %0t: Row %0d => A=%0d B=%0d, Mult_Result=%0d", $time, row_cnt_r1, dsp_a, b_data_out, signed_dsp_a * b_data_out);
            end
        end
    end

    // --- Stage 3: Accumulate ---
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_r3 <= 0; is_last_r3 <= 0; row_cnt_r3 <= 0; dsp_acc <= 0;
        end else begin
            valid_r3   <= valid_r2;
            is_last_r3 <= is_last_r2;
            row_cnt_r3 <= row_cnt_r2;
            
            if (valid_r2) begin
                if (is_first_r2) 
                    dsp_acc <= dsp_m;                          // 新的一行，重置累加器
                else 
                    dsp_acc <= dsp_acc + dsp_m;       // 累加当前行的点积
            end
        end
    end

    // =========================================================================
    // 4. Output Storage (Result RAM)
    // =========================================================================
    // 当流水线最后阶段的 `is_last_r3` 拉高时，意味着这一行的 N 个数据均已累加完毕
    wire out_ram_we = valid_r3 & is_last_r3;
    wire [$clog2(M)-1:0]        out_ram_waddr = row_cnt_r3;
    wire signed [OUT_WIDTH-1:0] out_ram_wdata = dsp_acc;

    // 推断为块 RAM (Block RAM / LUTRAM)
    reg signed [OUT_WIDTH-1:0] out_ram [0:M-1];
    reg signed [OUT_WIDTH-1:0] out_data_reg;

    always @(posedge clk) begin
        if (out_ram_we) begin
            out_ram[out_ram_waddr] <= out_ram_wdata;
        end
    end

    // 输出读取接口，1 周期读延迟（与 APE 行为对齐）
    always @(posedge clk) begin
        if (out_rd_en) begin
            out_data_reg <= out_ram[out_rd_addr];
        end
    end
    assign out_data = out_data_reg;

    // =========================================================================
    // 5. Matrix Done Indicator
    // =========================================================================
    reg is_all_processed;
    reg [7:0] done_shift;
    
    // 当所有行全部累加并写入 RAM 完成时，触发计算结束信号
    wire computation_finished = (out_ram_we && (row_cnt_r3 == M - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_all_processed <= 1'b0;
            done_shift       <= 8'd0;
        end else begin
            if (computation_finished) begin
                is_all_processed <= 1'b1;
            end
            
            // 使用移位寄存器延迟 8 拍输出，完全对齐原滑动窗口模块的时序习惯
            if (is_all_processed || computation_finished) begin
                done_shift <= {done_shift[6:0], 1'b1};
            end else begin
                done_shift <= 8'd0;
            end
        end
    end

    assign matrix_done = done_shift[7];

endmodule