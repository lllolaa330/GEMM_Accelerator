`timescale 1ns / 1ps

module tb_SlidingWindow_CU;

    // =========================================================================
    // 1. 参数与配置
    // =========================================================================
    parameter T_WIDTH = 8;  
    parameter TILE_SIZE = 64;
    parameter M = TILE_SIZE, N = TILE_SIZE;                         
    parameter B_DATA_BITS = 8, PSUM_WIDTH = 24, OUT_WIDTH = 32;
    parameter WINDOW_SIZE = 16;

    // 自动推导参数
    localparam A_PACKED_WIDTH    = 10 + T_WIDTH; 
    localparam BATCH_NUMS        = N / T_WIDTH;
    localparam BATCH_SIZE        = M * 4;
    localparam OFFSET_ADDR_WIDTH = $clog2(BATCH_NUMS);
    localparam OUTPUT_ADDR_WIDTH = $clog2(M);
    localparam NUM_FILES         = 100; 

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n;
    reg cfg_we_b;
    reg [7:0] cfg_addr_b, cfg_data_b;
    reg a_valid;
    reg [A_PACKED_WIDTH-1:0] a_payload;
    reg [OFFSET_ADDR_WIDTH-1:0] k_offset;
    reg out_rd_en;
    reg [OUTPUT_ADDR_WIDTH-1:0] out_rd_addr;

    wire matrix_done;
    wire signed [OUT_WIDTH-1:0] out_data; 

    // 存储阵列
    reg [7:0]                  mem_b [0: N-1]; 
    reg [A_PACKED_WIDTH-1:0]   mem_a [0: (BATCH_SIZE * BATCH_NUMS - 1)]; 
    reg signed [OUT_WIDTH-1:0] golden_ref [0: M-1]; 

    // 统计变量
    integer file_idx, error_count, global_pass_count = 0;
    real    real_ppe_ops, real_ape_ops;
    real    dense_ppe_ops = (T_WIDTH) * (M*4) * (N/T_WIDTH);
    real    dense_ape_ops = 1 * (M*4) * (N/T_WIDTH);
    real    overall_density_only_ppe, overall_density_with_ape;
    real    total_overall_density_only_ppe = 0.0;
    real    total_overall_density_with_ape = 0.0;
    time    start_time, end_time, global_time_accum = 0;
    
    string  base_dir = "/media/ese/HDD-Data/vivado_projects/0409data_llama3";

    // =========================================================================
    // 3. 时钟与监控逻辑 (Monitoring)
    // =========================================================================
    // 300MHz 时钟
    parameter real CLK_FREQ_MHZ = 300.0;
    parameter real CLK_PERIOD_NS = 1000.0 / CLK_FREQ_MHZ; 
    initial clk = 0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    
    // 辅助计算函数
    function integer get_popcount(input [T_WIDTH-1:0] val);
        integer k;
        begin
            get_popcount = 0;
            for(k=0; k<T_WIDTH; k=k+1) get_popcount = get_popcount + val[k];
        end
    endfunction

    // 实时硬件操作统计
    always @(posedge clk) begin
        if (!rst_n) begin
            real_ppe_ops <= 0;
            real_ape_ops <= 0;
        end else begin
            if (dut.r_s2_valid)  real_ppe_ops <= real_ppe_ops + get_popcount(dut.s2_mask);
            if (dut.r_s3_valid)  real_ape_ops <= real_ape_ops + 1;
        end
    end
    

    // =========================================================================
    // 4. 任务定义 (Tasks)
    // =========================================================================

    // A. 系统初始化
    task init_signals();
        begin
            rst_n = 0; cfg_we_b = 0; a_valid = 0;
            k_offset = 0; out_rd_en = 0; out_rd_addr = 0;
            #(CLK_PERIOD_NS * 30); 
            rst_n = 1; 
            #(CLK_PERIOD_NS * 5);
        end
    endtask

    // B. 配置常量 B RAM
    // 所有驱动信号在时钟下降沿跳变，完美规避后仿 Setup/Hold 违例
    task configure_b_ram();
        integer i;
        string b_path;
        begin
            $display("[Init] Loading mem_b from b.txt...");
            b_path = $sformatf("%s/b.txt", base_dir);
            $readmemh(b_path, mem_b);
            for (i = 0; i < N; i = i + 1) begin
                @(negedge clk);
                cfg_we_b   = 1; 
                cfg_addr_b = i[7:0]; 
                cfg_data_b = mem_b[i];
            end
            @(negedge clk); 
            cfg_we_b = 0;
            $display("[Init] B-RAM Configuration Done.");
        end
    endtask

    // C. 软复位 (单次文件测试前)
    task soft_reset_dut();
        begin
            @(negedge clk);
            rst_n = 0; a_valid = 0;
            #(CLK_PERIOD_NS * 15); 
            rst_n = 1; 
            #(CLK_PERIOD_NS * 5);
        end
    endtask

    // D. 加载测试文件
    task load_test_data(input integer idx);
        string a_path, g_path;
        begin
            a_path = $sformatf("%s/a/up_proj/a_%04d.txt", base_dir, idx);
            g_path = $sformatf("%s/golden/up_proj/golden_%04d.txt", base_dir, idx);
            $readmemh(a_path, mem_a);
            $readmemh(g_path, golden_ref);
        end
    endtask

    // E. 注入 A 矩阵数据
    // 数据注入强制在下降沿进行，且取消了容易引起问题的 #1 延迟
    task drive_a_payload();
        integer i, j;
        begin
            start_time = $time;
            for (i = 0; i < BATCH_NUMS; i = i + 1) begin
                for (j = 0; j < BATCH_SIZE; j = j + 1) begin
                    @(negedge clk);
                    a_payload = mem_a[i * BATCH_SIZE + j]; 
                    k_offset  = i[OFFSET_ADDR_WIDTH-1:0]; 
                    a_valid   = 1;
                end
            end
            
            // 传完最后一笔后拉低 valid
            @(negedge clk);
            a_valid = 0;

            // 等待计算完成信号
            @(posedge matrix_done);
            end_time = $time;
        end
    endtask

    // F. 验证结果并计算误差
    task verify_and_report(input integer idx);
        integer i;
        begin
            error_count = 0;
            for (i = 0; i < M; i = i + 1) begin
                // 1. 在下降沿给读地址和使能
                @(negedge clk);
                out_rd_en = 1; 
                out_rd_addr = i[OUTPUT_ADDR_WIDTH-1:0];
                
                // 2. 让上升沿去采样这个地址
                @(posedge clk);
                
                // 3. 再等一个下降沿去读取结果，避开后仿中由布线导致的最大 Output Delay
                @(negedge clk);
                if ($signed(out_data) !== golden_ref[i]) begin
                    error_count = error_count + 1;
                end
            end
            
            out_rd_en = 0;
            
            
            // 统计计算
            if (error_count == 0) global_pass_count = global_pass_count + 1;
            global_time_accum = global_time_accum + (end_time - start_time);
            
            overall_density_only_ppe = (real_ppe_ops / dense_ppe_ops) * 100.0;
            overall_density_with_ape = ((real_ppe_ops+real_ape_ops) / (dense_ppe_ops+dense_ape_ops)) * 100.0;

            total_overall_density_only_ppe = total_overall_density_only_ppe + overall_density_only_ppe;
            total_overall_density_with_ape = total_overall_density_with_ape + overall_density_with_ape;
            
             $display("[File %04d] %s | Time: %0t | Errors: %0d ", 
                      idx, (error_count==0 ? "PASS" : "FAIL"), (end_time-start_time), error_count);
            
        end
    endtask

    // =========================================================================
    // 5. 主控制流程 (Main Control)
    // =========================================================================
    initial begin
        // 1. 初始化
        init_signals();
        configure_b_ram();

        // 2. 批处理循环
        $display("\n--- Starting Batch Processing ---");
        $display("Time: %0t ", $time);
        for (file_idx = 1; file_idx <= NUM_FILES; file_idx = file_idx + 1) begin
            load_test_data(file_idx);
            soft_reset_dut();
            drive_a_payload();
            verify_and_report(file_idx);
        end

        // 3. 打印全局总结报告
         print_final_summary();
        $finish;
    end

    
    // G. 打印总结
    task print_final_summary();
        begin
            $display("\n==================================================");
            $display("         GLOBAL EXPERIMENT REPORT (T = %0d, Window Size = %0d, Tile size = %0d)       ", T_WIDTH, WINDOW_SIZE, TILE_SIZE);
            $display("==================================================");
            $display("  => Total Files Tested       : %0d", NUM_FILES);
            $display("  => Total Passed Files       : %0d", global_pass_count);
            $display("  => Global Accuracy Rate     : %0.2f %%", ($itor(global_pass_count)/NUM_FILES)*100.0);
            $display("  => Avg Compute Time/File    : %0.2f ns", $itor(global_time_accum)/NUM_FILES);
            $display("  => Avg Overall Density(Only PPE)      : %0.2f %%", total_overall_density_only_ppe/NUM_FILES);
            $display("  => Avg Overall Density(With APE)      : %0.2f %%", total_overall_density_with_ape/NUM_FILES);
            $display("==================================================\n");
        end
    endtask
    

    // =========================================================================
    // 6. DUT 实例化
    // =========================================================================
    SlidingWindow_CU #(
        .M(M), .N(N), .T_WIDTH(T_WIDTH),
        .A_PACKED_WIDTH(A_PACKED_WIDTH), .WINDOW_SIZE(WINDOW_SIZE),
        .PSUM_WIDTH(PSUM_WIDTH), .OUT_WIDTH(OUT_WIDTH)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_we_b(cfg_we_b), .cfg_addr_b(cfg_addr_b), .cfg_data_b(cfg_data_b),
        .a_valid(a_valid), .a_payload(a_payload), .k_offset(k_offset),
        .matrix_done(matrix_done),
        .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
    );

endmodule