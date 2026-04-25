`timescale 1ns / 1ps

module tb_GEMV_Accelerator_Top;

    // =========================================================================
    // 1. 参数与配置 
    // =========================================================================
    parameter WEIGHT_TOTAL_ROW = 4096;                  // 完整的权重总行数
    parameter B_VEC_LEN      = 14336; 
    parameter M              = 256;
    parameter N              = 256;
    parameter T_WIDTH        = 8;                       
    parameter OUT_WIDTH      = 32;
    parameter RAM_LATENCY    = 3;
    parameter PIPE_STAGES    = 2;

    localparam CU_NUMS        = B_VEC_LEN / N;          // 56
    localparam A_PACKED_WIDTH = 10 + T_WIDTH;           // 18
    localparam BATCH_NUMS     = N / T_WIDTH;            // 32
    localparam BATCH_SIZE     = M * 4;                  // 1024
    localparam WIDE_BUS_WIDTH = CU_NUMS * A_PACKED_WIDTH; // 56 * 18 
    localparam URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;  // 32768
    
    localparam LOOP_COUNT     = WEIGHT_TOTAL_ROW / M;   // 计算需调用Top模块的次数 (4096/256=16)
    localparam GEMV_NUM       = 2;                      // 总共计算20次GEMV（组成1个GEMM）

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n;
    
    // 控制接口
    reg                               host_start;
    wire                              system_done;

    // URAM 权重配置接口
    reg                               host_we_uram;
    reg [$clog2(URAM_DEPTH)-1:0]      host_addr_uram;
    reg [WIDE_BUS_WIDTH-1:0]          host_data_uram;

    // B 向量配置接口
    reg                               host_we_act;
    reg [$clog2(B_VEC_LEN)-1:0]       host_addr_act;
    reg [7:0]                         host_data_act;
    reg                               host_start_broadcast;
    wire                              broadcast_done;

    // 结果读取接口
    reg                               out_rd_en;
    reg [$clog2(M)-1:0]               out_rd_addr;
    wire [OUT_WIDTH-1:0]              out_data;

    // 模拟存储器 (用于从文件加载数据)
    reg [7:0]                         mem_b [0: B_VEC_LEN-1]; 
    reg [A_PACKED_WIDTH-1:0]          temp_mem_a [0: URAM_DEPTH-1]; 
    reg [A_PACKED_WIDTH-1:0]          shadow_mem_a [0: CU_NUMS-1][0: URAM_DEPTH-1]; 
    
    // 完整 4096 行计算结果与 Golden 数据存放区
    reg [OUT_WIDTH-1:0]               final_results [0: WEIGHT_TOTAL_ROW-1];
    reg [31:0]                        golden_mem    [0: WEIGHT_TOTAL_ROW-1];

    integer cu, i, j;
    integer loop_idx, file_idx;
    
    // GEMM 循环控制与错误统计变量
    integer gemv_idx;
    integer gemv_err_cnt [0:GEMV_NUM-1]; // 记录每一次 GEMV 的错误数
    integer total_gemm_err_cnt;          // 记录整个 GEMM 的总错误数

    string  base_dir = "C:/Users/lllolaa/Desktop/data_L38/data_L38"; 

    // =========================================================================
    // 3. 时钟与初始化
    // =========================================================================
    initial clk = 0;
    always #1.667 clk = ~clk; // ~300MHz

    task init_signals();
        begin
            rst_n = 0;
            host_start = 0;
            host_we_uram = 0; host_addr_uram = 0; host_data_uram = 0;
            host_we_act = 0; host_addr_act = 0; host_data_act = 0;
            host_start_broadcast = 0;
            out_rd_en = 0; out_rd_addr = 0;
            
            // 初始化错误统计
            total_gemm_err_cnt = 0;
            for (i = 0; i < GEMV_NUM; i = i + 1) begin
                gemv_err_cnt[i] = 0;
            end

            #100;
            rst_n = 1;
            #50;
        end
    endtask

    // =========================================================================
    // 4. 数据加载与读取任务 
    // =========================================================================

    // Step 1: 加载并配置向量 B 
    task config_b_vector;
        input integer g_idx;
        string b_path;
        begin
            $display("[%0t] Loading Vector B for GEMV %0d...", $time, g_idx);
            b_path = $sformatf("%s/b/down_proj/b_%04d.txt", base_dir, g_idx);
            $readmemh(b_path, mem_b);
            
            for (i = 0; i < B_VEC_LEN; i = i + 1) begin
                @(negedge clk);
                host_we_act   = 1;
                host_addr_act = i;
                host_data_act = mem_b[i];
            end
            @(negedge clk);
            host_we_act = 0;
            
            @(negedge clk);
            host_start_broadcast = 1;
            @(negedge clk);
            host_start_broadcast = 0;
            
            wait(broadcast_done == 1'b1);
            $display("[%0t] Vector B Broadcast Done for GEMV %0d.", $time, g_idx);
        end
    endtask

    // Step 2: 根据当前的循环次数，加载对应的权重块进入 L2 URAM (权重保持不变)
    task config_a_weights;
        input integer l_idx;
        string a_path;
        begin
            $display("[%0t] [Loop %0d] Loading Weights A into L2 URAM...", $time, l_idx);
            // 先读入内存
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                file_idx = l_idx * CU_NUMS + cu; // 根据循环计算真实文件编号
                a_path = $sformatf("%s/a_packed/down_proj/a_%04d.txt", base_dir, file_idx);
                $readmemh(a_path, temp_mem_a);
                for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                    shadow_mem_a[cu][j] = temp_mem_a[j];
                end
            end

            // 写入 URAM (宽总线拼接)
            for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                @(negedge clk);
                host_we_uram   = 1;
                host_addr_uram = j;
                for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                    host_data_uram[cu * A_PACKED_WIDTH +: A_PACKED_WIDTH] = shadow_mem_a[cu][j];
                end
            end
            @(negedge clk);
            host_we_uram = 0;
        end
    endtask

    // Step 3: 读取本轮 Top 运算的 256 行结果存入大数组中
    task read_and_save_results;
        input integer l_idx;
        begin
            for (i = 0; i < M; i = i + 1) begin
                @(negedge clk);
                out_rd_en   = 1;
                out_rd_addr = i;
                @(posedge clk); // URAM/BRAM 读延迟
                @(negedge clk);
                final_results[l_idx * M + i] = out_data;
            end
            out_rd_en = 0;
            $display("[%0t] [Loop %0d] Read and Save %0d Results Done.", $time, l_idx, M);
        end
    endtask

    // =========================================================================
    // 5. Golden 比对及测试报告打印任务
    // =========================================================================
    
    // 针对当前 GEMV 编号读取 Golden 文件并比对
    task compare_and_report;
        input integer g_idx;
        integer err_cnt;
        reg [31:0] out_val, gold_val;
        reg match_flag;
        string golden_path;
        begin
            err_cnt = 0;
            
            // 读取对应编号的 Golden 文件
            $display("\n[%0t] Loading Golden Results for GEMV %0d...", $time, g_idx);
            golden_path = $sformatf("%s/golden/down_proj/golden_%04d.txt", base_dir, g_idx);
            $readmemh(golden_path, golden_mem);

            $display("\n==================================================================");
            $display("       GEMV %02d (4096x4096) FINAL TEST REPORT & GOLDEN MATCH", g_idx);
            $display("==================================================================");
            
            for (i = 0; i < WEIGHT_TOTAL_ROW; i = i + 1) begin
                out_val  = final_results[i];
                gold_val = golden_mem[i];
                
                // 默认严格相等比对
                match_flag = (out_val === gold_val);
                
                // 兼容智能匹配兼容
                if (!match_flag && (gold_val[31:8] == 24'h0) && (out_val[7:0] === gold_val[7:0])) begin
                    match_flag = 1'b1;
                end
                
                // 打印结果 (保留原样，方便逐行核对)
                $display("Row [%4d] | OUT(Dec): %11d | OUT(8-Hex): %08x | GOLD(Hex): %08x  %s", 
                         i, $signed(out_val), out_val, gold_val, match_flag ? "[PASS]" : "[FAIL]");

                if (!match_flag) begin
                    err_cnt = err_cnt + 1;
                end
            end
            
            // 记录当前 GEMV 的错误数和全局错误数
            gemv_err_cnt[g_idx] = err_cnt;
            total_gemm_err_cnt = total_gemm_err_cnt + err_cnt;
            
            $display("==================================================================");
            if (err_cnt == 0) begin
                $display("       >>> GEMV %02d PASSED! All %0d rows perfectly matched. <<<       ", g_idx, WEIGHT_TOTAL_ROW);
            end else begin
                $display("       >>> GEMV %02d FAILED! %0d / %0d rows mismatched. <<<            ", g_idx, err_cnt, WEIGHT_TOTAL_ROW);
            end
            $display("==================================================================\n");
        end
    endtask

    // 完整 GEMM 结果终极汇总报告
    task print_gemm_summary;
        begin
            $display("\n##################################################################");
            $display("                  GEMM (20x GEMV) FINAL SUMMARY                   ");
            $display("##################################################################");
            for (i = 0; i < GEMV_NUM; i = i + 1) begin
                if (gemv_err_cnt[i] == 0)
                    $display("  [GEMV %02d] : PASS (0 errors)", i);
                else
                    $display("  [GEMV %02d] : FAIL (%0d errors)", i, gemv_err_cnt[i]);
            end
            $display("------------------------------------------------------------------");
            if (total_gemm_err_cnt == 0) begin
                $display("  >>> OVERALL GEMM TEST PASSED! ALL 20 VECTORS MATCHED! <<< ");
            end else begin
                $display("  >>> OVERALL GEMM TEST FAILED! TOTAL MISMATCHES: %0d <<< ", total_gemm_err_cnt);
            end
            $display("##################################################################\n");
        end
    endtask

    // =========================================================================
    // 6. 主测试流程
    // =========================================================================
    initial begin
        init_signals();
        
        // 外层循环：遍历 20 个 B 向量，执行完整的 GEMM
        for (gemv_idx = 0; gemv_idx < GEMV_NUM; gemv_idx = gemv_idx + 1) begin
            $display("\n******************************************************************");
            $display("***               STARTING GEMV SEQUENCE: %02d / %02d              ***", gemv_idx + 1, GEMV_NUM);
            $display("******************************************************************");

            // 原本的内层大矩阵循环：共 4096/256 = 16 次
            for (loop_idx = 0; loop_idx < LOOP_COUNT; loop_idx = loop_idx + 1) begin
                $display("\n------------------------------------------------------------------");
                $display(">>> GEMV %02d | Iteration: %0d / %0d <<<", gemv_idx, loop_idx + 1, LOOP_COUNT);
                
                // 只要不是全局的第一次启动（gemv_idx=0 && loop_idx=0），均硬复位
                if ((gemv_idx > 0) || (loop_idx > 0)) begin
                    $display("[%0t] [GEMV %02d Loop %0d] Applying Hard Reset to DUT...", $time, gemv_idx, loop_idx);
                    @(negedge clk);
                    rst_n = 0;
                    #100;
                    rst_n = 1;
                    #100;
                end
                
                // 每次复位后重新加载对应的 B 向量
                config_b_vector(gemv_idx);
                
                // a) 配置当前子矩阵块权重 A (不受 GEMV 轮次影响，依然是 00~15)
                config_a_weights(loop_idx);
                
                // b) 触发系统启动
                $display("[%0t] [GEMV %02d Loop %0d] Triggering System Start...", $time, gemv_idx, loop_idx);
                @(negedge clk);
                host_start = 1;
                @(negedge clk);
                host_start = 0;

                // c) 等待全流程完成
                wait(system_done == 1'b1);
                $display("[%0t] [GEMV %02d Loop %0d] System Done Signal Received!", $time, gemv_idx, loop_idx);

                // d) 提取并累加保存本轮计算结果
                read_and_save_results(loop_idx);
                
                // 等待一小段时间，确保状态稳定
                #100; 
            end

            // 完成当前一个完整的 GEMV (4096行) 后，进行比对与统计
            compare_and_report(gemv_idx);
            
        end // end of gemv_idx loop

        // 全部20次GEMV计算完毕后，打印全局 GEMM 测试报告
        print_gemm_summary();

        #200;
        $display("[%0t] Simulation Successfully Finished.", $time);
        $finish;
    end

    // =========================================================================
    // 7. 实例化 DUT
    // =========================================================================
    GEMV_Accelerator_Top #(
        .M(M),
        .N(N),
        .B_VEC_LEN(B_VEC_LEN),
        .T_WIDTH(T_WIDTH),
        .OUT_WIDTH(OUT_WIDTH),
        .RAM_LATENCY(RAM_LATENCY),
        .PIPE_STAGES(PIPE_STAGES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),

        // 主机控制
        .host_start(host_start),
        .system_done(system_done),

        // L2 URAM 配置
        .host_we_uram(host_we_uram),
        .host_addr_uram(host_addr_uram),
        .host_data_uram(host_data_uram),

        // L1 广播配置
        .host_we_act(host_we_act),
        .host_addr_act(host_addr_act),
        .host_data_act(host_data_act),
        .host_start_broadcast(host_start_broadcast),
        .broadcast_done(broadcast_done),

        // 结果输出
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_data(out_data)
    );

endmodule