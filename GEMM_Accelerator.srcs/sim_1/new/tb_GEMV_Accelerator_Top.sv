`timescale 1ns / 1ps

module tb_GEMV_Accelerator_Top;

    // =========================================================================
    // 1. 参数与配置 
    // =========================================================================
    parameter WEIGHT_TOTAL_ROW = 1024;                  // 完整的权重总行数
    parameter B_VEC_LEN        = 4096;                  // 完整的权重总列数 也是 B向量的长度
    localparam GEMV_NUM        = 1;                     // 总共计算的GEMV次数（组成完整GEMM）
    string  base_dir           = "C:/Users/lllolaa/Desktop/data_L38/data_L38"; 
    string target_layer        = "k_proj";              // 目标层名称，用于构建文件路径
    parameter CLK_MHZ          = 300;                   // 时钟频率 (MHz)
    parameter CU_TYPE          = 2;                     // 0:Static, 1:Dynamic, 2:SlidingWindow

    parameter M              = 256;                     // 底层 CU 计算的行数 (每次处理256行)
    parameter N              = 256;                     // 底层 CU 计算的列数 (每次处理256列) 
    parameter T_WIDTH        = 8;                       // 比特重用位宽
    parameter OUT_WIDTH      = 32;
    parameter RAM_LATENCY    = 3;
    parameter PIPE_STAGES    = 2;

    localparam CU_NUMS        = B_VEC_LEN / N;          // 实例化的CU数量 
    localparam A_PACKED_WIDTH = 10 + T_WIDTH;           // 每个Packed A元素的位宽  node:[17:10] row_idx:[9:2] bitplane_id:[1:0]  
    localparam BATCH_NUMS     = N / T_WIDTH;            // 批次数量
    localparam BATCH_SIZE     = M * 4;                  // 每个批次的大小
    localparam WIDE_BUS_WIDTH = CU_NUMS * A_PACKED_WIDTH; // 宽总线宽度
    localparam URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;  // URAM深度
    localparam CLK_PERIOD = 1000 / CLK_MHZ;             // 时钟周期 (ns)
    localparam CLK_HALF_PERIOD = CLK_PERIOD / 2;        // 时钟半周期 (ns)
    localparam LOOP_COUNT     = WEIGHT_TOTAL_ROW / M;   // 计算需调用Top模块的Loop数

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n;
    
    // 控制接口
    reg                               host_start;
    reg                               host_uram_batch_ready;
    wire                              system_done;

    // URAM 权重配置接口
    reg                               host_we_uram;
    reg [$clog2(URAM_DEPTH)-1:0]      host_addr_uram;
    reg [WIDE_BUS_WIDTH-1:0]          host_data_uram;

    // B 向量配置接口
    reg                               host_we_act;
    reg [$clog2(B_VEC_LEN)-1:0]       host_addr_act;
    reg [7:0]                         host_data_act;
    reg                               host_start_broadcast_act;
    wire                              broadcast_done_act;

    reg                               host_we_si;
    reg [T_WIDTH-1:0]                 host_addr_si;
    reg [T_WIDTH-1:0]                 host_data_si;
    reg                               host_start_broadcast_si;
    wire                              broadcast_done_si;

    // 结果读取接口
    reg                               out_rd_en;
    reg [$clog2(M)-1:0]               out_rd_addr;
    wire [OUT_WIDTH-1:0]              out_data;

    // 模拟存储器 - 用于加载输入数据和保存输出结果
    reg [7:0]                         mem_b [0: B_VEC_LEN-1];
    reg [T_WIDTH-1:0]                 mem_si [0: 2**T_WIDTH-1]; 
    reg [A_PACKED_WIDTH-1:0]          temp_mem_a [0: URAM_DEPTH-1]; 
    reg [A_PACKED_WIDTH-1:0]          shadow_mem_a [0: CU_NUMS-1][0: URAM_DEPTH-1]; 
    
    // 完整所有行计算结果与 Golden 数据存放区
    reg [OUT_WIDTH-1:0]               final_results [0: WEIGHT_TOTAL_ROW-1];
    reg [31:0]                        golden_mem    [0: WEIGHT_TOTAL_ROW-1];
    
    // 变量
    integer cu, i, j;
    integer loop_idx, file_idx;
    integer gemv_idx;
    integer gemv_err_cnt [0:GEMV_NUM-1]; // 记录每一次 GEMV 的错误数
    integer total_gemm_err_cnt;          // 记录整个 GEMM 的总错误数
  
    integer global_cycle_cnt;            // 全局时钟周期计数器
    integer prof_act_ext_to_bram  [0:GEMV_NUM-1];
    integer prof_act_bram_to_cu   [0:GEMV_NUM-1];
    integer prof_si_ext_to_bram   ;
    integer prof_si_bram_to_cu    ;
    integer prof_wt_ext_to_uram   [0:GEMV_NUM-1];
    integer prof_compute          [0:GEMV_NUM-1]; // 包含 URAM->BRAM->CU 以及底层计算
    integer prof_out_bram_to_ext  [0:GEMV_NUM-1];
    integer tmp_cycle_start;          

    // =========================================================================
    // 3. 时钟与初始化
    // =========================================================================
    initial begin 
        clk = 0;
        global_cycle_cnt = 0;
    end
    always #CLK_HALF_PERIOD clk = ~clk; // ~300MHz

    always @(posedge clk) begin
        global_cycle_cnt <= global_cycle_cnt + 1;
    end

    task init_signals();
        begin
            rst_n = 0;
            host_start = 0;
            host_uram_batch_ready = 0;
            host_we_uram = 0; host_addr_uram = 0; host_data_uram = 0;
            host_we_act = 0; host_addr_act = 0; host_data_act = 0;
            host_we_si = 0; host_addr_si = 0; host_data_si = 0;
            host_start_broadcast_act = 0;
            host_start_broadcast_si = 0;
            out_rd_en = 0; out_rd_addr = 0;

            total_gemm_err_cnt = 0;
            for (i = 0; i < GEMV_NUM; i = i + 1) begin
                gemv_err_cnt[i] = 0;
                prof_act_ext_to_bram[i]  = 0;
                prof_act_bram_to_cu[i]   = 0;
                prof_si_ext_to_bram       = 0;
                prof_si_bram_to_cu        = 0;
                prof_wt_ext_to_uram[i]   = 0;
                prof_compute[i]          = 0;
                prof_out_bram_to_ext[i]  = 0;
            end

            #100;
            rst_n = 1;
            #50;
        end
    endtask

    // =========================================================================
    // 4. 数据加载与读取任务
    // =========================================================================

    // Step 1: 加载并配置向量 B 和 SI
    task config_b_vector;
        input integer g_idx;
        string b_path;
        integer start_cc; // 局部时间记录
        begin
            $display("[%0t] Loading Vector B for GEMV %0d...", $time, g_idx);
            b_path = $sformatf("%s/b/%s/b_%04d.txt", base_dir, target_layer, g_idx);
            $readmemh(b_path, mem_b);
            
            // 统计 Act：外部写入 BRAM 用时
            start_cc = global_cycle_cnt;
            for (i = 0; i < B_VEC_LEN; i = i + 1) begin
                @(negedge clk);
                host_we_act   = 1;
                host_addr_act = i;
                host_data_act = mem_b[i];
            end
            @(negedge clk);
            host_we_act = 0;
            prof_act_ext_to_bram[g_idx] = prof_act_ext_to_bram[g_idx] + (global_cycle_cnt - start_cc);
  
            // 统计 Act：BRAM 广播到底层 CU 用时
            start_cc = global_cycle_cnt;
            @(negedge clk);
            host_start_broadcast_act = 1;
            @(negedge clk);
            host_start_broadcast_act = 0;
            
            wait(broadcast_done_act == 1'b1);
            prof_act_bram_to_cu[g_idx] = prof_act_bram_to_cu[g_idx] + (global_cycle_cnt - start_cc);

            $display("[%0t] Vector B Broadcast Done for GEMV %0d.", $time, g_idx);
        end
    endtask

    task config_si_vector;
        string si_path;
        integer start_cc; // 局部时间记录
        begin
            $display("[%0t] Loading SI for all CUs...", $time);
            si_path = $sformatf("%s/SI.txt", base_dir);
            $readmemh(si_path, mem_si);
            
            // 统计 SI：外部写入 BRAM 用时
            start_cc = global_cycle_cnt;
            for (i = 0; i < 2**T_WIDTH; i = i + 1) begin
                @(negedge clk);
                host_we_si   = 1;
                host_addr_si = i;
                host_data_si = mem_si[i];
            end
            @(negedge clk);
            host_we_si = 0;
            prof_si_ext_to_bram = prof_si_ext_to_bram + (global_cycle_cnt - start_cc);

            // 统计 SI：BRAM 广播到底层 CU 用时
            start_cc = global_cycle_cnt;
            @(negedge clk);
            host_start_broadcast_si = 1;
            @(negedge clk);
            host_start_broadcast_si = 0;
            
            wait(broadcast_done_si == 1'b1);
            prof_si_bram_to_cu = prof_si_bram_to_cu + (global_cycle_cnt - start_cc);
            
            $display("[%0t] Vector SI Loaded for all CUs.", $time);
        end
    endtask

    // Step 2: 根据当前的循环次数，加载对应的权重块进入 L2 URAM
    task config_a_weights;
        input integer l_idx;
        string a_path;
        integer start_cc; // 局部时间记录
        begin
            $display("[%0t] [Loop %0d] Loading Weights A into L2 URAM...", $time, l_idx);
            // 先从文件读入内存
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                file_idx = l_idx * CU_NUMS + cu; 
                a_path = $sformatf("%s/a_packed/%s/a_%04d.txt", base_dir, target_layer, file_idx);
                $readmemh(a_path, temp_mem_a);
                for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                    shadow_mem_a[cu][j] = temp_mem_a[j];
                end
            end

            // 写入 URAM (宽总线拼接，带 Batch Ready 脉冲)
            // 要改：加入等待CU_ALL_READY信号的逻辑，确保CU准备好接收下一批数据
            for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                @(negedge clk);
                host_we_uram   = 1;
                host_addr_uram = j;
                for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                    host_data_uram[cu * A_PACKED_WIDTH +: A_PACKED_WIDTH] = shadow_mem_a[cu][j];
                end
                
                // 如果写满了一个 Batch
                if ((j + 1) % BATCH_SIZE == 0) begin
                    @(negedge clk);
                    host_we_uram = 0;               // 暂停写一拍
                    host_uram_batch_ready = 1;      // 发送 Batch 写入uram完毕 脉冲
                    @(negedge clk);
                    host_uram_batch_ready = 0;      // 撤销脉冲，继续下一个 Batch 写入
                end
            end
            @(negedge clk);
            host_we_uram = 0;
        end
    endtask

    // Step 3: 读取本轮 Top 运算的 M 行 结果存入大数组中
    task read_and_save_results;
        input integer l_idx;
        integer start_cc; // 局部时间记录
        begin
            // 统计 Out：BRAM 读出到外部 用时
            start_cc = global_cycle_cnt;
            for (i = 0; i < M; i = i + 1) begin
                @(negedge clk);
                out_rd_en   = 1;
                out_rd_addr = i;
                @(posedge clk); // URAM/BRAM 读延迟
                @(negedge clk);
                final_results[l_idx * M + i] = out_data;
            end
            out_rd_en = 0;
            prof_out_bram_to_ext[gemv_idx] = prof_out_bram_to_ext[gemv_idx] + (global_cycle_cnt - start_cc);
            
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
            
            $display("\n[%0t] Loading Golden Results for GEMV %0d...", $time, g_idx);
            golden_path = $sformatf("%s/golden/%s/golden_%04d.txt", base_dir, target_layer, g_idx);
            $readmemh(golden_path, golden_mem);

            $display("\n==================================================================");
            $display("       GEMV %02d FINAL TEST REPORT & GOLDEN MATCH", g_idx);
            $display("==================================================================");
            
            for (i = 0; i < WEIGHT_TOTAL_ROW; i = i + 1) begin
                out_val  = final_results[i];
                gold_val = golden_mem[i];
                
                match_flag = (out_val === gold_val);
                
                $display("Row [%4d] | OUT(Dec): %11d | OUT(8-Hex): %08x | GOLD(Hex): %08x  %s", 
                         i, $signed(out_val), out_val, gold_val, match_flag ? "[PASS]" : "[FAIL]");

                if (!match_flag) begin
                    err_cnt = err_cnt + 1;
                end
            end
            
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
            $display("                  GEMM FINAL SUMMARY                   ");
            $display("##################################################################");
            for (i = 0; i < GEMV_NUM; i = i + 1) begin
                if (gemv_err_cnt[i] == 0)
                    $display("  [GEMV %02d] : PASS (0 errors)", i);
                else
                    $display("  [GEMV %02d] : FAIL (%0d errors)", i, gemv_err_cnt[i]);
            end
            $display("------------------------------------------------------------------");
            if (total_gemm_err_cnt == 0) begin
                $display("  >>> OVERALL GEMM TEST PASSED! ALL VECTORS MATCHED! <<< ");
            end else begin
                $display("  >>> OVERALL GEMM TEST FAILED! TOTAL MISMATCHES: %0d <<< ", total_gemm_err_cnt);
            end
            $display("##################################################################\n");
        end
    endtask

    // 性能测试周期汇总统计报告
    task print_performance_profiling;
        integer total_gemv_cycles;
        begin
            $display("\n##############################################################################################################");
            $display("                                      GEMM PERFORMANCE PROFILING REPORT                                       ");
            $display("##############################################################################################################");
            $display("----------------------------------------------------------------------------------------------------------");
            $display(" GEMV | Act:Ext->BRAM | Act:BRAM->CU | Overlapped (Host->URAM + Compute) | Out:BRAM->Ext | TOTAL CYCLES ");
            $display("----------------------------------------------------------------------------------------------------------");
            for (i = 0; i < GEMV_NUM; i = i + 1) begin
                total_gemv_cycles = prof_act_ext_to_bram[i] + prof_act_bram_to_cu[i] + 
                                    prof_compute[i] + prof_out_bram_to_ext[i];
                $display("  %02d  | %13d | %12d | %33d | %13d | %12d ", 
                         i, 
                         prof_act_ext_to_bram[i], 
                         prof_act_bram_to_cu[i],
                         prof_compute[i],         // 从一个Loop权重从外部注入开始，到该Loop计算完成，总时间
                         prof_out_bram_to_ext[i],
                         total_gemv_cycles);
            end
            $display("##############################################################################################################\n");
        end
    endtask

    // =========================================================================
    // 6. 主测试流程
    // =========================================================================
    initial begin
        init_signals();
        config_si_vector();
        
        // 外层循环：遍历所有 B 向量，执行完整的 GEMM
        for (gemv_idx = 0; gemv_idx < GEMV_NUM; gemv_idx = gemv_idx + 1) begin
            $display("\n******************************************************************");
            $display("***               STARTING GEMV SEQUENCE: %02d / %02d              ***", gemv_idx + 1, GEMV_NUM);
            $display("******************************************************************");

            // 内层大矩阵循环
            for (loop_idx = 0; loop_idx < LOOP_COUNT; loop_idx = loop_idx + 1) begin
                $display("\n------------------------------------------------------------------");
                $display(">>> GEMV %02d | Iteration: %0d / %0d <<<", gemv_idx, loop_idx + 1, LOOP_COUNT);
                
                if ((gemv_idx > 0) || (loop_idx > 0)) begin
                    $display("[%0t] [GEMV %02d Loop %0d] Applying Hard Reset to DUT...", $time, gemv_idx, loop_idx);
                    @(negedge clk);
                    rst_n = 0;
                    #100;
                    rst_n = 1;
                    #100;
                end
                
                // 加载对应的 B 向量 & SI
                config_b_vector(gemv_idx);
                
                // a) 触发系统，进入WAIT_URAM_PRE状态等待数据
                $display("[%0t] [GEMV %02d Loop %0d] Triggering System Start (Wait for URAM)...", $time, gemv_idx, loop_idx);
                
                tmp_cycle_start = global_cycle_cnt; // 记录包含Host写URAM及底层Compute在内的 Overall 总时间起点
                
                @(negedge clk);
                host_start = 1;
                @(negedge clk);
                host_start = 0;

                // b) 配置当前子矩阵块权重 A 
                config_a_weights(loop_idx);

                // c) 等待全流程计算完成
                wait(system_done == 1'b1);
                
                prof_compute[gemv_idx] = prof_compute[gemv_idx] + (global_cycle_cnt - tmp_cycle_start);
                $display("[%0t] [GEMV %02d Loop %0d] System Done Signal Received!", $time, gemv_idx, loop_idx);

                // d) 提取并累加保存本轮计算结果
                read_and_save_results(loop_idx);
                
                #100; 
            end

            // 完成当前一个完整的 GEMV 后，进行比对与统计
            compare_and_report(gemv_idx);
            
        end // end of gemv_idx loop

        // 全部GEMV计算完毕后，打印全局 GEMM 测试报告和性能报告
        print_gemm_summary();
        print_performance_profiling();

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
        .PIPE_STAGES(PIPE_STAGES),
        .CU_TYPE(CU_TYPE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),

        // 主机控制
        .host_start(host_start),
        .host_uram_batch_ready(host_uram_batch_ready),
        .system_done(system_done),

        // L2 URAM 配置
        .host_we_uram(host_we_uram),
        .host_addr_uram(host_addr_uram),
        .host_data_uram(host_data_uram),

        // L1 广播配置
        .host_we_act(host_we_act),
        .host_addr_act(host_addr_act),
        .host_data_act(host_data_act),
        .host_start_broadcast_act(host_start_broadcast_act),
        .broadcast_done_act(broadcast_done_act),

        .host_we_si(host_we_si),
        .host_addr_si(host_addr_si),
        .host_data_si(host_data_si),
        .host_start_broadcast_si(host_start_broadcast_si),
        .broadcast_done_si(broadcast_done_si),

        // 结果输出
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_data(out_data)
    );

endmodule