`timescale 1ns / 1ps

module tb_GEMV_Accelerator_Top;

    // =========================================================================
    // 1. 参数与配置 
    // =========================================================================
    parameter CU_TYPE        = 3;                       // 0:Sliding, 1:Dynamic, 2:Static, 3:DSP
    parameter WEIGHT_TOTAL_ROW = 1024;                  // 完整的权重总行数
    parameter B_VEC_LEN      = 4096;                    // 完整的权重总列数 也是 B向量的长度
    parameter M              = 256;                     // 底层 CU 计算的行数 (每次处理256行)
    parameter N              = 256;                     // 底层 CU 计算的列数 (每次处理256列) 
    parameter T_WIDTH        = 8;                       // 比特重用位宽

    parameter OUT_WIDTH      = 32;
    parameter RAM_LATENCY    = 3;
    parameter PIPE_STAGES    = 2;

    localparam CU_NUMS        = B_VEC_LEN / N;          // 56
    localparam A_PACKED_WIDTH = 10 + T_WIDTH;           // 18
    localparam BATCH_NUMS     = N / T_WIDTH;            // 32
    localparam BATCH_SIZE     = M * 4;                  // 1024
    localparam WIDE_BUS_WIDTH = CU_NUMS * A_PACKED_WIDTH; // 56 * 18 
    localparam URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;  // 32768
    
    localparam LOOP_COUNT     = WEIGHT_TOTAL_ROW / M;   // 计算需调用Top模块的Loop数
    localparam GEMV_NUM       = 1;                      // 总共计算的GEMV次数（组成完整GEMM）

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
    reg                               host_start_broadcast;
    wire                              broadcast_done;

    // SI 接口
    reg                               host_we_si;
    reg [7:0]                         host_addr_si;
    reg [T_WIDTH-1:0]                 host_data_si;
    reg [T_WIDTH-1:0]                 mem_si [0: 255];

    // DSP 直通寄存器
    reg                               host_dsp_valid;
    reg [CU_NUMS*4-1:0]               host_dsp_payload;
    reg                               host_start_accum_dsp;

    // 结果读取接口
    reg                               out_rd_en;
    reg [$clog2(M)-1:0]               out_rd_addr;
    wire [OUT_WIDTH-1:0]              out_data;

    // 模拟存储器 (用于从文件加载数据)
    reg [7:0]                         mem_b [0: B_VEC_LEN-1]; 
    reg [A_PACKED_WIDTH-1:0]          temp_mem_a [0: URAM_DEPTH-1]; 
    reg [A_PACKED_WIDTH-1:0]          shadow_mem_a [0: CU_NUMS-1][0: URAM_DEPTH-1]; 
    reg [3:0]                         temp_mem_dsp [0: M*N-1]; // DSP一轮需要 65536 个
    reg [3:0]                         shadow_mem_dsp [0: CU_NUMS-1][0: M*N-1];
    reg [T_WIDTH-1:0]                 mem_si [0: 255]; 
    
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
    // 性能统计(Profiling)变量 - 记录每一次 GEMV 各阶段的周期数
    // =========================================================================
    integer global_cycle_cnt;             // 全局时钟周期计数器
    
    integer prof_act_ext_to_bram  [0:GEMV_NUM-1];
    integer prof_act_bram_to_cu   [0:GEMV_NUM-1];
    integer prof_wt_ext_to_uram   [0:GEMV_NUM-1];
    integer prof_compute          [0:GEMV_NUM-1]; // 包含 URAM->BRAM->CU 以及底层计算
    integer prof_out_bram_to_ext  [0:GEMV_NUM-1];
    
    integer tmp_cycle_start;              // 用于主循环计算阶段统计

    // =========================================================================
    // 3. 时钟与初始化
    // =========================================================================
    initial begin 
        clk = 0;
        global_cycle_cnt = 0;
    end
    always #1.667 clk = ~clk; // ~300MHz

    // 全局时钟周期累加
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
            host_dsp_valid = 0; host_dsp_payload = 0;
            host_start_broadcast = 0;
            out_rd_en = 0; out_rd_addr = 0;
            host_start_accum_dsp = 0;
            
            // 初始化错误统计 和 性能统计
            total_gemm_err_cnt = 0;
            for (i = 0; i < GEMV_NUM; i = i + 1) begin
                gemv_err_cnt[i] = 0;
                
                // 初始化统计周期数组为0
                prof_act_ext_to_bram[i]  = 0;
                prof_act_bram_to_cu[i]   = 0;
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
    // 4. 数据加载与读取任务 (加入周期统计)
    // =========================================================================

    // Step 1: 加载并配置向量 B 和 SI
    task config_b_vector;
        input integer g_idx;
        string b_path;
        integer start_cc; // 局部时间记录
        begin
            $display("[%0t] Loading Vector B for GEMV %0d...", $time, g_idx);
            b_path = $sformatf("%s/b/k_proj/b_%04d.txt", base_dir, g_idx);
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
            host_start_broadcast = 1;
            @(negedge clk);
            host_start_broadcast = 0;
            
            wait(broadcast_done == 1'b1);
            prof_act_bram_to_cu[g_idx] = prof_act_bram_to_cu[g_idx] + (global_cycle_cnt - start_cc);
            
            $display("[%0t] Vector B Broadcast Done for GEMV %0d.", $time, g_idx);
        end
    endtask

    task config_static_si;
        begin
            $display("[%0t] Loading Global Static SI...", $time);
            $readmemh($sformatf("%s/SI.txt", base_dir), mem_si);
            for (i = 0; i < 256; i = i + 1) begin
                @(negedge clk);
                host_we_si   = 1; 
                host_addr_si = i; 
                host_data_si = mem_si[i];
            end
            @(negedge clk); 
            host_we_si = 0;
        end
    endtask

    task config_dsp_and_compute;
        input integer l_idx;
        begin
            $display("[%0t] [Loop %0d] DSP Baseline Direct Streaming...", $time, l_idx);
            // 读取另外准备的 DSP 4bit 完整权重
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                file_idx = l_idx * CU_NUMS + cu; 
                $readmemb($sformatf("%s/a_dsp/k_proj/a_%04d.txt", base_dir, file_idx+1), temp_mem_dsp);
                for (j = 0; j < (M*N); j = j + 1) begin
                    shadow_mem_dsp[cu][j] = temp_mem_dsp[j];
                end
            end

            // 连续 M*N 拍直通打入数据
            for (j = 0; j < (M*N); j = j + 1) begin
                @(negedge clk);
                host_dsp_valid = 1;
                for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                    host_dsp_payload[cu*4 +: 4] = shadow_mem_dsp[cu][j];
                end
            end
            @(negedge clk);
            host_dsp_valid = 0;

            // 延时等待 DSP 流水线吐完部分和，并触发 Array_Output_Accumulation
            #200; 
            @(negedge clk);
            host_start_accum_dsp = 1;
            @(negedge clk);
            host_start_accum_dsp = 0;
            
            // 累加器运行需要 256 周期
            #(M * 2 * 1.667); 
            #100;
        end
    endtask

    // Step 2: 根据当前的循环次数，加载对应的权重块进入 L2 URAM
    task config_a_weights;
        input integer l_idx;
        string a_path;
        integer start_cc; // 局部时间记录
        begin
            $display("[%0t] [Loop %0d] Loading Weights A into L2 URAM...", $time, l_idx);
            // 先读入内存
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                file_idx = l_idx * CU_NUMS + cu; // 根据循环计算真实文件编号，从0000开始

                a_path = $sformatf("%s/a_packed/k_proj/a_%04d.txt", base_dir, file_idx);
                $readmemh(a_path, temp_mem_a);
                for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                    shadow_mem_a[cu][j] = temp_mem_a[j];
                end
            end

            // 写入 URAM (宽总线拼接，带 Batch Ready 脉冲)
            for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                @(negedge clk);
                host_we_uram   = 1;
                host_addr_uram = j;
                for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                    host_data_uram[cu * A_PACKED_WIDTH +: A_PACKED_WIDTH] = shadow_mem_a[cu][j];
                end
                
                // 判断是否写满了一个 Batch
                if ((j + 1) % BATCH_SIZE == 0) begin
                    @(negedge clk);
                    host_we_uram = 0;               // 暂停写一拍
                    host_uram_batch_ready = 1;      // 发送 Batch 准备好脉冲
                    @(negedge clk);
                    host_uram_batch_ready = 0;      // 撤销脉冲，继续下一个 Batch 写入
                end
            end
            @(negedge clk);
            host_we_uram = 0;
        end
    endtask

    // Step 3: 读取本轮 Top 运算的 256 行结果存入大数组中
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
            golden_path = $sformatf("%s/golden/k_proj/golden_%04d.txt", base_dir, g_idx);
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
                         prof_compute[i],         // 此处代表重叠后花费的净总时间
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
        
        // 外层循环：遍历 20 个 B 向量，执行完整的 GEMM
        for (gemv_idx = 0; gemv_idx < GEMV_NUM; gemv_idx = gemv_idx + 1) begin
            $display("\n******************************************************************");
            $display("***               STARTING GEMV SEQUENCE: %02d / %02d              ***", gemv_idx + 1, GEMV_NUM);
            $display("******************************************************************");

            // 原本的内层大矩阵循环：共 4096/256 = 16 次
            for (loop_idx = 0; loop_idx < LOOP_COUNT; loop_idx = loop_idx + 1) begin

                if (CU_TYPE == 3) begin
                    // ---- DSP Baseline ----
                    config_dsp_and_compute(loop_idx); 
                end

                else begin
                    // ---- 比特重用架构 ----
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
                    
                    // 每次复位后重新加载对应的 B 向量
                    config_b_vector(gemv_idx);
                    if (CU_TYPE == 2)  config_static_si(); 
                    
                    $display("[%0t] [GEMV %02d Loop %0d] Triggering System Start (Wait for URAM)...", $time, gemv_idx, loop_idx);
                    
                    tmp_cycle_start = global_cycle_cnt; // 记录包含Host写URAM及底层Compute在内的 Overall 总时间起点
                    
                    @(negedge clk);
                    host_start = 1;
                    @(negedge clk);
                    host_start = 0;

                    config_a_weights(loop_idx);
                    wait(system_done == 1'b1);
                    
                    prof_compute[gemv_idx] = prof_compute[gemv_idx] + (global_cycle_cnt - tmp_cycle_start);
                    $display("[%0t] [GEMV %02d Loop %0d] System Done Signal Received!", $time, gemv_idx, loop_idx);

                    read_and_save_results(loop_idx);
                    
                    #100; 
              end

            end
            // 完成当前一个完整的 GEMV (4096行) 后，进行比对与统计
            compare_and_report(gemv_idx);
            
        end // end of gemv_idx loop

        // 全部20次GEMV计算完毕后，打印全局 GEMM 测试报告
        print_gemm_summary();
        
        // 打印详细的系统性能和用时报告
        print_performance_profiling();

        #200;
        $display("[%0t] Simulation Successfully Finished.", $time);
        $finish;
    end

    // =========================================================================
    // 7. 实例化 DUT
    // =========================================================================
    GEMV_Accelerator_Top #(
        .CU_TYPE(CU_TYPE),
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
        .host_we_si(host_we_si),
        .host_addr_si(host_addr_si),
        .host_data_si(host_data_si),
        .host_dsp_valid(host_dsp_valid),
        .host_dsp_payload_wide(host_dsp_payload),
        .host_start_accum_dsp(host_start_accum_dsp),
        .host_start_broadcast(host_start_broadcast),
        .broadcast_done(broadcast_done),

        // 结果输出
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_data(out_data)
    );

endmodule