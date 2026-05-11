`timescale 1ns / 1ps

module tb_GEMV_Accelerator_Top;

    // =========================================================================
    // 1. 参数与配置 (注意：批量测试时，前三个参数由外部仿真脚本动态传入覆盖)
    // =========================================================================
    // LLaMA3-8B 标准层尺寸参考: 
    // q_proj/o_proj: 4096x4096 | k_proj/v_proj: 1024x4096 
    // gate_proj/up_proj: 14336x4096 | down_proj: 4096x14336
    parameter string target_layer  = "k_proj";          // 目标层名称
    parameter WEIGHT_TOTAL_ROW     = 1024;              // 完整的权重总行数
    parameter B_VEC_LEN            = 4096;              // 完整的权重总列数 (B向量长度)
    parameter CU_TYPE              = 3;                 // 0:Static, 1:Dynamic, 2:SlidingWindow, 3:DSPBaseline

    localparam GEMV_NUM            = 2;                // 固定的总测试次数
    string    base_dir             = "C:/Users/lllolaa/Desktop/data_L38/data_L38"; 
    parameter CLK_MHZ              = 300;               // 时钟频率 (MHz)

    parameter M              = 256;                     // 底层 CU 计算的行数 
    parameter N              = 256;                     // 底层 CU 计算的列数 
    parameter T_WIDTH        = 8;                       // 比特重用位宽
    parameter OUT_WIDTH      = 32;
    parameter RAM_LATENCY    = 3;
    parameter PIPE_STAGES    = 2;

    localparam CU_NUMS        = B_VEC_LEN / N;          
    localparam A_PACKED_WIDTH = 10 + T_WIDTH;           
    localparam BATCH_NUMS     = N / T_WIDTH;            
    localparam BATCH_SIZE     = M * 4;                  
    localparam WIDE_BUS_WIDTH = CU_NUMS * A_PACKED_WIDTH; 
    localparam URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;  
    localparam CLK_PERIOD     = 1000 / CLK_MHZ;         
    localparam CLK_HALF_PERIOD = CLK_PERIOD / 2;        
    localparam LOOP_COUNT     = WEIGHT_TOTAL_ROW / M;   

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n, rst_batch_n;
    
    reg                               host_start;
    reg                               host_uram_batch_ready;
    wire                              system_done;

    reg                               host_we_uram;
    reg [$clog2(URAM_DEPTH)-1:0]      host_addr_uram;
    reg [WIDE_BUS_WIDTH-1:0]          host_data_uram;

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

    reg                               out_rd_en;
    reg [$clog2(M)-1:0]               out_rd_addr;
    wire [OUT_WIDTH-1:0]              out_data;

    reg [7:0]                         mem_b [0: B_VEC_LEN-1];
    reg [T_WIDTH-1:0]                 mem_si [0: 2**T_WIDTH-1]; 
    reg [A_PACKED_WIDTH-1:0]          temp_mem_a [0: URAM_DEPTH-1]; 
    reg [A_PACKED_WIDTH-1:0]          shadow_mem_a [0: CU_NUMS-1][0: URAM_DEPTH-1]; 
    
    reg [OUT_WIDTH-1:0]               final_results [0: WEIGHT_TOTAL_ROW-1];
    reg [31:0]                        golden_mem    [0: WEIGHT_TOTAL_ROW-1];
    
    // =========================================================================
    // 性能与测试变量
    // =========================================================================
    integer cu, i, j, loop_idx, file_idx, gemv_idx;
    integer gemv_err_cnt [0:GEMV_NUM-1]; 
    integer total_gemm_err_cnt;          
  
    integer global_cycle_cnt;            
    integer prof_act_ext_to_bram, prof_act_bram_to_cu;
    integer prof_si_ext_to_bram,  prof_si_bram_to_cu;
    integer prof_compute; 
    integer prof_out_bram_to_ext;
    integer tmp_cycle_start;          

    integer csv_fd; // CSV 文件句柄

    // =========================================================================
    // 3. 时钟与初始化
    // =========================================================================
    initial begin 
        clk = 0;
        global_cycle_cnt = 0;
    end
    always #CLK_HALF_PERIOD clk = ~clk;

    always @(posedge clk) begin
        global_cycle_cnt <= global_cycle_cnt + 1;
    end

    task init_signals();
        begin
            rst_n = 0; rst_batch_n = 0; host_start = 0; host_uram_batch_ready = 0;
            host_we_uram = 0; host_addr_uram = 0; host_data_uram = 0;
            host_we_act = 0; host_addr_act = 0; host_data_act = 0;
            host_we_si = 0; host_addr_si = 0; host_data_si = 0;
            host_start_broadcast_act = 0; host_start_broadcast_si = 0;
            out_rd_en = 0; out_rd_addr = 0;

            total_gemm_err_cnt = 0;
            prof_act_ext_to_bram = 0; prof_act_bram_to_cu = 0;
            prof_si_ext_to_bram = 0;  prof_si_bram_to_cu = 0;
            prof_compute = 0;         prof_out_bram_to_ext = 0;
        end
    endtask

    // =========================================================================
    // 4. 数据加载与读取任务 (优化了统计方式)
    // =========================================================================
    task config_b_vector;
        input integer g_idx;
        string b_path;
        integer start_cc;
        begin
            $display("[%0t] Loading Vector B for GEMV %0d...", $time, g_idx);
            b_path = $sformatf("%s/b/%s/b_%04d.txt", base_dir, target_layer, g_idx);
            $readmemh(b_path, mem_b);
            
            start_cc = global_cycle_cnt;
            for (i = 0; i < B_VEC_LEN; i = i + 1) begin
                @(negedge clk);
                host_we_act = 1; host_addr_act = i; host_data_act = mem_b[i];
            end
            @(negedge clk); host_we_act = 0;
            prof_act_ext_to_bram = prof_act_ext_to_bram + (global_cycle_cnt - start_cc);
  
            start_cc = global_cycle_cnt;
            @(negedge clk); host_start_broadcast_act = 1;
            @(negedge clk); host_start_broadcast_act = 0;
            wait(broadcast_done_act == 1'b1);
            prof_act_bram_to_cu = prof_act_bram_to_cu + (global_cycle_cnt - start_cc);
        end
    endtask

    task config_si_vector;
        string si_path;
        integer start_cc;
        begin
            $display("[%0t] Loading SI for all CUs...", $time);
            si_path = $sformatf("%s/SI.txt", base_dir);
            $readmemh(si_path, mem_si);
            
            start_cc = global_cycle_cnt;
            for (i = 0; i < 2**T_WIDTH; i = i + 1) begin
                @(negedge clk);
                host_we_si = 1; host_addr_si = i; host_data_si = mem_si[i];
            end
            @(negedge clk); host_we_si = 0;
            prof_si_ext_to_bram = prof_si_ext_to_bram + (global_cycle_cnt - start_cc);

            start_cc = global_cycle_cnt;
            @(negedge clk); host_start_broadcast_si = 1;
            @(negedge clk); host_start_broadcast_si = 0;
            wait(broadcast_done_si == 1'b1);
            prof_si_bram_to_cu = prof_si_bram_to_cu + (global_cycle_cnt - start_cc);
        end
    endtask

    task config_a_weights;
        input integer l_idx;
        string a_path;
        begin
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                file_idx = l_idx * CU_NUMS + cu; 
                a_path = $sformatf("%s/a_packed/%s/a_%04d.txt", base_dir, target_layer, file_idx);
                $readmemh(a_path, temp_mem_a);
                for (j = 0; j < URAM_DEPTH; j = j + 1) shadow_mem_a[cu][j] = temp_mem_a[j];
            end

            for (j = 0; j < URAM_DEPTH; j = j + 1) begin
                @(negedge clk);
                host_we_uram = 1; host_addr_uram = j;
                for (cu = 0; cu < CU_NUMS; cu = cu + 1)
                    host_data_uram[cu * A_PACKED_WIDTH +: A_PACKED_WIDTH] = shadow_mem_a[cu][j];
                
                if ((j + 1) % BATCH_SIZE == 0) begin
                    @(negedge clk); host_we_uram = 0; host_uram_batch_ready = 1;
                    @(negedge clk); host_uram_batch_ready = 0; 
                end
            end
            @(negedge clk); host_we_uram = 0;
        end
    endtask

    task read_and_save_results;
        input integer l_idx;
        integer start_cc;
        begin
            start_cc = global_cycle_cnt;
            for (i = 0; i < M; i = i + 1) begin
                @(negedge clk);
                out_rd_en = 1; out_rd_addr = i;
                @(posedge clk); @(negedge clk);
                final_results[l_idx * M + i] = out_data;
            end
            out_rd_en = 0;
            prof_out_bram_to_ext = prof_out_bram_to_ext + (global_cycle_cnt - start_cc);
        end
    endtask

    task compare_and_report;
        input integer g_idx;
        integer err_cnt;
        reg [31:0] out_val, gold_val;
        string golden_path;
        begin
            err_cnt = 0;
            golden_path = $sformatf("%s/golden/%s/golden_%04d.txt", base_dir, target_layer, g_idx);
            $readmemh(golden_path, golden_mem);
            
            for (i = 0; i < WEIGHT_TOTAL_ROW; i = i + 1) begin
                out_val = final_results[i]; gold_val = golden_mem[i];
                if (out_val !== gold_val) err_cnt = err_cnt + 1;
            end
            gemv_err_cnt[g_idx] = err_cnt;
            total_gemm_err_cnt = total_gemm_err_cnt + err_cnt;
        end
    endtask

    // =========================================================================
    // 5. 保存结果到 CSV 任务
    // =========================================================================
    task save_to_csv;
        integer total_cycles;
        begin
            total_cycles = prof_act_ext_to_bram + prof_act_bram_to_cu + prof_si_ext_to_bram + prof_si_bram_to_cu + prof_compute + prof_out_bram_to_ext;
            
            // "a" 模式为追加模式，便于批量测试记录积累
            csv_fd = $fopen("gemv_batch_results.csv", "a");
            if (csv_fd) begin
                // 如果是新文件，可以手动在外部先创建好表头，或者直接在这里追加数据行
                $fdisplay(csv_fd, "%0d,%s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                          CU_TYPE, target_layer, WEIGHT_TOTAL_ROW, B_VEC_LEN, GEMV_NUM, 
                          total_gemm_err_cnt, prof_act_ext_to_bram, prof_act_bram_to_cu,
                          prof_si_ext_to_bram, prof_si_bram_to_cu, prof_compute, 
                          prof_out_bram_to_ext, total_cycles);
                $fclose(csv_fd);
                $display("\n>>> Results successfully saved to gemv_batch_results.csv <<<");
            end else begin
                $display("ERROR: Cannot open CSV file for writing!");
            end
        end
    endtask

    // =========================================================================
    // 6. 主测试流程 (修复了逻辑嵌套)
    // =========================================================================
    initial begin
        init_signals();
        
        $display("\n=======================================================================");
        $display("   STARTING BATCH TEST | CU_TYPE: %0d | LAYER: %s", CU_TYPE, target_layer);
        $display("   SIZE: [W_ROW: %0d, B_LEN: %0d] | GEMV_NUM: %0d", WEIGHT_TOTAL_ROW, B_VEC_LEN, GEMV_NUM);
        $display("=======================================================================\n");

        for (gemv_idx = 0; gemv_idx < GEMV_NUM; gemv_idx = gemv_idx + 1) begin
            
            // Batch 复位
            rst_batch_n = 0; #100;  rst_batch_n = 1; #100;
            
            // 加载 B向量
            config_b_vector(gemv_idx);
            
            // 加载 SI 向量配置
            if (CU_TYPE == 0) begin
                config_si_vector();
            end

            for (loop_idx = 0; loop_idx < LOOP_COUNT; loop_idx = loop_idx + 1) begin
                // Loop 复位
                rst_n = 0;  #100; rst_n = 1;  #100;

                tmp_cycle_start = global_cycle_cnt; 
                @(negedge clk); host_start = 1;
                @(negedge clk); host_start = 0;

                config_a_weights(loop_idx);
                
                wait(system_done == 1'b1);
                prof_compute = prof_compute + (global_cycle_cnt - tmp_cycle_start);
                
                read_and_save_results(loop_idx);
                #50; 
            end
            
            compare_and_report(gemv_idx);
        end

        // 统一保存到CSV
        save_to_csv();

        #200;
        $display("[%0t] Simulation Successfully Finished.", $time);
        $finish;
    end

    // =========================================================================
    // 7. 实例化 DUT
    // =========================================================================
    GEMV_Accelerator_Top #(
        .M(M), .N(N), .B_VEC_LEN(B_VEC_LEN), .T_WIDTH(T_WIDTH),
        .OUT_WIDTH(OUT_WIDTH), .RAM_LATENCY(RAM_LATENCY), .PIPE_STAGES(PIPE_STAGES),
        .CU_TYPE(CU_TYPE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .host_start(host_start), .host_uram_batch_ready(host_uram_batch_ready), .system_done(system_done),
        .host_we_uram(host_we_uram), .host_addr_uram(host_addr_uram), .host_data_uram(host_data_uram),
        .host_we_act(host_we_act), .host_addr_act(host_addr_act), .host_data_act(host_data_act),
        .host_start_broadcast_act(host_start_broadcast_act), .broadcast_done_act(broadcast_done_act),
        .host_we_si(host_we_si), .host_addr_si(host_addr_si), .host_data_si(host_data_si),
        .host_start_broadcast_si(host_start_broadcast_si), .broadcast_done_si(broadcast_done_si),
        .out_rd_en(out_rd_en), .out_rd_addr(out_rd_addr), .out_data(out_data)
    );

endmodule