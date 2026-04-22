`timescale 1ns / 1ps

module tb_GEMV_Accelerator_Top;

    // =========================================================================
    // 1. 参数与配置 (需与 RTL 保持一致)
    // =========================================================================
    parameter B_VEC_LEN      = 4096; 
    parameter M              = 256;
    parameter N              = 256;
    parameter T_WIDTH        = 8;                       
    parameter OUT_WIDTH      = 32;
    parameter PIPE_STAGES    = 2;

    localparam CU_NUMS        = B_VEC_LEN / N;          // 16
    localparam A_PACKED_WIDTH = 10 + T_WIDTH;           // 18
    localparam BATCH_NUMS     = N / T_WIDTH;            // 32
    localparam BATCH_SIZE     = M * 4;                  // 1024
    localparam WIDE_BUS_WIDTH = CU_NUMS * A_PACKED_WIDTH; // 16 * 18 = 288
    localparam URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;  // 32768

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
    reg [A_PACKED_WIDTH-1:0]          temp_mem_a [0: URAM_DEPTH-1]; // 临时加载单CU文件
    reg [A_PACKED_WIDTH-1:0]          shadow_mem_a [0: CU_NUMS-1][0: URAM_DEPTH-1]; 

    integer cu, i, j;
    string  base_dir = "C:/Users/lllolaa/Desktop/0422_data"; // 请确保路径正确

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
            #100;
            rst_n = 1;
            #50;
        end
    endtask

    // =========================================================================
    // 4. 数据加载任务 (与 tb_Compute_Core_Array 格式一致)
    // =========================================================================

    // Step 1: 加载并配置向量 B
    task config_b_vector();
        string b_path;
        begin
            $display("[%0t] [Step 1] Loading Vector B...", $time);
            b_path = $sformatf("%s/b.txt", base_dir);
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
            $display("[%0t] [Step 1] Vector B Broadcast Done.", $time);
        end
    endtask

    // Step 2: 加载所有权重并写入 Top 层的 L2 URAM
    task config_a_weights();
        string a_path;
        begin
            $display("[%0t] [Step 2] Loading Weights A into L2 URAM...", $time);
            // 先读入内存
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                a_path = $sformatf("%s/a/a_%04d.txt", base_dir, cu);
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
            $display("[%0t] [Step 2] L2 URAM Configuration Complete.", $time);
        end
    endtask

    // =========================================================================
    // 5. 主测试流程
    // =========================================================================
    initial begin
        init_signals();
        
        // 1. 配置数据
        config_b_vector();
        config_a_weights();
        
        // 2. 启动系统 (一键启动)
        $display("[%0t] [Step 3] Triggering System Start...", $time);
        @(negedge clk);
        host_start = 1;
        @(negedge clk);
        host_start = 0;

        // 3. 等待全局控制器报告完成
        // 全局控制器会经历: PRELOAD -> SYNC_RUN -> DRAIN -> ACCUM -> DONE
        wait(system_done == 1'b1);
        $display("[%0t] [Step 4] System Done Signal Received!", $time);

        // 4. 读取并打印结果
        read_results();

        #100;
        $display("[%0t] Simulation Finished.", $time);
        $finish;
    end

    // =========================================================================
    // 6. 结果读取任务
    // =========================================================================
    task read_results();
        begin
            $display("\n==================================================================");
            $display("             GEMV TOP LEVEL FINAL RESULTS (ACCUMULATED)           ");
            $display("==================================================================");
            for (i = 0; i < M; i = i + 1) begin
                @(negedge clk);
                out_rd_en   = 1;
                out_rd_addr = i;
                @(posedge clk); // URAM/BRAM 读延迟
                @(negedge clk);
                $display("Row [%3d] : %d", i, $signed(out_data));
            end
            out_rd_en = 0;
            $display("==================================================================\n");
        end
    endtask

    // =========================================================================
    // 7. 实例化 DUT
    // =========================================================================
    GEMV_Accelerator_Top #(
        .M(M),
        .N(N),
        .B_VEC_LEN(B_VEC_LEN),
        .T_WIDTH(T_WIDTH),
        .OUT_WIDTH(OUT_WIDTH),
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