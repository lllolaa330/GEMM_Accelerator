`timescale 1ns / 1ps

module tb_Compute_Core_Array;

    // =========================================================================
    // 1. 参数与配置
    // =========================================================================
    parameter CU_NUMS        = 4;
    parameter T_WIDTH        = 8;  
    parameter TILE_SIZE      = 64; // 为了加快仿真使用 64，若跑全量可改为 256
    parameter M = TILE_SIZE, N = TILE_SIZE;                         
    parameter B_DATA_BITS = 8, PSUM_WIDTH = 24, OUT_WIDTH = 32;
    parameter WINDOW_SIZE = 16;

    // 自动推导参数
    localparam A_PACKED_WIDTH    = 10 + T_WIDTH; // 18-bit
    localparam BATCH_NUMS        = N / T_WIDTH;
    localparam BATCH_SIZE        = M * 4;
    localparam WIDE_BUS_WIDTH    = CU_NUMS * A_PACKED_WIDTH; // 774-bit
    localparam OUT_WIDE_WIDTH    = CU_NUMS * OUT_WIDTH;      // 1376-bit

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n;
    
    // 广播中心接口 (Host -> L2)
    reg                               host_we_act;
    reg [7:0]                         host_addr_act;
    reg [7:0]                         host_data_act;
    reg                               start_broadcast;
    wire                              broadcast_done;

    // DMA 宽总线接口 (L2 -> PingPong BRAM)
    reg                               ping_pong_sel;
    reg                               dma_wr_en;
    reg [$clog2(BATCH_SIZE)-1:0]      dma_wr_addr;
    reg [WIDE_BUS_WIDTH-1:0]          dma_wr_data_wide;

    // 计算控制接口 (Control -> CUs)
    reg                               start_compute;
    reg [$clog2(BATCH_NUMS)-1:0]      current_k_offset;
    wire                              all_batch_done;
    wire                              all_matrix_done;

    // 结果读出接口 (CUs -> Accumulator/Host)
    reg                               out_rd_en;
    reg [$clog2(M)-1:0]               out_rd_addr;
    wire [OUT_WIDE_WIDTH-1:0]         out_data_wide;

    // 存储阵列
    reg [7:0]                         mem_b [0: N-1]; 
    reg [A_PACKED_WIDTH-1:0]          mem_a [0: CU_NUMS-1][0: (BATCH_SIZE * BATCH_NUMS - 1)]; 

    // 统计变量
    integer cu, i, j, batch;
    time    start_time, end_time;
    string  base_dir = "/media/ese/HDD-Data/vivado_projects/0409data_llama3";

    // =========================================================================
    // 3. 时钟逻辑
    // =========================================================================
    parameter real CLK_FREQ_MHZ = 300.0;
    parameter real CLK_PERIOD_NS = 1000.0 / CLK_FREQ_MHZ; 
    initial clk = 0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // =========================================================================
    // 4. 任务定义 (Tasks)
    // =========================================================================

    // A. 系统初始化
    task init_signals();
        begin
            rst_n = 0; 
            host_we_act = 0; host_addr_act = 0; host_data_act = 0; start_broadcast = 0;
            ping_pong_sel = 0; dma_wr_en = 0; dma_wr_addr = 0; dma_wr_data_wide = 0;
            start_compute = 0; current_k_offset = 0;
            out_rd_en = 0; out_rd_addr = 0;
            #(CLK_PERIOD_NS * 30); 
            rst_n = 1; 
            #(CLK_PERIOD_NS * 5);
        end
    endtask

    // B. 加载 B 向量并广播至所有 CU
    task config_and_broadcast_b();
        string b_path;
        begin
            $display("\n[%0t] [Task 1] Loading Vector B and Broadcasting...", $time);
            b_path = $sformatf("%s/b.txt", base_dir);
            $readmemh(b_path, mem_b);
            
            // 写入广播 Buffer
            for (i = 0; i < N; i = i + 1) begin
                @(negedge clk);
                host_we_act   = 1; 
                host_addr_act = i[7:0]; 
                host_data_act = mem_b[i];
            end
            @(negedge clk); 
            host_we_act = 0;
            
            // 触发内部广播机制
            @(negedge clk);
            start_broadcast = 1;
            @(negedge clk);
            start_broadcast = 0;
            
            // 等待广播完成
            wait(broadcast_done == 1'b1);
            $display("[%0t] [Task 1] Broadcast Complete!", $time);
        end
    endtask

    // C. 从本地目录加载 43 个 CU 的 A 权重切片
    // 一维临时数组
    reg [A_PACKED_WIDTH-1:0] temp_mem [0: (BATCH_SIZE * BATCH_NUMS - 1)]; 
    task load_all_weights();
        string a_path;
        begin
            $display("\n[%0t] [Task 2] Loading 43 Weight Files...", $time);
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                a_path = $sformatf("%s/a/up_proj/a_%04d.txt", base_dir, cu + 1);
                
                // 先读入 1D 数组，再赋给 2D 数组
                $readmemh(a_path, temp_mem);
                for (j = 0; j < BATCH_SIZE * BATCH_NUMS; j = j + 1) begin
                    mem_a[cu][j] = temp_mem[j];
                end
            end
            $display("[%0t] [Task 2] All %0d Files Loaded Successfully.", $time, CU_NUMS);
        end
    endtask

    // D. 核心数据流控制：DMA 搬运 -> Ping-Pong 切换 -> CU 计算
    task run_gemv_dataflow();
        begin
            $display("\n[%0t] [Task 3] Starting Ping-Pong Dataflow...", $time);
            start_time = $time;
            
            ping_pong_sel = 0; // 初始 DMA 写入 Ping (Bank 0)

            for (batch = 0; batch < BATCH_NUMS; batch = batch + 1) begin
                $display("[%0t]   -> Processing Batch %0d / %0d", $time, batch+1, BATCH_NUMS);
                
                // 1. DMA 搬运：将当前 Batch 的数据拼接到 774-bit 宽总线写入 BRAM
                for (j = 0; j < BATCH_SIZE; j = j + 1) begin
                    @(negedge clk);
                    dma_wr_en   = 1;
                    dma_wr_addr = j;
                    for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                        dma_wr_data_wide[cu * A_PACKED_WIDTH +: A_PACKED_WIDTH] = mem_a[cu][batch * BATCH_SIZE + j];
                    end
                end
                @(negedge clk);
                dma_wr_en = 0;

                // 2. Ping-Pong 翻转，让 CU 读刚刚写进去的 Bank，DMA 下一轮写对面的 Bank
                ping_pong_sel = ~ping_pong_sel; 
                
                // 3. 启动 CU 阵列计算
                @(negedge clk);
                current_k_offset = batch;
                start_compute    = 1;
                @(negedge clk);
                start_compute    = 0; // 产生启动单脉冲

                // 4. 等待 43 个 CU 全都跑完本轮 Batch
                wait(all_batch_done == 1'b1);
                
                // Note: 在真实的工程中，步骤1(下个Batch的搬运)会和步骤3,4(当前Batch的计算)同时进行，
                //       为了仿真时序最安全清晰，这里采用了严格的串行模拟。
            end
            
            // 等待最终全局完成信号
            wait(all_matrix_done == 1'b1);
            end_time = $time;
            $display("[%0t] [Task 3] GEMV Dataflow Finished! Core Compute Time: %0t", $time, (end_time - start_time));
        end
    endtask

    // E. 读出并打印结果 (提取宽总线数据)
    task read_and_print_results();
        reg signed [OUT_WIDTH-1:0] val;
        begin
            $display("\n==================================================================");
            $display("                      GEMV OUTPUT RESULTS                         ");
            $display("==================================================================");
            
            for (i = 0; i < M; i = i + 1) begin
                // 1. 给出读地址
                @(negedge clk);
                out_rd_en   = 1; 
                out_rd_addr = i;
                
                // 2. 抵消 1 拍的 SRAM 读延迟
                @(posedge clk);
                @(negedge clk); 
                
                // 3. 提取 43 个 CU 在这一行的结果并打印
                $write("Row [%3d] : ", i);
                for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                    val = out_data_wide[cu * OUT_WIDTH +: OUT_WIDTH];
                    $write("%8d ", val);
                end
                $display(""); // 换行
            end
            
            out_rd_en = 0;
            $display("==================================================================\n");
        end
    endtask

    // =========================================================================
    // 5. 主控制流程 (Main Control)
    // =========================================================================
    initial begin
        init_signals();
        
        config_and_broadcast_b();
        
        load_all_weights();
        
        run_gemv_dataflow();
        
        read_and_print_results();
        
        $finish;
    end

    // =========================================================================
    // 6. DUT 实例化
    // =========================================================================
    Compute_Core_Array #(
        .CU_NUMS(CU_NUMS), .M(M), .N(N), .T_WIDTH(T_WIDTH), 
        .BATCH_SIZE(BATCH_SIZE), .BATCH_NUMS(BATCH_NUMS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        
        // 广播中心接口
        .host_we_act(host_we_act),
        .host_addr_act(host_addr_act),
        .host_data_act(host_data_act),
        .start_broadcast(start_broadcast),
        .broadcast_done(broadcast_done),

        // 宽总线数据流接口
        .ping_pong_sel(ping_pong_sel),
        .dma_wr_en(dma_wr_en),
        .dma_wr_addr(dma_wr_addr),
        .dma_wr_data_wide(dma_wr_data_wide),

        // 计算控制与监控接口
        .start_compute(start_compute),
        .current_k_offset(current_k_offset),
        .all_batch_done(all_batch_done),
        .all_matrix_done(all_matrix_done),

        // 结果输出
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_data_wide(out_data_wide)
    );

endmodule