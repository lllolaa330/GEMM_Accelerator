`timescale 1ns / 1ps

module tb_Compute_Core_Array;

    // =========================================================================
    // 1. 参数与配置
    // =========================================================================
    parameter B_VEC_LEN      = 4096; 
    parameter M              = 256;
    parameter N              = 256;
    parameter T_WIDTH        = 8;                       
    parameter OUT_WIDTH      = 32;

    localparam CU_NUMS           = B_VEC_LEN / N; // 64
    localparam A_PACKED_WIDTH    = 10 + T_WIDTH; 
    localparam BATCH_NUMS        = N / T_WIDTH;
    localparam BATCH_SIZE        = M * 4;
    localparam WIDE_BUS_WIDTH    = CU_NUMS * A_PACKED_WIDTH; 
    
    // 地址位宽计算
    localparam B_ADDR_W          = $clog2(B_VEC_LEN); 

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n;
    
    // 广播中心接口
    reg                               host_we_act;
    reg [B_ADDR_W-1:0]                host_addr_act; 
    reg [7:0]                         host_data_act;
    reg                               start_broadcast;
    wire                              broadcast_done;

    // DMA 接口
    reg                               ping_pong_sel;
    reg                               dma_wr_en;
    reg [$clog2(BATCH_SIZE)-1:0]      dma_wr_addr;
    reg [WIDE_BUS_WIDTH-1:0]          dma_wr_data_wide;

    // 计算控制
    reg                               start_compute;
    reg [$clog2(BATCH_NUMS)-1:0]      current_k_offset;
    wire                              all_batch_done;
    wire                              all_matrix_done;

    // 累加器与输出控制
    reg                               start_accum;    
    wire                              accum_done;     
    reg                               out_rd_en;
    reg [$clog2(M)-1:0]               out_rd_addr;
    wire signed [OUT_WIDTH-1:0]       out_data;       

    // 存储阵列
    reg [7:0]                         mem_b [0: B_VEC_LEN-1]; 
    reg [A_PACKED_WIDTH-1:0]          mem_a [0: CU_NUMS-1][0: (BATCH_SIZE * BATCH_NUMS - 1)]; 

    integer cu, i, j, batch;
    string  base_dir = "C:/Users/lllolaa/Desktop/0422_data";

    // 时钟
    initial clk = 0;
    always #1.667 clk = ~clk; // ~300MHz

    // =========================================================================
    // 4. 任务定义
    // =========================================================================

    task init_signals();
        begin
            rst_n = 0; 
            host_we_act = 0; host_addr_act = 0; host_data_act = 0; start_broadcast = 0;
            ping_pong_sel = 0; dma_wr_en = 0; dma_wr_addr = 0; dma_wr_data_wide = 0;
            start_compute = 0; current_k_offset = 0;
            start_accum = 0;
            out_rd_en = 0; out_rd_addr = 0;
            #100; 
            rst_n = 1; 
            #50;
        end
    endtask

    // 加载并写入激活数据到广播 Buffer
    task config_and_broadcast_b();
        string b_path;
        begin
            $display("\n[%0t] [Task 1] Loading Vector B ...", $time);
            b_path = $sformatf("%s/b.txt", base_dir);
            $readmemh(b_path, mem_b);
            
            // 顺序写入广播 Buffer，内部会自动路由到各个CU的子内存
            for (i = 0; i < B_VEC_LEN; i = i + 1) begin
                @(negedge clk);
                host_we_act   = 1; 
                host_addr_act = i[B_ADDR_W-1:0]; 
                host_data_act = mem_b[i];
            end
            @(negedge clk); 
            host_we_act = 0;
            
            // 触发并行广播
            @(negedge clk);
            start_broadcast = 1;
            @(negedge clk);
            start_broadcast = 0;
            
            wait(broadcast_done == 1'b1);
            $display("[%0t] [Task 1] Parallel Broadcast Complete!", $time);
        end
    endtask

    // 加载权重 A 
    reg [A_PACKED_WIDTH-1:0] temp_mem [0: (BATCH_SIZE * BATCH_NUMS - 1)]; 
    task load_all_weights();
        string a_path;
        begin
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                a_path = $sformatf("%s/a/a_%04d.txt", base_dir, cu);
                $readmemh(a_path, temp_mem);
                for (j = 0; j < BATCH_SIZE * BATCH_NUMS; j = j + 1) begin
                    mem_a[cu][j] = temp_mem[j];
                end
            end
            $display("[%0t] [Task 2] All Weight Files Loaded.", $time);
        end
    endtask

    task run_gemv_and_accum();
        begin
            $display("\n[%0t] [Task 3] Starting Computation Flow...", $time);
            
            // --- 1. 执行矩阵乘法 ---
            for (batch = 0; batch < BATCH_NUMS; batch = batch + 1) begin
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
                ping_pong_sel = ~ping_pong_sel; 
                
                @(negedge clk);
                current_k_offset = batch;
                start_compute    = 1;
                @(negedge clk);
                start_compute    = 0;
                wait(all_batch_done == 1'b1);
            end
            wait(all_matrix_done == 1'b1);
            $display("[%0t] Matrix Multiply Done.", $time);

            // --- 2. 触发累加器 (将43个CU的结果求和) ---
            @(negedge clk);
            start_accum = 1;
            @(negedge clk);
            start_accum = 0;
            wait(accum_done == 1'b1);
            $display("[%0t] Accumulation Done.", $time);
        end
    endtask

    // 修改：读出最终的累加结果
    task read_and_print_results();
        begin
            $display("\n==================================================================");
            $display("                GEMV FINAL ACCUMULATED RESULTS                    ");
            $display("==================================================================");
            for (i = 0; i < M; i = i + 1) begin
                @(negedge clk);
                out_rd_en   = 1; 
                out_rd_addr = i;
                @(posedge clk); // 读延迟
                @(negedge clk); 
                $display("Row [%3d] : %d", i, out_data);
            end
            out_rd_en = 0;
            $display("==================================================================\n");
        end
    endtask

    // =========================================================================
    // 5. 主流程
    // =========================================================================
    initial begin
        init_signals();
        config_and_broadcast_b();
        load_all_weights();
        run_gemv_and_accum();
        read_and_print_results();
        $finish;
    end

    // =========================================================================
    // 6. DUT 实例化 (注意接口变化)
    // =========================================================================
    Compute_Core_Array #(
        .CU_NUMS(CU_NUMS), .M(M), .N(N), .T_WIDTH(T_WIDTH), 
        .B_VEC_LEN(B_VEC_LEN)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        
        .host_we_act(host_we_act),
        .host_addr_act(host_addr_act),
        .host_data_act(host_data_act),
        .start_broadcast(start_broadcast),
        .broadcast_done(broadcast_done),

        .ping_pong_sel(ping_pong_sel),
        .dma_wr_en(dma_wr_en),
        .dma_wr_addr(dma_wr_addr),
        .dma_wr_data_wide(dma_wr_data_wide),

        .start_compute(start_compute),
        .current_k_offset(current_k_offset),
        .all_batch_done(all_batch_done),
        .all_matrix_done(all_matrix_done),

        .start_accum(start_accum),
        .accum_done(accum_done),
        .out_rd_en(out_rd_en),
        .out_rd_addr(out_rd_addr),
        .out_data(out_data) 
    );

endmodule