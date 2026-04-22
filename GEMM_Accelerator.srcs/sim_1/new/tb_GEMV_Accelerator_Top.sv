`timescale 1ns / 1ps

module tb_GEMV_Accelerator_Top;

    // =========================================================================
    // 1. 参数与配置
    // =========================================================================
    parameter B_VEC_LEN      = 4096; 
    parameter M              = 256;
    parameter N              = 256;
    parameter T_WIDTH        = 8;                       
    parameter OUT_WIDTH      = 32;                      
    
    localparam A_PACKED_WIDTH = 18;
    localparam BATCH_NUMS     = N / T_WIDTH;
    localparam BATCH_SIZE     = M * 4;
    localparam WIDE_BUS       = CU_NUMS * A_PACKED_WIDTH;
    localparam URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;
    localparam OUT_WIDE       = CU_NUMS * OUT_WIDTH;

    // =========================================================================
    // 2. 信号定义
    // =========================================================================
    reg clk, rst_n;
    
    reg                   host_start;
    wire                  system_done;

    reg                   host_we_uram;
    reg [$clog2(URAM_DEPTH)-1:0] host_addr_uram;
    reg [WIDE_BUS-1:0]    host_data_uram;

    reg                   host_we_act;
    reg [7:0]             host_addr_act;
    reg [7:0]             host_data_act;
    reg                   host_start_broadcast;
    wire                  broadcast_done;

    reg                   out_rd_en;
    reg [$clog2(M)-1:0]   out_rd_addr;
    wire [OUT_WIDE-1:0]   out_data_wide;

    // 测试数据存储
    reg [7:0]             mem_b [0: N-1]; 
    reg [A_PACKED_WIDTH-1:0] temp_mem [0: URAM_DEPTH-1]; // 一维缓存
    reg [A_PACKED_WIDTH-1:0] mem_a [0: CU_NUMS-1][0: URAM_DEPTH-1]; 

    integer cu, i;
    string  base_dir = "C:/Users/lllolaa/Desktop/0422_data";

    // =========================================================================
    // 3. 时钟逻辑
    // =========================================================================
    initial clk = 0;
    always #1.666 clk = ~clk; // ~300MHz

    // =========================================================================
    // 4. 自动化测试流程
    // =========================================================================
    initial begin
        // --- 初始化 ---
        rst_n = 0; host_start = 0; host_we_uram = 0; host_we_act = 0; host_start_broadcast = 0; out_rd_en = 0;
        #20 rst_n = 1; #10;

        // --- Step 1: 配置 Vector B 广播 ---
        $display("[%0t] Step 1: Loading & Broadcasting Vector B...", $time);
        $readmemh($sformatf("%s/b.txt", base_dir), mem_b);
        for (i = 0; i < N; i = i + 1) begin
            @(negedge clk);
            host_we_act = 1; host_addr_act = i; host_data_act = mem_b[i];
        end
        @(negedge clk); host_we_act = 0; host_start_broadcast = 1;
        @(negedge clk); host_start_broadcast = 0;
        wait(broadcast_done);
        $display("[%0t] Vector B Broadcast Done.", $time);

        // --- Step 2: 加载文件并拼接写入 URAM ---
        $display("[%0t] Step 2: Loading Weights into L2 URAM...", $time);
        for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
            $readmemh($sformatf("%s/a/up_proj/a_%04d.txt", base_dir, cu + 1), temp_mem);
            for (i = 0; i < URAM_DEPTH; i = i + 1) mem_a[cu][i] = temp_mem[i];
        end
        
        for (i = 0; i < URAM_DEPTH; i = i + 1) begin
            @(negedge clk);
            host_we_uram   = 1;
            host_addr_uram = i;
            // 组装宽总线
            for (cu = 0; cu < CU_NUMS; cu = cu + 1) begin
                host_data_uram[cu*A_PACKED_WIDTH +: A_PACKED_WIDTH] = mem_a[cu][i];
            end
        end
        @(negedge clk); host_we_uram = 0;
        $display("[%0t] URAM Configured.", $time);

        // --- Step 3: 一键启动自动数据流计算 ---
        $display("[%0t] Step 3: Trigger System Start!", $time);
        @(negedge clk); host_start = 1;
        @(negedge clk); host_start = 0;

        wait(system_done);
        $display("[%0t] Step 3: SYSTEM DONE!", $time);

    // --- Step 4: 结果打印 ---
    $display("======= FINAL OUTPUTS =======");
    for (i = 0; i < M; i = i + 1) begin
        @(negedge clk); out_rd_en = 1; out_rd_addr = i;
        @(posedge clk); @(negedge clk); 
        // 外部看起来，这个巨大的系统就像一个普通的单口 RAM 一样干净！
        $display("Row [%0d] Result: %0d", i, $signed(out_data_wide)); 
    end
    out_rd_en = 0;
        $finish;
    end

    // =========================================================================
    // 5. DUT
    // =========================================================================
    GEMV_Accelerator_Top #(
        .CU_NUMS(CU_NUMS), .M(M), .N(N), .T_WIDTH(T_WIDTH)
    ) dut (.*);

endmodule