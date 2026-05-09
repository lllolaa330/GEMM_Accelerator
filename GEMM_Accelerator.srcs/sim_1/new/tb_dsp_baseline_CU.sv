`timescale 1ns / 1ps

module tb_dsp_baseline_CU();

    // =========================================================================
    // 参数配置
    // =========================================================================
    parameter M              = 256;
    parameter N              = 256;
    parameter T_WIDTH        = 8;
    parameter A_PACKED_WIDTH = 10 + T_WIDTH; // 18
    parameter B_DATA_BITS    = 8;
    parameter OUT_WIDTH      = 32;
    parameter OUT_ADDR_W     = 8;            // $clog2(256)
    parameter B_VEC_LEN      = N;            // 256
    localparam BATCH_NUMS     = N / T_WIDTH;            // 批次数量
    localparam BATCH_SIZE     = M * 4;                  // 每个批次的大小
    parameter URAM_DEPTH     = BATCH_SIZE * BATCH_NUMS;            // 256
    parameter WEIGHT_TOTAL_ROW = M;          // 256
    string base_dir = "C:/Users/lllolaa/Desktop/data";

    // =========================================================================
    // 信号声明
    // =========================================================================
    reg                               clk;
    reg                               rst_n;

    // Interface B
    reg                               cfg_we_b;
    reg  [7:0]                        cfg_addr_b;
    reg  signed [B_DATA_BITS-1:0]     cfg_data_b;

    // Interface A (Runtime Data)
    reg                               a_valid;
    reg  [A_PACKED_WIDTH-1:0]         a_payload;
    reg  [7:0]                        k_offset;
    wire                              cu_ready;
    wire                              batch_done;
    wire                              matrix_done;

    // Output Interface
    reg                               out_rd_en;
    reg  [OUT_ADDR_W-1:0]             out_rd_addr;
    wire signed [OUT_WIDTH-1:0]       out_data;

    // =========================================================================
    // 实例化 DUT
    // =========================================================================
    dsp_baseline_CU #(
        .M(M),
        .N(N),
        .T_WIDTH(T_WIDTH),
        .A_PACKED_WIDTH(A_PACKED_WIDTH),
        .B_DATA_BITS(B_DATA_BITS),
        .OUT_WIDTH(OUT_WIDTH)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        
        .cfg_we_b       (cfg_we_b),
        .cfg_addr_b     (cfg_addr_b),
        .cfg_data_b     (cfg_data_b),
        
        .a_valid        (a_valid),
        .a_payload      (a_payload),
        .k_offset       (k_offset),
        .cu_ready       (cu_ready),
        .batch_done     (batch_done),
        .matrix_done    (matrix_done),
        
        .out_rd_en      (out_rd_en),
        .out_rd_addr    (out_rd_addr),
        .out_data       (out_data)
    );

    // =========================================================================
    // 时钟生成 (100MHz)
    // =========================================================================
    initial clk = 0;
    always #5 clk = ~clk;

    // =========================================================================
    // 内存与文件句柄定义
    // =========================================================================
    reg [OUT_WIDTH-1:0] golden_mem [0:M-1]; // 存储期望结果
    
    string file_b, file_a, file_golden;
    integer status;
    integer i,j,idx;
    integer err_count;
    
    // 存储
    reg [7:0]                         b_mem [0: B_VEC_LEN-1];
    reg [A_PACKED_WIDTH-1:0]          a_mem [0: URAM_DEPTH-1]; 
    reg [31:0]                        golden_mem    [0: WEIGHT_TOTAL_ROW-1];

    // =========================================================================
    // 主测试流程
    // =========================================================================
    initial begin
        // 1. 初始化信号
        rst_n       = 0;
        cfg_we_b    = 0;
        cfg_addr_b  = 0;
        cfg_data_b  = 0;
        a_valid     = 0;
        a_payload   = 0;
        k_offset    = 0;
        out_rd_en   = 0;
        out_rd_addr = 0;
        err_count   = 0;

        // 提前把 golden.txt 读进内存
        file_golden = $sformatf("%s/golden/golden_0001.txt", base_dir);
        $readmemh(file_golden, golden_mem);
        $display("[%0t] Golden values loaded from golden.txt", $time);

        // 2. 复位释放
        #100;
        @(negedge clk);
        rst_n = 1;
        $display("[%0t] Reset De-asserted.", $time);

        // 3. 读取 b.txt 配置 B 向量 (LUTRAM)
        file_b = $sformatf("%s/b.txt", base_dir);
        $readmemh(file_b, b_mem);
        $display("[%0t] Vector B values loaded from b.txt", $time);
        for (i = 0; i < N; i = i + 1) begin
            @(negedge clk); // 在下降沿驱动，避免竞争冒险
            cfg_we_b   = 1;
            cfg_addr_b = i;
            cfg_data_b = b_mem[i];
        end
        @(negedge clk);
        cfg_we_b = 0;
        $display("[%0t] Finished loading Vector B.", $time);

        // 4. 读取 a.txt 产生 A 流水线激励
        file_a = $sformatf("%s/T8/a_0001.txt", base_dir);
        $readmemh(file_a, a_mem);
        $display("[%0t] Starting Data Stream A...", $time);
        for (j=0; j < BATCH_NUMS; j = j + 1) begin
            $display("[%0t] Starting Batch %0d / %0d", $time, j+1, BATCH_NUMS);
            k_offset = j[7:0]; // 每个批次从 k_offset=0 开始
            for (i = 0; i < BATCH_SIZE; i = i + 1) begin
                idx = j * BATCH_SIZE + i;
                @(negedge clk);
                a_valid   = 1;
                a_payload = a_mem[idx][A_PACKED_WIDTH-1:0];
            end
            #10; // 最后一个数据后保持 a_valid 一段时间，确保数据被正确采样
            a_valid = 0; // 每个批次结束后拉低 a_valid，等待 batch_done 信号
            $display("[%0t] Finished sending Batch %0d. Waiting for batch_done...", $time, j+1);
            wait(batch_done);
          end
        @(negedge clk);
        a_valid = 0;

        // 5. 等待运算完成信号 (matrix_done)
        $display("[%0t] Wait for Matrix Done...", $time);
        while (!matrix_done) begin
            @(posedge clk);
        end
        $display("[%0t] matrix_done detected! Waiting pipeline flush...", $time);
        
        // 等待几拍，保证最后一次 APE 结果稳妥写入寄存器
        repeat(5) @(posedge clk); 

        // 6. 从 DUT 读取结果并与 Golden 对比
        $display("[%0t] Start verifying results...", $time);
        
        out_rd_en = 1;
        // 循环 M+1 次，因为同步读取在给地址的下一拍才出数据
        for (i = 0; i <= M; i = i + 1) begin
            @(negedge clk);
            if (i < M) begin
                out_rd_addr = i;
            end else begin
                out_rd_en = 0;
            end
            
            // 数据在给出地址的下一个周期生效，因此验证上一周期指定的地址
            if (i > 0) begin
                if (out_data !== golden_mem[i-1]) begin
                    $display("ERROR @ Row %3d: Expected = %h, Got = %h", 
                             i-1, golden_mem[i-1], out_data);
                    err_count = err_count + 1;
                end
            end
        end

        // 7. 打印最终测试结果
        $display("---------------------------------------------------");
        if (err_count == 0) begin
            $display("TEST PASSED! All %0d outputs matched.", M);
        end else begin
            $display("TEST FAILED! Total errors: %0d / %0d", err_count, M);
        end
        $display("---------------------------------------------------");

        $finish;
    end

endmodule