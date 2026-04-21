`timescale 1ns / 1ps

module tb_CU_Wrapper();

    parameter M = 16;
    parameter N = 16;
    parameter T_WIDTH = 8;
    parameter BATCH_SIZE = 64; // 为加快仿真，使用较小参数
    parameter BATCH_NUMS = 2;
    parameter OUT_WIDTH = 32;
    parameter A_PACKED_WIDTH = 18;

    reg clk;
    reg rst_n;
    reg ping_pong_sel;
    
    reg dma_wr_en;
    reg [$clog2(BATCH_SIZE)-1:0] dma_wr_addr;
    reg [A_PACKED_WIDTH-1:0]     dma_wr_data;
    
    reg start_compute;
    reg [$clog2(BATCH_NUMS)-1:0] current_k_offset;
    
    reg cfg_we_b;
    reg [7:0] cfg_addr_b;
    reg [7:0] cfg_data_b;
    
    reg out_rd_en;
    reg [$clog2(M)-1:0] out_rd_addr;
    wire [OUT_WIDTH-1:0] out_data;
    wire batch_done;
    wire matrix_done;

    CU_Wrapper #(
        .M(M), .N(N), .T_WIDTH(T_WIDTH),
        .BATCH_SIZE(BATCH_SIZE), .BATCH_NUMS(BATCH_NUMS)
    ) dut (.*);

    initial begin
        clk = 0; forever #5 clk = ~clk; 
    end

    integer i;

    initial begin
        rst_n = 0;
        ping_pong_sel = 0;
        dma_wr_en = 0; dma_wr_addr = 0; dma_wr_data = 0;
        start_compute = 0; current_k_offset = 0;
        cfg_we_b = 0; cfg_addr_b = 0; cfg_data_b = 0;
        out_rd_en = 0; out_rd_addr = 0;
        
        #20 rst_n = 1;
        
        // 1. 配置 Vector B
        $display("[%0t] Configuring Vector B...", $time);
        @(posedge clk);
        for(i=0; i<N; i=i+1) begin
            cfg_we_b <= 1; cfg_addr_b <= i; cfg_data_b <= $random;
            @(posedge clk);
        end
        cfg_we_b <= 0;
        
        // 2. DMA 搬运 Batch 0 的权重到 Ping Bank
        $display("[%0t] DMA Writing Batch 0 to Ping...", $time);
        @(posedge clk);
        ping_pong_sel <= 0; // DMA写Ping
        for(i=0; i<BATCH_SIZE; i=i+1) begin
            dma_wr_en <= 1; dma_wr_addr <= i; dma_wr_data <= i; // 用序号做假数据
            @(posedge clk);
        end
        dma_wr_en <= 0;
        
        // 3. 翻转 Ping-Pong，命令 CU 开始计算，同时模拟 DMA 继续写下一个
        $display("[%0t] Swap Ping-Pong. Start Compute Batch 0!", $time);
        @(posedge clk);
        ping_pong_sel <= 1; // 翻转：CU读Ping，DMA写Pong
        current_k_offset <= 0;
        start_compute <= 1; 
        @(posedge clk);
        start_compute <= 0; // 单脉冲
        
        // 此时可在这个时段内让 DMA 继续写 BATCH_SIZE 数据，由于篇幅省去。
        
        // 4. 等待计算完成
        $display("[%0t] Waiting for batch_done...", $time);
        wait(batch_done == 1'b1);
        $display("[%0t] SUCCESS: batch Done received!", $time);
        
        // 5. 读出几个结果看看
        @(posedge clk);
        out_rd_en <= 1; out_rd_addr <= 0;
        @(posedge clk);
        out_rd_addr <= 1;
        @(posedge clk);
        $display("Result 0 = %d", out_data);
        out_rd_en <= 0;
        
        #100 $finish;
    end
endmodule