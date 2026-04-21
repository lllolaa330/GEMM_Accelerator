`timescale 1ns / 1ps

module tb_PingPong_Weight_BRAM();

    parameter DATA_WIDTH = 18;
    parameter DEPTH      = 1024;
    
    reg clk;
    reg rst_n;
    reg ping_pong_sel;
    
    reg dma_wr_en;
    reg [$clog2(DEPTH)-1:0] dma_wr_addr;
    reg [DATA_WIDTH-1:0]    dma_wr_data;
    
    reg cu_rd_en;
    reg [$clog2(DEPTH)-1:0] cu_rd_addr;
    wire [DATA_WIDTH-1:0]   cu_rd_data;
    
    PingPong_Weight_BRAM #(
        .DATA_WIDTH(DATA_WIDTH), .DEPTH_PER_BANK(DEPTH)
    ) dut (.*);

    initial begin
        clk = 0; forever #5 clk = ~clk; 
    end

    integer i;

    initial begin
        rst_n = 0; ping_pong_sel = 0;
        dma_wr_en = 0; dma_wr_addr = 0; dma_wr_data = 0;
        cu_rd_en = 0; cu_rd_addr = 0;
        #20 rst_n = 1;
        
        // 1. sel=0 (DMA写Ping)
        $display("[%0t] Write to Ping Bank (Bank 0)", $time);
        @(posedge clk);
        for(i=0; i<10; i=i+1) begin
            dma_wr_en <= 1; dma_wr_addr <= i; dma_wr_data <= 18'h1A00 + i;
            @(posedge clk);
        end
        dma_wr_en <= 0;
        
        // 2. 切换 Ping-Pong (sel=1: CU读Ping, DMA写Pong)
        $display("[%0t] Swap Ping-Pong", $time);
        @(posedge clk);
        ping_pong_sel <= 1; 
        
        // 3. 同时进行: CU 读 Ping, DMA 写 Pong
        $display("[%0t] Simultaneous Read/Write Test", $time);
        @(posedge clk);
        for(i=0; i<10; i=i+1) begin
            // CU 读取刚刚写进去的 1A00 系列
            cu_rd_en <= 1; cu_rd_addr <= i;
            
            // DMA 写入 Pong (Bank 1) 2B00 系列
            dma_wr_en <= 1; dma_wr_addr <= i; dma_wr_data <= 18'h2B00 + i;
            
            @(posedge clk);
            if (i > 0) $display("Read Data = %h", cu_rd_data);
        end
        dma_wr_en <= 0; cu_rd_en <= 0;
        @(posedge clk);
        $display("Last Read Data = %h", cu_rd_data);
        
        #50 $finish;
    end
endmodule