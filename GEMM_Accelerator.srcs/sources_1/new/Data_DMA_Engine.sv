`timescale 1ns / 1ps

module Data_DMA_Engine #(
    parameter M = 256,
    parameter N = 256,
    parameter T_WIDTH = 8,
    parameter RAM_LATENCY = 2,      // URAM 读出延迟
    parameter PIPE_STAGES = 2,      // 流水线延迟
    parameter BATCH_SIZE = M *4,
    parameter BATCH_NUMS = N / T_WIDTH,
    parameter DEPTH      = BATCH_SIZE * BATCH_NUMS   // URAM 存放 CU 所需的完整权重
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // --- 控制侧接口 ---
    input  wire                               start_dma,
    input  wire [$clog2(BATCH_NUMS)-1:0]      batch_idx,
    output wire                               dma_done,

    // --- URAM 读侧接口 ---
    output wire                               uram_rd_en,
    output wire [$clog2(DEPTH)-1:0]           uram_rd_addr,  // 所有batch

    // --- BRAM 写侧同步控制接口 ---
    output wire                               bram_wr_en,
    output wire [$clog2(BATCH_SIZE)-1:0]      bram_wr_addr   // 1个batch
);

    // =========================================================================
    // 参数与内部信号声明
    // =========================================================================
    localparam TOTAL_LATENCY = RAM_LATENCY + PIPE_STAGES;
    localparam BRAM_ADDR_W   = $clog2(BATCH_SIZE);
    localparam URAM_ADDR_W   = $clog2(DEPTH);

    reg                   active;
    reg [BRAM_ADDR_W-1:0] local_cnt;
    reg [URAM_ADDR_W-1:0] base_addr;

    // 延迟对齐移位寄存器
    reg [TOTAL_LATENCY-1:0]   en_shift;
    reg [BRAM_ADDR_W-1:0]     addr_shift [0:TOTAL_LATENCY-1];

    integer i;

    // =========================================================================
    // 逻辑 1: 地址发生器 (AGU) 
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active    <= 1'b0;
            local_cnt <= 0;
            base_addr <= 0;
        end else begin
            if (start_dma) begin
                active    <= 1'b1;
                local_cnt <= 0;
                base_addr <= batch_idx * BATCH_SIZE; // 计算起始地址
            end else if (active) begin
                if (local_cnt == BATCH_SIZE - 1) begin
                    active <= 1'b0; // 发送完一个 Batch 后停止
                end else begin
                    local_cnt <= local_cnt + 1'b1;
                end
            end
        end
    end

    assign uram_rd_en   = active;
    assign uram_rd_addr = base_addr + local_cnt;

    // =========================================================================
    // 逻辑 2: 时延匹配流水线 (Latency Matcher)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en_shift <= 0;
            for(i=0; i<TOTAL_LATENCY; i=i+1) addr_shift[i] <= 0;
        end else begin
            // 数据移入打拍器
            en_shift[0]    <= active;
            addr_shift[0]  <= local_cnt;
            
            // 移位
            for(i=1; i<TOTAL_LATENCY; i=i+1) begin
                en_shift[i]   <= en_shift[i-1];
                addr_shift[i] <= addr_shift[i-1];
            end
        end
    end

    // 经过延迟后，使能和地址完美与数据对齐
    assign bram_wr_en   = en_shift[TOTAL_LATENCY-1];
    assign bram_wr_addr = addr_shift[TOTAL_LATENCY-1];
    
    // DMA 全部写入完成信号 (下降沿提取)
    assign dma_done = (en_shift[TOTAL_LATENCY-1] == 1'b1) && (en_shift[TOTAL_LATENCY-2] == 1'b0);

endmodule