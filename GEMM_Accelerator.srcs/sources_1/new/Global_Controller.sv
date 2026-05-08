`timescale 1ns / 1ps

module Global_Controller #(
      parameter N = 256,
      parameter T_WIDTH = 8,
      parameter BATCH_NUMS = N / T_WIDTH
)(
    input  wire                               clk,
    input  wire                               rst_n,

    input  wire                               start_system,
    input  wire                               host_uram_batch_ready, 
    output reg                                system_done,

    output reg                                start_dma,
    output wire [$clog2(BATCH_NUMS)-1:0]      dma_batch_idx,
    input  wire                               dma_done,

    output reg                                ping_pong_sel,
    output reg                                start_compute,
    output wire [$clog2(BATCH_NUMS)-1:0]      compute_batch_idx,
    input  wire                               all_batch_done,
    input  wire                               all_matrix_done,

    output reg                                start_accum,
    input  wire                               accum_done
);

    localparam [3:0] 
        IDLE            = 4'd0,
        // WAIT_CU_READY   = 4'd1, 
        PRELOAD_DMA     = 4'd2, // 只停留1拍 输出start_dma脉冲
        WAIT_PRE        = 4'd3, // 等待第一个batch进入URAM完成预加载
        // CHECK_URAM_SYNC = 4'd4, // 运行中检查URAM是否有下一个Batch
        SYNC_RUN        = 4'd5, // 只停留1拍 输出start_compute与start_dma脉冲
        WAIT_SYNC       = 4'd6,
        DRAIN           = 4'd7, 
        ACCUM           = 4'd8, // 只停留1拍 输出start_accum脉冲
        WAIT_ACCUM      = 4'd9,
        DONE            = 4'd10;// 只停留1拍 输出system_done脉冲

    reg [3:0] state, next_state;
    reg [$clog2(BATCH_NUMS):0] batch_cnt;  // 拓宽1位，能够容纳 BATCH_NUMS (32) 这个数值
    reg [$clog2(BATCH_NUMS):0] uram_ready_cnt; // 追踪Host写完了几个Batch
    // 用于独立捕获异步脉冲的 flags
    reg dma_ready_flag;
    reg cmp_ready_flag;

    assign dma_batch_idx     = batch_cnt[$clog2(BATCH_NUMS)-1:0]; 
    assign compute_batch_idx = batch_cnt[$clog2(BATCH_NUMS)-1:0] - 1'b1; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= IDLE;
        else        state <= next_state;
    end

    // 维护 uram_ready_cnt 的 always 块
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uram_ready_cnt <= 0;
        end else begin
            if (state == DONE) begin
                uram_ready_cnt <= 0; // 一轮矩阵计算结束，清零
            end else if (host_uram_batch_ready) begin
                uram_ready_cnt <= uram_ready_cnt + 1'b1; // Host写完一个Batch就+1
            end
        end
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE:        if (start_system) next_state = PRELOAD_DMA; 
            // WAIT_CU_READY:   if (uram_ready_cnt > 0) next_state = PRELOAD_DMA; // 等待第一个Batch就绪
            PRELOAD_DMA: next_state = WAIT_PRE;  
            // WAIT_PRE:    if (dma_done) next_state = SYNC_RUN;
            WAIT_PRE:    if (host_uram_batch_ready) next_state = SYNC_RUN;
            // CHECK_URAM_SYNC: if (batch_cnt == BATCH_NUMS) next_state = DRAIN; 
            //                 else if (uram_ready_cnt > batch_cnt) next_state = SYNC_RUN; 
            SYNC_RUN:    if (batch_cnt == BATCH_NUMS) next_state = DRAIN; 
                         else next_state = WAIT_SYNC;
            WAIT_SYNC:   begin
                            if(batch_cnt == 0) begin
                              if (dma_done) next_state = SYNC_RUN; // 第一个batch只等待DMA完成
                            end else begin
                              if ((dma_done || dma_ready_flag) && (all_batch_done || cmp_ready_flag)) next_state = SYNC_RUN;
                            end
                         end
            DRAIN:       if (all_matrix_done) next_state = ACCUM;
            ACCUM:       next_state = WAIT_ACCUM; 
            WAIT_ACCUM:  if (accum_done) next_state = DONE;
            DONE:        next_state = IDLE;       
            default:     next_state = IDLE;      
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_dma <= 0; start_compute <= 0; start_accum <= 0;
            ping_pong_sel <= 0; batch_cnt <= 0; system_done <= 0;
            dma_ready_flag <= 0; cmp_ready_flag <= 0;
        end else begin
            start_dma <= 0; start_compute <= 0; start_accum <= 0; system_done <= 0;

            case (state)
                IDLE: begin batch_cnt <= 0; ping_pong_sel <= 0; end
                // WAIT_CU_READY: if (cu_ready) // 改
                // PRELOAD_DMA: begin
                //     // start_dma <= 1;  // 第一个batch的DMA
                //     // batch_cnt <= 1;  
                // end
                SYNC_RUN: begin
                    if(batch_cnt == 0) start_compute <= 0; // 第一个batch还不能启动计算
                    else start_compute <= 1;  // 从第二个batch开始，计算和DMA同时启动
                    start_dma <= 1;           // 下一batch从URAM-BRAM
                    ping_pong_sel <= ~ping_pong_sel; 
                    dma_ready_flag <= 0; 
                    cmp_ready_flag <= 0;
                end
                WAIT_SYNC:  begin
                    if (batch_cnt == 0) begin
                        if (dma_done) batch_cnt <= batch_cnt + 1;
                    end else begin
                        if (dma_done)       dma_ready_flag <= 1;
                        if (all_batch_done) cmp_ready_flag <= 1;
                        if ((dma_done || dma_ready_flag) && (all_batch_done || cmp_ready_flag)) begin
                            batch_cnt <= batch_cnt + 1;
                            end
                    end
                end
                ACCUM:      start_accum <= 1; 
                DONE:       system_done <= 1;
            endcase
        end
    end
endmodule