`timescale 1ns / 1ps

module Global_Controller #(
      parameter N = 256,
      parameter T_WIDTH = 8,
      parameter BATCH_NUMS = N / T_WIDTH
)(
    input  wire                               clk,
    input  wire                               rst_n,

    input  wire                               start_system,
    output reg                                system_done,

    output reg                                start_dma,
    output wire [$clog2(BATCH_NUMS)-1:0]      dma_batch_idx,
    input  wire                               dma_done,

    output reg                                ping_pong_sel,
    output reg                                start_compute,
    output wire [$clog2(BATCH_NUMS)-1:0]      compute_batch_idx,
    input  wire                               all_batch_done,
    input  wire                               all_matrix_done,

    // --- 新增：累加器控制 ---
    output reg                                start_accum,
    input  wire                               accum_done
);

    localparam [3:0] 
        IDLE        = 4'd0,
        PRELOAD_DMA = 4'd1, 
        WAIT_PRE    = 4'd2,
        SYNC_RUN    = 4'd3, 
        WAIT_SYNC   = 4'd4,
        DRAIN       = 4'd5, 
        ACCUM       = 4'd6, 
        WAIT_ACCUM  = 4'd7,
        DONE        = 4'd8;

    reg [3:0] state, next_state;
    reg [$clog2(BATCH_NUMS)-1:0] batch_cnt;

    assign dma_batch_idx     = batch_cnt; 
    assign compute_batch_idx = batch_cnt - 1'b1; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= IDLE;
        else        state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE:        if (start_system) next_state = PRELOAD_DMA;
            PRELOAD_DMA: next_state = WAIT_PRE;
            WAIT_PRE:    if (dma_done) next_state = SYNC_RUN;
            SYNC_RUN:    if (batch_cnt == BATCH_NUMS) next_state = DRAIN; 
                         else next_state = WAIT_SYNC;
            WAIT_SYNC:   if (dma_done && all_batch_done) next_state = SYNC_RUN;
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
        end else begin
            start_dma <= 0; start_compute <= 0; start_accum <= 0; system_done <= 0;

            case (state)
                IDLE: begin batch_cnt <= 0; ping_pong_sel <= 0; end
                PRELOAD_DMA: start_dma <= 1; 
                WAIT_PRE: if (dma_done) batch_cnt <= batch_cnt + 1; 
                SYNC_RUN: begin
                    if (batch_cnt < BATCH_NUMS) start_dma <= 1; 
                    start_compute <= 1; 
                    ping_pong_sel <= ~ping_pong_sel; 
                end
                WAIT_SYNC:  if (dma_done && all_batch_done) batch_cnt <= batch_cnt + 1;
                ACCUM:      start_accum <= 1; 
                DONE:       system_done <= 1;
            endcase
        end
    end
endmodule