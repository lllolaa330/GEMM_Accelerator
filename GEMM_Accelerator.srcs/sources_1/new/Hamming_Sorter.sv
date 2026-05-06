`timescale 1ns / 1ps

module Hamming_Sorter #(
    parameter T_WIDTH = 8,
    parameter DATA_WIDTH = 18,
    parameter BATCH_SIZE = 1024 
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   in_valid,
    input  wire [DATA_WIDTH-1:0]  in_data,
    output wire                   in_ready,    // 当前桶未满，告知上游何时可发数据

    output wire                   out_valid,
    output wire [DATA_WIDTH-1:0]  out_data,
    input  wire                   out_ready,

    output wire                   soft_rst_batch_done,
    output wire                   accepting_first_data
);

    localparam BATCH_ADDR = $clog2(BATCH_SIZE);
    localparam WEIGHT_W = $clog2(T_WIDTH + 1);          
    // 根据 T_WIDTH 自动计算权值寄存器需要的位宽
    // 例如 T_WIDTH=8 时，权重范围0~8，需要4bit；T_WIDTH=4时，权重范围0~4，需要3bit

    // 1. 输入寄存处理
    reg [DATA_WIDTH-1:0] in_data_reg;
    reg [WEIGHT_W-1:0]   weight_reg;
    reg                  in_valid_reg;

    function[WEIGHT_W-1:0] get_popcount(input [T_WIDTH-1:0] node);
        integer j;
        begin
            get_popcount = 0;
            for (j = 0; j < T_WIDTH; j = j + 1) begin
                get_popcount = get_popcount + node[j]; // 累加每个 bit
            end
        end
    endfunction

    // 只有在 COLLECT 状态才允许接收新数据
    assign in_ready = (state == COLLECT) && (count < BATCH_SIZE);
    assign accepting_first_data = (state == COLLECT) && (count == 1'b1);

    always @(posedge clk) begin
        if (!rst_n) begin
            in_valid_reg <= 0;
        end else begin
            // 只有当接收到有效输入且处于收集状态时才处理
            if (in_valid && in_ready) begin
                in_data_reg  <= in_data;
                weight_reg   <= get_popcount(in_data[DATA_WIDTH-1:DATA_WIDTH-T_WIDTH]);
                in_valid_reg <= 1'b1;
            end else begin
                in_valid_reg <= 1'b0;
            end
        end
    end

    // 2. 状态机
    localparam COLLECT   = 2'd0;
    localparam WAIT_PIPE = 2'd1; 
    localparam DISPATCH  = 2'd2;
    localparam DONE_GAP  = 2'd3;

    reg [1:0]  state;
    reg [3:0]  read_ptr;
    
    // 位宽必须足够容纳 BATCH_SIZE ($clog2(1024) = 10)
    reg [$clog2(BATCH_SIZE):0]  count; 
    
    reg [3:0]  gap_cnt;
    reg [7:0]  batch_cnt;

    reg batch_done_sig;

    // 3. FIFO 实例化
    wire [DATA_WIDTH-1:0] fifo_dout[0:T_WIDTH];
    wire [T_WIDTH:0] fifo_empty;
    wire[T_WIDTH:0] fifo_re_mask;

    genvar i;
    generate
        for (i = 0; i <= T_WIDTH; i = i + 1) begin : gen_buckets
            simple_fifo #(.DEPTH(BATCH_SIZE), .WIDTH(DATA_WIDTH)) bucket (
                .clk(clk),
                .rst(!rst_n || batch_done_sig), // 只有在每组结束后的 GAP 期才复位
                .din(in_data_reg),
                .we(in_valid_reg && (weight_reg == i)),
                .dout(fifo_dout[i]),
                .re(fifo_re_mask[i]),
                .empty(fifo_empty[i]),
                .full()
            );
        end
    endgenerate

    // 4. 输出逻辑
    wire current_fifo_valid = (state == DISPATCH) && (read_ptr <= T_WIDTH) && !fifo_empty[read_ptr];
    assign out_valid = current_fifo_valid;
    assign out_data  = current_fifo_valid ? fifo_dout[read_ptr] : {DATA_WIDTH{1'b0}};
    // 握手逻辑：只有下游 ready 且当前有数据，才真正读 FIFO
    assign fifo_re_mask = (out_valid && out_ready) ? (1 << read_ptr) : 0;

    assign soft_rst_batch_done = batch_done_sig;

    // 5. 状态机逻辑
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= COLLECT;
            count <= 0;
            read_ptr <= 0;
            batch_done_sig <= 0;
            batch_cnt <= 0;
            gap_cnt <= 0;
        end else begin
            case (state)
                COLLECT: begin
                    batch_done_sig <= 0;
                    if (in_valid && in_ready) begin
                        if (count == BATCH_SIZE - 1) begin
                            state <= WAIT_PIPE;
                            count <= 0;
                        end else begin
                            count <= count + 1;
                        end
                    end
                end
                
                WAIT_PIPE: begin
                    // 容错：确保流水线最后一拍数据安全落入 FIFO 后再开始派发
                    if (gap_cnt == 1) begin
                        state <= DISPATCH;
                        read_ptr <= 0;
                        gap_cnt <= 0;
                    end else begin
                        gap_cnt <= gap_cnt + 1;
                    end
                end

                DISPATCH: begin
                    // 只有当桶真的空了，且 read_ptr 可以指向下一位时才跳转
                    if (read_ptr > T_WIDTH) begin
                        state <= DONE_GAP;
                        gap_cnt <= 0;
                    end else if (fifo_empty[read_ptr]) begin
                        read_ptr <= read_ptr + 1;
                    end
                end

                DONE_GAP: begin
                    // 发出复位信号，通知下游这一组结束了
                    batch_done_sig <= 1'b1;
                    
                    // 留出间隙，确保 APE 接收到复位并完成清空
                    if (gap_cnt == 1) begin
                        state <= COLLECT;
                        batch_done_sig <= 0;
                        gap_cnt <= 0;
                        
                    end else begin
                        gap_cnt <= gap_cnt + 1;
                    end
                end
            endcase
        end
    end
endmodule