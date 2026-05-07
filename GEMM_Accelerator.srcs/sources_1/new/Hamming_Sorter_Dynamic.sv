`timescale 1ns / 1ps

module Hamming_Sorter_Dynamic #(
    parameter T_WIDTH = 8,
    parameter DATA_WIDTH = 18,
    parameter BATCH_SIZE = 1024 
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   in_batch_start,
    input  wire                   in_valid,
    input  wire [DATA_WIDTH-1:0]  in_data,
    input  wire                   out_ready,   // 可以进入 DISPTCH 的标志
    
    output reg                    in_ready,    // IDLE 状态
    output wire                   out_valid,
    output wire [DATA_WIDTH-1:0]  out_data,

    output reg                    batch_done
);
        
    // 1. 输入寄存处理
    // 根据 T_WIDTH 自动计算权值寄存器需要的位宽
    // 例如 T_WIDTH=8 时，权重范围0~8，需要4bit；T_WIDTH=4时，权重范围0~4，需要3bit
    localparam WEIGHT_W = $clog2(T_WIDTH + 1);  
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

    always @(posedge clk) begin
        if (!rst_n) begin
            in_valid_reg <= 0;
        end else begin
            if (in_valid) begin
                in_data_reg  <= in_data;
                weight_reg   <= get_popcount(in_data[DATA_WIDTH-1:DATA_WIDTH-T_WIDTH]);
                in_valid_reg <= 1'b1;
            end else begin
                in_valid_reg <= 1'b0;
            end
        end
    end

    // 2. 状态机
    localparam IDLE      = 3'd0;
    localparam LOAD      = 3'd1;
    localparam WAIT      = 3'd2; 
    localparam DISPATCH  = 3'd3;
    localparam DONE      = 3'd4;

    reg [2:0]  state;
    reg [3:0]  read_ptr;
    
    // 位宽必须足够容纳 BATCH_SIZE ($clog2(1024) = 10)
    reg [$clog2(BATCH_SIZE):0]  load_cnt; 
    reg [1:0]                   gap_cnt;

    // 3. FIFO 实例化
    wire [DATA_WIDTH-1:0] fifo_dout[0:T_WIDTH];
    wire [T_WIDTH:0] fifo_empty;
    wire[T_WIDTH:0] fifo_re_mask;

    genvar i;
    generate
        for (i = 0; i <= T_WIDTH; i = i + 1) begin : gen_buckets
            simple_fifo #(.DEPTH(BATCH_SIZE), .WIDTH(DATA_WIDTH)) bucket (
                .clk(clk),
                .rst(!rst_n || batch_done), // 只有在每组结束后的 GAP 期才复位
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
    assign fifo_re_mask = (out_valid ) ? (1 << read_ptr) : 0;

    // 5. 状态机逻辑
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= IDLE;
            in_ready   <= 1'b1;
            load_cnt   <= 1'b0;
            gap_cnt    <= 1'b0;
            read_ptr   <= 1'b0;
            batch_done <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    in_ready     <= 1'b1;
                    batch_done   <= 1'b0;
                    if (in_batch_start) begin
                        state    <= LOAD;
                        load_cnt <= 1'b0;
                    end
                end

                LOAD: begin
                    in_ready <= 1'b0;
                    if (in_valid) begin
                        if (load_cnt == BATCH_SIZE - 1) begin
                            state    <= WAIT;
                            load_cnt <= 1'b0;
                        end else begin
                            load_cnt <= load_cnt + 1'b1;
                        end
                    end
                end
                
                WAIT: begin
                    // 只有 si 生成结束，sorter 才可以开始输出 Dataflow
                    if (out_ready) begin
                        state    <= DISPATCH;
                        read_ptr <= 0;
                    end 
                end

                DISPATCH: begin
                    // 只有当桶真的空了，且 read_ptr 可以指向下一位时才跳转
                    if (read_ptr > T_WIDTH) begin
                        state    <= DONE;
                        gap_cnt  <= 1'b0;
                    end else if (fifo_empty[read_ptr]) begin
                        read_ptr <= read_ptr + 1'b1;
                    end
                end

                DONE: begin
                    // Batch Dataflow 派发结束
                    batch_done <= 1'b1;
                    if (gap_cnt == 2'b11) begin
                        state      <= IDLE;
                        batch_done <= 1'b0;
                        gap_cnt    <= 2'b00;
                    end else begin
                        gap_cnt <= gap_cnt + 1'b1;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end
endmodule