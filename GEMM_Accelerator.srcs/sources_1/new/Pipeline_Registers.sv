`timescale 1ns / 1ps

module Pipeline_Registers #(
    parameter N = 256,
    parameter B_VEC_LEN = 4096,
    parameter T_WIDTH = 8,
    parameter A_PACKED_WIDTH = 10 + T_WIDTH,
    parameter CU_NUMS    = B_VEC_LEN / N,
    parameter DATA_WIDTH = A_PACKED_WIDTH * CU_NUMS, // 所有CU的权重展平，宽总线 
    parameter STAGES = 2     // 打拍级数
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DATA_WIDTH-1:0]  data_in,
    output wire [DATA_WIDTH-1:0]  data_out
);

    // =========================================================================
    // 内部信号声明
    // =========================================================================
    reg [DATA_WIDTH-1:0] pipe_regs [0:STAGES-1];
    integer i;

    // =========================================================================
    // 逻辑实现
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < STAGES; i = i + 1) begin
                pipe_regs[i] <= {DATA_WIDTH{1'b0}};
            end
        end else begin
            pipe_regs[0] <= data_in;
            for (i = 1; i < STAGES; i = i + 1) begin
                pipe_regs[i] <= pipe_regs[i-1];
            end
        end
    end

    assign data_out = pipe_regs[STAGES-1];

endmodule