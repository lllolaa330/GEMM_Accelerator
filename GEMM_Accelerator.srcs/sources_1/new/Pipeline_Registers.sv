`timescale 1ns / 1ps

module Pipeline_Registers #(
    parameter WIDTH  = 774,
    parameter STAGES = 2     // 打拍级数
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [WIDTH-1:0]  data_in,
    output wire [WIDTH-1:0]  data_out
);

    // =========================================================================
    // 内部信号声明
    // =========================================================================
    reg [WIDTH-1:0] pipe_regs [0:STAGES-1];
    integer i;

    // =========================================================================
    // 逻辑实现
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < STAGES; i = i + 1) begin
                pipe_regs[i] <= {WIDTH{1'b0}};
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