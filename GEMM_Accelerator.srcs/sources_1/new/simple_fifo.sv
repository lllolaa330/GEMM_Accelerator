`timescale 1ns / 1ps
// 同步FIFO，用于分布式RAM实现
module simple_fifo #(
    parameter DEPTH = 256,
    parameter WIDTH = 18
)(
    input  wire             clk,
    input  wire             rst,
    input  wire [WIDTH-1:0] din,
    input  wire             we,
    output wire [WIDTH-1:0] dout,
    input  wire             re,
    output wire             empty,
    output wire             full
);
    localparam ADDR_W = $clog2(DEPTH);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_W:0]  wptr, rptr;

    always @(posedge clk) begin
        if (rst) begin
            wptr <= 0;
            rptr <= 0; 
        end else begin
            if (we && !full) wptr <= wptr + 1;
            if (re && !empty) rptr <= rptr + 1;
        end
    end

    always @(posedge clk) begin
        if (we && !full) mem[wptr[ADDR_W-1:0]] <= din;
    end

    assign dout  = mem[rptr[ADDR_W-1:0]];
    assign full  = (wptr[ADDR_W] != rptr[ADDR_W]) && (wptr[ADDR_W-1:0] == rptr[ADDR_W-1:0]);
    assign empty = (wptr == rptr);
endmodule