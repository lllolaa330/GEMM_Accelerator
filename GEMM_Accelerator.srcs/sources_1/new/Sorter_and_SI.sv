`timescale 1ns / 1ps

module Sorter_and_SI #(
    parameter T_WIDTH = 8,
    parameter DATA_WIDTH = 10 + T_WIDTH,
    parameter PSUM_WIDTH = 24,
    parameter BATCH_SIZE = 1024,
    parameter BATCH_NUMS = 256 / T_WIDTH,
    parameter OFFSET_ADDR_WIDTH = $clog2(BATCH_NUMS),
    parameter SI_SIZE = 2 ** T_WIDTH
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          in_valid,
    input  wire [DATA_WIDTH-1:0]         in_data,
    input  wire [OFFSET_ADDR_WIDTH-1:0]  in_k_offset,

    output wire                         in_ready,
    output wire                         out_valid,
    output wire [DATA_WIDTH-1:0]        out_data,
    output wire [PSUM_WIDTH-1:0]        out_prefix_data,
    output reg [OFFSET_ADDR_WIDTH-1:0]  out_k_offset,
    output wire                         batch_done
);
    reg  in_valid_q;
    reg [DATA_WIDTH-1:0] in_data_q;
    wire batch_start;

    wire dsc_clear_done, dsc_in_ready;
    wire sorter_in_ready, sorter_out_ready;

    wire si_we;
    wire [T_WIDTH-1:0] si_addr, si_prefix;

    wire                         sorter_out_valid;
    wire [DATA_WIDTH-1:0]        sorter_out_data;
    wire                         sorter_batch_done;

    reg                         out_valid_reg;
    reg [DATA_WIDTH-1:0]        out_data_reg;
    reg                         batch_done_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_valid_q <= 1'b0;
            in_data_q  <= {DATA_WIDTH{1'b0}};
            out_k_offset <= 0;
        end else begin
            in_valid_q <= in_valid; // 延迟一拍
            in_data_q  <= in_data;
            if(batch_start==1'b1) begin
              out_k_offset <= in_k_offset;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid_reg  <= 1'b0;
            out_data_reg   <= {DATA_WIDTH{1'b0}};
            batch_done_reg <= 1'b0;
        end else begin
            out_valid_reg  <= sorter_out_valid;
            out_data_reg   <= sorter_out_data;
            batch_done_reg <= sorter_batch_done;
        end
    end

    assign out_valid  = out_valid_reg;
    assign out_data   = out_data_reg;
    assign batch_done = batch_done_reg;

    assign batch_start = in_valid && !in_valid_q;

    assign in_ready = sorter_in_ready && dsc_in_ready;      
    assign sorter_out_ready = dsc_clear_done;             


    Hamming_Sorter_Dynamic #(
        .T_WIDTH(T_WIDTH), .DATA_WIDTH(DATA_WIDTH), .BATCH_SIZE(BATCH_SIZE)
    ) u_sorter (
        .clk(clk), .rst_n(rst_n),
        .in_batch_start(batch_start),
        .in_valid(in_valid_q), .in_data(in_data_q), .out_ready(sorter_out_ready),
        // output
        .in_ready(sorter_in_ready), 
        .out_valid(sorter_out_valid), .out_data(sorter_out_data),
        .batch_done(sorter_batch_done)
    );


    Dynamic_Scoreboard_Core #(
        .T(T_WIDTH), .BATCH_SIZE(BATCH_SIZE)
    ) u_dsb_core (
        // input
        .clk(clk), .rst_n(rst_n),
        .start_load(batch_start),
        .in_node(in_data_q[DATA_WIDTH-1:DATA_WIDTH-T_WIDTH]),
        .in_valid(in_valid_q),   
        // output
        .si_update_en(si_we),
        .si_addr(si_addr),
        .si_prefix(si_prefix),
        .clear_done(dsc_clear_done),
        .in_ready(dsc_in_ready)
    );

    xpm_memory_sdpram #(
        .ADDR_WIDTH_A(T_WIDTH), .ADDR_WIDTH_B(T_WIDTH),
        .MEMORY_SIZE(SI_SIZE * T_WIDTH), .WRITE_DATA_WIDTH_A(T_WIDTH), .READ_DATA_WIDTH_B(T_WIDTH),
        .READ_LATENCY_B(1),  // 读数据在时钟上升沿的下一个周期有效
        .WRITE_MODE_B("read_first"), .MEMORY_PRIMITIVE("distributed")
    ) u_si_ram (
        .clka(clk), .ena(1'b1), .wea(si_we), .addra(si_addr), .dina(si_prefix),
        .clkb(clk), .enb(1'b1), .addrb(sorter_out_data[DATA_WIDTH-1:DATA_WIDTH-T_WIDTH]), .doutb(out_prefix_data)
    );

endmodule