`timescale 1ns / 1ps

module Act_SI_Broadcast_Buffer #(
    parameter MAX_ACT_LEN = 256,         // Vector B 最大长度
    parameter SI_SIZE     = 256,         // Search Index 容量
    parameter DATA_WIDTH  = 8
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // --- 主机 (Host) 写入接口 ---
    input  wire                          host_we_act,
    input  wire [$clog2(MAX_ACT_LEN)-1:0]host_addr_act,
    input  wire [DATA_WIDTH-1:0]         host_data_act,
    
    input  wire                          host_we_si,
    input  wire [$clog2(SI_SIZE)-1:0]    host_addr_si,
    input  wire [DATA_WIDTH-1:0]         host_data_si,

    // --- 广播控制接口 ---
    input  wire                          start_broadcast,
    output reg                           broadcast_done,

    // --- 向 43 个 CU 广播的输出总线 ---
    // (目前底层 CU 只接了 Act 的配置，SI 可预留接线)
    output reg                           cfg_we_b,
    output reg  [7:0]                    cfg_addr_b,
    output reg  [7:0]                    cfg_data_b
);

    // =========================================================================
    // 内部信号与存储器声明
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] act_mem [0:MAX_ACT_LEN-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] si_mem  [0:SI_SIZE-1];

    reg [1:0] state;
    localparam IDLE       = 2'd0;
    localparam BROADCAST  = 2'd1;
    localparam DONE       = 2'd2;

    reg [$clog2(MAX_ACT_LEN)-1:0] brd_cnt;

    // =========================================================================
    // 逻辑实现
    // =========================================================================

    // --- 主机写入逻辑 ---
    always @(posedge clk) begin
        if (host_we_act) act_mem[host_addr_act] <= host_data_act;
        if (host_we_si)  si_mem[host_addr_si]   <= host_data_si;
    end

    // --- 广播状态机与地址发生器 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            brd_cnt        <= 0;
            broadcast_done <= 1'b0;
            cfg_we_b       <= 1'b0;
            cfg_addr_b     <= 8'd0;
            cfg_data_b     <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    broadcast_done <= 1'b0;
                    cfg_we_b       <= 1'b0;
                    if (start_broadcast) begin
                        state   <= BROADCAST;
                        brd_cnt <= 0;
                    end
                end
                
                BROADCAST: begin
                    cfg_we_b   <= 1'b1;
                    cfg_addr_b <= brd_cnt;
                    cfg_data_b <= act_mem[brd_cnt]; // 读出并立即广播
                    
                    if (brd_cnt == MAX_ACT_LEN - 1) begin
                        state <= DONE;
                    end else begin
                        brd_cnt <= brd_cnt + 1'b1;
                    end
                end
                
                DONE: begin
                    cfg_we_b       <= 1'b0;
                    broadcast_done <= 1'b1;
                    state          <= IDLE;
                end
            endcase
        end
    end
endmodule