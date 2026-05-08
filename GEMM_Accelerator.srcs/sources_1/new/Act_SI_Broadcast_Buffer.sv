`timescale 1ns / 1ps

module Act_SI_Broadcast_Buffer #(
    parameter N            = 256,
    parameter B_VEC_LEN    = 11008,                // 11008 = 256 * 43 (共需43个CU)
    parameter CU_NUMS      = B_VEC_LEN / N,
    parameter T_WIDTH      = 8,
    parameter SI_SIZE      = 2 ** T_WIDTH,
    parameter B_DATA_WIDTH = 8,
    parameter ADDR_W       = $clog2(N + 1)
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // --- 主机写入接口 ---
    input  wire                          host_we_act,
    input  wire [$clog2(B_VEC_LEN)-1:0]  host_addr_act,
    input  wire [B_DATA_WIDTH-1:0]       host_data_act,
    
    input  wire                          host_we_si,
    input  wire [T_WIDTH-1:0]            host_addr_si,
    input  wire [T_WIDTH-1:0]            host_data_si,

    // --- 广播控制接口 ---
    input  wire                          start_broadcast,
    output reg                           broadcast_done,

    // --- 向量 B 广播总线 (并行宽总线)---
    output reg                                   cfg_we_b,
    output reg  [$clog2(N)-1:0]                  cfg_addr_b,    
    output reg  [CU_NUMS * B_DATA_WIDTH - 1 : 0] cfg_data_b_bus, // 数据并行输出

    // --- SI 广播总线 ---
    output reg                                   cfg_we_si,
    output reg  [T_WIDTH-1:0]                    cfg_addr_si,    
    output reg  [T_WIDTH-1:0]                    cfg_data_si     // 共享一份si数据，所有CU同时接收
);
    // =========================================================================
    // 1. 内部存储器
    // =========================================================================
    // Vector B 内存：分成 CU_NUMS 份，每份 N 字节，分别对应一个 CU 的广播数据
    genvar m;
    generate
        for (m = 0; m < CU_NUMS; m = m + 1) begin : B_MEM_GEN
            (* ram_style = "block" *) reg [B_DATA_WIDTH-1:0] sub_act_mem [0:N-1];
            
            // 主机写入
            wire is_this_cu = (host_addr_act[$clog2(B_VEC_LEN)-1:$clog2(N)] == m);
            always @(posedge clk) begin
                if (host_we_act && is_this_cu)
                    sub_act_mem[host_addr_act[$clog2(N)-1:0]] <= host_data_act;
            end

            // 数据输出 1拍延迟
            always @(posedge clk) begin
                cfg_data_b_bus[m*B_DATA_WIDTH +: B_DATA_WIDTH] <= sub_act_mem[brd_cnt_b[$clog2(N)-1:0]];
            end
        end
    endgenerate

    // SI 内存：SI_SIZE 字节，所有 CU 共享一份 SI 数据
    (* ram_style = "block" *) reg [T_WIDTH-1:0] si_mem [0:SI_SIZE-1];
    // 数据输出 1拍延迟
    always @(posedge clk) if (host_we_si) si_mem[host_addr_si] <= host_data_si;
    always @(posedge clk) cfg_data_si <= si_mem[brd_cnt_si];

    // =========================================================================
    // 2. 广播状态机与对齐逻辑
    // =========================================================================
    reg [1:0] state;
    localparam IDLE         = 2'd0;
    localparam BROADCAST    = 2'd1;
    localparam WAIT_LAST    = 2'd2; 
    localparam DONE         = 2'd3;

    reg [ADDR_W-1:0]  brd_cnt_b;
    reg [T_WIDTH:0]   brd_cnt_si;

    reg b_done_flag;  // Vector B 广播完成标志
    reg si_done_flag; // SI 广播完成标志

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            brd_cnt_b      <= 1'b0;
            brd_cnt_si     <= 1'b0;
            broadcast_done <= 1'b0;
            cfg_we_b       <= 1'b0; 
            cfg_addr_b     <= 1'b0;
            cfg_we_si      <= 1'b0; 
            cfg_addr_si    <= 1'b0; 
            cfg_data_si    <= 1'b0;
            b_done_flag    <= 1'b0;
            si_done_flag   <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    broadcast_done <= 1'b0;
                    cfg_we_b       <= 1'b0;
                    cfg_we_si      <= 1'b0;
                    brd_cnt_b      <= 1'b0;
                    brd_cnt_si     <= 1'b0;
                    b_done_flag    <= 1'b0;
                    si_done_flag   <= 1'b0;
                    if (start_broadcast) state <= BROADCAST;
                end
                
                BROADCAST: begin                    
                    // Vector B 控制
                    if (brd_cnt_b < N) begin
                        cfg_we_b   <= 1'b1;
                        cfg_addr_b <= brd_cnt_b[$clog2(N)-1:0];
                    end else begin
                        cfg_we_b   <= 1'b0;
                    end

                    // SI 向量控制与数据
                    if (brd_cnt_si < SI_SIZE) begin
                        cfg_we_si   <= 1'b1;
                        cfg_addr_si <= brd_cnt_si[T_WIDTH-1:0];
                    end else begin
                        cfg_we_si   <= 1'b0;
                    end
                    
                    if (brd_cnt_b == N - 1) begin
                        b_done_flag <= 1'b1; // Vector B 最后一拍数据正在被广播
                    end
                    if (brd_cnt_si == SI_SIZE - 1) begin
                        si_done_flag <= 1'b1; // SI 最后一拍数据正在被广播
                    end

                    // 状态跳转
                    if (b_done_flag && si_done_flag) begin
                        state <= WAIT_LAST;
                    end else begin
                        brd_cnt_b <= brd_cnt_b + 1'b1;
                        brd_cnt_si <= brd_cnt_si + 1'b1;
                    end
                end

                WAIT_LAST: begin
                    cfg_we_b  <= 1'b0;
                    cfg_we_si <= 1'b0;
                    state     <= DONE;
                end
                
                DONE: begin
                    broadcast_done <= 1'b1;
                    state          <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule