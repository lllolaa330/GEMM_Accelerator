`timescale 1ns / 1ps

module Act_SI_Broadcast_Buffer #(
    parameter N            = 256,
    parameter B_VEC_LEN    = 11008,                // 11008 = 256 * 43 (共需43个CU)
    parameter CU_NUMS      = B_VEC_LEN / N,
    parameter T_WIDTH      = 8,
    parameter SI_SIZE      = 2 ** T_WIDTH,
    parameter B_DATA_WIDTH = 8,
    parameter MAX_CNT      = N,
    parameter ADDR_W       = $clog2(MAX_CNT + 1)
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
    output reg  [CU_NUMS * B_DATA_WIDTH - 1 : 0] cfg_data_b_bus, // 43个数据并行输出

    // --- SI 广播总线 ---
    output reg                           cfg_we_si,
    output reg  [T_WIDTH-1:0]            cfg_addr_si,    
    output reg  [T_WIDTH-1:0]            cfg_data_si
);
   // =========================================================================
    // 1. 内部存储器：43块子内存
    // =========================================================================
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

            // --- 核心修改：数据输出 (1拍延迟) ---
            always @(posedge clk) begin
                cfg_data_b_bus[m*B_DATA_WIDTH +: B_DATA_WIDTH] <= sub_act_mem[brd_cnt[$clog2(N)-1:0]];
            end
        end
    endgenerate

    // SI 内存
    (* ram_style = "block" *) reg [T_WIDTH-1:0] si_mem [0:SI_SIZE-1];
    always @(posedge clk) if (host_we_si) si_mem[host_addr_si] <= host_data_si;

    // =========================================================================
    // 2. 广播状态机与对齐逻辑
    // =========================================================================
    reg [1:0] state;
    localparam IDLE       = 2'd0;
    localparam BROADCAST  = 2'd1;
    localparam WAIT_LAST  = 2'd2; // 等待最后一拍数据出炉
    localparam DONE       = 2'd3;

    reg [ADDR_W-1:0] brd_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            brd_cnt        <= 0;
            broadcast_done <= 1'b0;
            cfg_we_b       <= 0; 
            cfg_addr_b     <= 0;
            cfg_we_si      <= 0; 
            cfg_addr_si    <= 0; 
            cfg_data_si    <= 0;
        end else begin
            case (state)
                IDLE: begin
                    broadcast_done <= 1'b0;
                    cfg_we_b       <= 1'b0;
                    cfg_we_si      <= 1'b0;
                    brd_cnt        <= 0;
                    if (start_broadcast) state <= BROADCAST;
                end
                
                BROADCAST: begin
                    // --- 核心修改：控制信号直接由当前 brd_cnt 驱动 (1拍延迟) ---
                    // 这里的赋值是在 posedge clk 发生的，所以端口上看到跳变是在下一拍
                    // 正好对齐了 cfg_data_b_bus 的 BRAM 读取延迟
                    
                    // Vector B 控制
                    if (brd_cnt < N) begin
                        cfg_we_b   <= 1'b1;
                        cfg_addr_b <= brd_cnt[$clog2(N)-1:0];
                    end else begin
                        cfg_we_b   <= 1'b0;
                    end

                    // SI 向量控制与数据 (如果是BRAM读取，逻辑一致)
                    if (brd_cnt < SI_SIZE) begin
                        cfg_we_si   <= 1'b1;
                        cfg_addr_si <= brd_cnt[T_WIDTH-1:0];
                        cfg_data_si <= si_mem[brd_cnt[T_WIDTH-1:0]];
                    end else begin
                        cfg_we_si   <= 1'b0;
                    end
                    
                    // 状态跳转
                    if (brd_cnt == MAX_CNT - 1) begin
                        state <= WAIT_LAST;
                    end else begin
                        brd_cnt <= brd_cnt + 1'b1;
                    end
                end

                WAIT_LAST: begin
                    // 此时最后一拍有效数据 (addr=255) 刚好在总线上
                    // 我们需要在这里把 WE 清零，防止下一拍误写
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