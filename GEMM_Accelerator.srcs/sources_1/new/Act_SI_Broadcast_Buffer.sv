`timescale 1ns / 1ps

module Act_SI_Broadcast_Buffer #(
    parameter N            = 256,
    parameter B_VEC_LEN    = 4096,                
    parameter T_WIDTH      = 8,
    parameter B_DATA_WIDTH = 8,
    parameter CU_NUMS      = B_VEC_LEN / N,
    parameter SI_SIZE      = 2 ** T_WIDTH,
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
    input  wire                          start_broadcast_b,
    output reg                           broadcast_done_b,
    input  wire                          start_broadcast_si,
    output reg                           broadcast_done_si,

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
    // 0. 内部计数器信号定义 (用于替换原有的 brd_cnt)
    // =========================================================================
    reg [ADDR_W-1:0]  cnt_b;
    reg [T_WIDTH:0]   cnt_si;

    // =========================================================================
    // 1. 内部存储器及数据输出逻辑 (读延迟1拍，与地址输出对齐)
    // =========================================================================
    // Vector B 内存
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

            // 数据输出 1拍延迟 (利用内部计数器读取)
            always @(posedge clk) begin
                cfg_data_b_bus[m*B_DATA_WIDTH +: B_DATA_WIDTH] <= sub_act_mem[cnt_b[$clog2(N)-1:0]];
            end
        end
    endgenerate

    // SI 内存
    (* ram_style = "block" *) reg [T_WIDTH-1:0] si_mem [0:SI_SIZE-1];
    always @(posedge clk) begin
        if (host_we_si) si_mem[host_addr_si] <= host_data_si;
    end
    always @(posedge clk) begin
        cfg_data_si <= si_mem[cnt_si[T_WIDTH-1:0]];
    end

    // =========================================================================
    // 2. 独立的 Vector B 广播状态机
    // =========================================================================
    localparam S_B_IDLE = 2'd0;
    localparam S_B_WORK = 2'd1;
    localparam S_B_DONE = 2'd2;

    reg [1:0] state_b, next_state_b;

    // FSM_B 状态寄存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state_b <= S_B_IDLE;
        else        state_b <= next_state_b;
    end

    // FSM_B 状态转移逻辑
    always @(*) begin
        next_state_b = state_b; // 默认保持
        case (state_b)
            S_B_IDLE: if (start_broadcast_b) next_state_b = S_B_WORK;
            S_B_WORK: if (cnt_b == N - 1)    next_state_b = S_B_DONE;
            S_B_DONE:                        next_state_b = S_B_IDLE;
            default:                         next_state_b = S_B_IDLE;
        endcase
    end

    // FSM_B 输出逻辑与计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_b            <= 0;
            cfg_we_b         <= 1'b0;
            cfg_addr_b       <= 0;
            broadcast_done_b <= 1'b0;
        end else begin
            case (state_b)
                S_B_IDLE: begin
                    cnt_b            <= 0;
                    cfg_we_b         <= 1'b0;
                    broadcast_done_b <= 1'b0;
                end
                S_B_WORK: begin
                    cnt_b      <= cnt_b + 1'b1;
                    cfg_we_b   <= 1'b1;
                    cfg_addr_b <= cnt_b[$clog2(N)-1:0]; // cfg_addr 与 延迟一拍的RAM cfg_data 完美对齐
                end
                S_B_DONE: begin
                    cfg_we_b         <= 1'b0;
                    cfg_addr_b       <= 1'b0; 
                    cfg_data_b_bus   <= 0;
                    broadcast_done_b <= 1'b1;           // 产生一拍的 Done 信号
                end
                default: ;
            endcase
        end
    end

    // =========================================================================
    // 3. 独立的 SI 广播状态机 (标准三段式)
    // =========================================================================
    localparam S_SI_IDLE = 2'd0;
    localparam S_SI_WORK = 2'd1;
    localparam S_SI_DONE = 2'd2;

    reg [1:0] state_si, next_state_si;

    // FSM_SI 状态寄存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state_si <= S_SI_IDLE;
        else        state_si <= next_state_si;
    end

    // FSM_SI 状态转移逻辑
    always @(*) begin
        next_state_si = state_si; // 默认保持
        case (state_si)
            S_SI_IDLE: if (start_broadcast_si) next_state_si = S_SI_WORK;
            S_SI_WORK: if (cnt_si == SI_SIZE - 1) next_state_si = S_SI_DONE;
            S_SI_DONE:                            next_state_si = S_SI_IDLE;
            default:                              next_state_si = S_SI_IDLE;
        endcase
    end

    // FSM_SI 输出逻辑与计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_si            <= 0;
            cfg_we_si         <= 1'b0;
            cfg_addr_si       <= 0;
            broadcast_done_si <= 1'b0;
        end else begin
            case (state_si)
                S_SI_IDLE: begin
                    cnt_si            <= 0;
                    cfg_we_si         <= 1'b0;
                    broadcast_done_si <= 1'b0;
                end
                S_SI_WORK: begin
                    cnt_si      <= cnt_si + 1'b1;
                    cfg_we_si   <= 1'b1;
                    cfg_addr_si <= cnt_si[T_WIDTH-1:0]; // 同理，保证地址输出与RAM读出数据在同一拍
                end
                S_SI_DONE: begin
                    cfg_we_si         <= 1'b0;
                    broadcast_done_si <= 1'b1;          // 产生一拍的 Done 信号
                end
                default: ;
            endcase
        end
    end

endmodule