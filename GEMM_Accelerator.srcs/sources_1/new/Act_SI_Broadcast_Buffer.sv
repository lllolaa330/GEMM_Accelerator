`timescale 1ns / 1ps

module Act_SI_Broadcast_Buffer #(
    parameter N            = 256,
    parameter T_WIDTH      = 8,
    parameter SI_SIZE      = 2 ** T_WIDTH,
    parameter B_DATA_WIDTH = 8,
    parameter MAX_CNT      = (N > SI_SIZE) ? N : SI_SIZE,
    parameter ADDR_W       = $clog2(MAX_CNT + 1)
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // --- 主机写入接口 ---
    input  wire                          host_we_act,
    input  wire [$clog2(N)-1:0]          host_addr_act,
    input  wire [B_DATA_WIDTH-1:0]       host_data_act,
    
    input  wire                          host_we_si,
    input  wire [T_WIDTH-1:0]            host_addr_si,
    input  wire [T_WIDTH-1:0]            host_data_si,

    // --- 广播控制接口 ---
    input  wire                          start_broadcast,
    output reg                           broadcast_done,

    // --- 向量 B 广播总线 ---
    output reg                           cfg_we_b,
    output reg  [$clog2(N)-1:0]          cfg_addr_b,    
    output reg  [B_DATA_WIDTH-1:0]       cfg_data_b,

    // --- SI 广播总线 ---
    output reg                           cfg_we_si,
    output reg  [T_WIDTH-1:0]            cfg_addr_si,    
    output reg  [T_WIDTH-1:0]            cfg_data_si
);

    // =========================================================================
    // 内部存储器声明
    // =========================================================================
    (* ram_style = "block" *) reg [B_DATA_WIDTH-1:0] act_mem [0:N-1];
    (* ram_style = "block" *) reg [T_WIDTH-1:0]      si_mem  [0:SI_SIZE-1];

    // --- 初始化内存 ---
    integer mem_i;
    initial begin
        for (mem_i = 0; mem_i < N; mem_i = mem_i + 1) act_mem[mem_i] = 0;
        for (mem_i = 0; mem_i < SI_SIZE; mem_i = mem_i + 1) si_mem[mem_i] = 0;
    end

    reg [1:0] state;
    localparam IDLE       = 2'd0;
    localparam BROADCAST  = 2'd1;
    localparam DONE       = 2'd2;

    reg [ADDR_W-1:0] brd_cnt;

    // --- 主机写入逻辑 ---
    always @(posedge clk) begin
        if (host_we_act) act_mem[host_addr_act] <= host_data_act;
        if (host_we_si)  si_mem[host_addr_si]   <= host_data_si;
    end

    // --- 广播状态机 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            brd_cnt        <= 0;
            broadcast_done <= 1'b0;
            cfg_we_b <= 0; cfg_addr_b <= 0; cfg_data_b <= 0;
            cfg_we_si <= 0; cfg_addr_si <= 0; cfg_data_si <= 0;
        end else begin
            case (state)
                IDLE: begin
                    broadcast_done <= 1'b0;
                    cfg_we_b       <= 1'b0;
                    cfg_we_si      <= 1'b0;
                    brd_cnt        <= 0;
                    if (start_broadcast) begin
                        state   <= BROADCAST;
                    end
                end
                
                BROADCAST: begin
                    // --- 路由 1: Vector B 广播 ---
                    if (brd_cnt < N) begin
                        cfg_we_b   <= 1'b1;
                        cfg_addr_b <= brd_cnt[$clog2(N)-1:0];
                        cfg_data_b <= act_mem[brd_cnt];
                    end else begin
                        cfg_we_b   <= 1'b0;
                    end

                    // --- 路由 2: SI 广播 ---
                    if (brd_cnt < SI_SIZE) begin
                        cfg_we_si   <= 1'b1;
                        cfg_addr_si <= brd_cnt[T_WIDTH-1:0];
                        cfg_data_si <= si_mem[brd_cnt];
                    end else begin
                        cfg_we_si   <= 1'b0;
                    end
                    
                    // --- 计数与跳转逻辑 ---
                    if (brd_cnt == MAX_CNT - 1) begin
                        state <= DONE;
                    end else begin
                        brd_cnt <= brd_cnt + 1'b1;
                    end
                end
                
                DONE: begin
                    cfg_we_b       <= 1'b0;
                    cfg_we_si      <= 1'b0;
                    broadcast_done <= 1'b1;
                    state          <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule