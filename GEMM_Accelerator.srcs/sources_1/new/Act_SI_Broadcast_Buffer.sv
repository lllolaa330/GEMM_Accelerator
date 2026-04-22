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
    // 内部存储器与输出流水线
    // =========================================================================
    
    // 预备信号（用于打拍同步）
    reg                          we_b_pre;
    reg [$clog2(N)-1:0]          addr_b_pre;
    reg                          we_si_pre;
    reg [T_WIDTH-1:0]            addr_si_pre;
    reg [T_WIDTH-1:0]            data_si_pre;

    genvar m;
    generate
        for (m = 0; m < CU_NUMS; m = m + 1) begin : B_MEM_GEN
            (* ram_style = "block" *) reg [B_DATA_WIDTH-1:0] sub_act_mem [0:N-1];
            
            // 主机写入逻辑
            wire is_this_cu = (host_addr_act[$clog2(B_VEC_LEN)-1:$clog2(N)] == m);
            always @(posedge clk) begin
                if (host_we_act && is_this_cu)
                    sub_act_mem[host_addr_act[$clog2(N)-1:0]] <= host_data_act;
            end

            // --- 时序优化：内存输出寄存器 ---
            // brd_cnt 在当前周期给出地址，数据在下一周期进入 cfg_data_b_bus
            always @(posedge clk) begin
                cfg_data_b_bus[m*B_DATA_WIDTH +: B_DATA_WIDTH] <= sub_act_mem[brd_cnt[$clog2(N)-1:0]];
            end
        end
    endgenerate

    // SI 内存
    (* ram_style = "block" *) reg [T_WIDTH-1:0] si_mem [0:SI_SIZE-1];
    always @(posedge clk) if (host_we_si) si_mem[host_addr_si] <= host_data_si;

    // =========================================================================
    // 状态机与对齐逻辑
    // =========================================================================
    reg [1:0] state;
    localparam IDLE       = 2'd0;
    localparam BROADCAST  = 2'd1;
    localparam WAIT_LAST  = 2'd2; // 新增：等待最后一拍数据输出
    localparam DONE       = 2'd3;

    reg [ADDR_W-1:0] brd_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            brd_cnt <= 0;
            broadcast_done <= 1'b0;
            // 清除输出寄存器
            cfg_we_b <= 0; cfg_addr_b <= 0;
            cfg_we_si <= 0; cfg_addr_si <= 0; cfg_data_si <= 0;
            // 清除内部预备信号
            we_b_pre <= 0; addr_b_pre <= 0;
            we_si_pre <= 0; addr_si_pre <= 0; data_si_pre <= 0;
        end else begin
            // --- 控制信号流水线：延迟一拍以匹配内存读取延迟 ---
            cfg_we_b    <= we_b_pre;
            cfg_addr_b  <= addr_b_pre;
            cfg_we_si   <= we_si_pre;
            cfg_addr_si <= addr_si_pre;
            cfg_data_si <= data_si_pre;

            case (state)
                IDLE: begin
                    broadcast_done <= 1'b0;
                    we_b_pre  <= 1'b0;
                    we_si_pre <= 1'b0;
                    brd_cnt   <= 0;
                    if (start_broadcast) state <= BROADCAST;
                end
                
                BROADCAST: begin
                    // 准备 B 向量控制信号
                    if (brd_cnt < N) begin
                        we_b_pre   <= 1'b1;
                        addr_b_pre <= brd_cnt[$clog2(N)-1:0];
                    end else begin
                        we_b_pre   <= 1'b0;
                    end

                    // 准备 SI 向量控制信号与数据读取
                    if (brd_cnt < SI_SIZE) begin
                        we_si_pre   <= 1'b1;
                        addr_si_pre <= brd_cnt[T_WIDTH-1:0];
                        data_si_pre <= si_mem[brd_cnt[T_WIDTH-1:0]];
                    end else begin
                        we_si_pre   <= 1'b0;
                    end
                    
                    if (brd_cnt == MAX_CNT - 1) begin
                        state <= WAIT_LAST; // 最后一组地址已发出，需多等一拍让数据出炉
                    end else begin
                        brd_cnt <= brd_cnt + 1'b1;
                    end
                end

                WAIT_LAST: begin
                    // 最后一拍数据正在移入 cfg_data_b_bus
                    // 此时 we_b_pre 应该变低，但由于上面有 cfg_we_b <= we_b_pre，
                    // 最后一组有效控制信号会在本周期末传递给输出寄存器
                    we_b_pre  <= 1'b0;
                    we_si_pre <= 1'b0;
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

//     // =========================================================================
//     // 内部存储器声明
//     // =========================================================================
//     // B Vector Buffer：分为 CU_NUMS 个子内存，每个子内存存储 N 个元素
//     genvar m;
//     generate
//         for (m = 0; m < CU_NUMS; m = m + 1) begin : B_MEM_GEN
//             (* ram_style = "block" *) reg [B_DATA_WIDTH-1:0] sub_act_mem [0:N-1];
//             // 主机写入逻辑：解码高位地址选择对应的子内存
//             wire is_this_cu = (host_addr_act >> $clog2(N)) == m;
//             always @(posedge clk) begin
//                 if (host_we_act && is_this_cu)
//                     sub_act_mem[host_addr_act[$clog2(N)-1:0]] <= host_data_act;
//             end

//             // 广播读取逻辑：所有子内存共用一个 brd_cnt
//             // 注意：为了时序，这里可以加一级寄存器，此处采用组合逻辑演示逻辑
//             assign cfg_data_b_bus[m*B_DATA_WIDTH +: B_DATA_WIDTH] = sub_act_mem[brd_cnt];
//         end
//     endgenerate

//     // SI Buffer：单一内存，存储 SI_SIZE 个元素
//     (* ram_style = "block" *) reg [T_WIDTH-1:0]      si_mem  [0:SI_SIZE-1];
//     always @(posedge clk) begin
//         if (host_we_si)  si_mem[host_addr_si]   <= host_data_si;
//     end
    
//     reg [1:0] state;
//     localparam IDLE       = 2'd0;
//     localparam BROADCAST  = 2'd1;
//     localparam DONE       = 2'd2;

//     reg [ADDR_W-1:0] brd_cnt;

//     // --- 广播状态机 ---
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             state          <= IDLE;
//             brd_cnt        <= 0;
//             broadcast_done <= 1'b0;
//             cfg_we_b <= 0; cfg_addr_b <= 0; cfg_data_b <= 0;
//             cfg_we_si <= 0; cfg_addr_si <= 0; cfg_data_si <= 0;
//         end else begin
//             case (state)
//                 IDLE: begin
//                     broadcast_done <= 1'b0;
//                     cfg_we_b       <= 1'b0;
//                     cfg_we_si      <= 1'b0;
//                     brd_cnt        <= 0;
//                     if (start_broadcast) begin
//                         state   <= BROADCAST;
//                     end
//                 end
                
//                 BROADCAST: begin
//                     // --- 路由 1: Vector B 广播 ---
//                     // 所有CU共用WE和地址，但数据线上是43个不同的值
//                     if (brd_cnt < N) begin
//                         cfg_we_b   <= 1'b1;
//                         cfg_addr_b <= brd_cnt[$clog2(N)-1:0];
//                     end else begin
//                         cfg_we_b   <= 1'b0;
//                     end

//                     // --- 路由 2: SI 广播 ---
//                     if (brd_cnt < SI_SIZE) begin
//                         cfg_we_si   <= 1'b1;
//                         cfg_addr_si <= brd_cnt[T_WIDTH-1:0];
//                         cfg_data_si <= si_mem[brd_cnt];
//                     end else begin
//                         cfg_we_si   <= 1'b0;
//                     end
                    
//                     // --- 计数与跳转逻辑 ---
//                     if (brd_cnt == MAX_CNT - 1) begin
//                         state <= DONE;
//                     end else begin
//                         brd_cnt <= brd_cnt + 1'b1;
//                     end
//                 end
                
//                 DONE: begin
//                     cfg_we_b       <= 1'b0;
//                     cfg_we_si      <= 1'b0;
//                     broadcast_done <= 1'b1;
//                     state          <= IDLE;
//                 end
//                 default: state <= IDLE;
//             endcase
//         end
//     end
// endmodule