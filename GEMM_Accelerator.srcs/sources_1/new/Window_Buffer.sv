`timescale 1ns / 1ps

module Window_Buffer #(
    parameter T_WIDTH     = 8,
    parameter WINDOW_SIZE = 16,        // 必须为 2 的幂 (8, 16, 32, 64...)
    parameter BATCH_SIZE  = 256 * 4
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               valid_in,
    input  wire [T_WIDTH-1:0] current_node,
    output reg  [T_WIDTH-1:0] best_node, 
    output reg                valid_out
);

    // =========================================================================
    // 参数与内部信号声明置顶
    // =========================================================================
    localparam NUM_LEVELS = $clog2(WINDOW_SIZE); 

    // 存储与控制寄存器
    reg [T_WIDTH-1:0]             window [0:WINDOW_SIZE-1];
    reg [$clog2(BATCH_SIZE)-1:0]  batch_cnt; 
    reg [$clog2(WINDOW_SIZE)-1:0] ptr;

    // 参数化二叉树存储节点 (tree_dist[level][index])
    reg [$clog2(T_WIDTH):0]       tree_dist [0:NUM_LEVELS][0:WINDOW_SIZE-1];
    reg [T_WIDTH-1:0]             tree_node [0:NUM_LEVELS][0:WINDOW_SIZE-1];

    // 组合逻辑计算辅助变量
    integer i, j, k, lvl, p;
    reg [$clog2(T_WIDTH):0]       temp_count;
    reg [T_WIDTH-1:0]             xor_val;

    // =========================================================================
    // 逻辑实现
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ptr       <= 0;
            valid_out <= 0;
            batch_cnt <= 0;
            best_node <= 0;
            for(i = 0; i < WINDOW_SIZE; i = i+1) window[i] <= 0;
        end 
        else if (valid_in) begin
            // -----------------------------------------------------------------
            // 1. 并行计算 Hamming 距离 (Level 0 准备)
            // -----------------------------------------------------------------
            for(j = 0; j < WINDOW_SIZE; j = j+1) begin
                xor_val = current_node ^ window[j];
                temp_count = 0;
                // 此循环会被综合成高效的组合逻辑 (查1的个数)
                for(k = 0; k < T_WIDTH; k = k+1) begin
                    temp_count = temp_count + xor_val[k];
                end
                tree_dist[0][j] = temp_count;
                tree_node[0][j] = window[j];
            end

            // -----------------------------------------------------------------
            // 2. 参数化二叉树锦标赛逻辑 (Carry Data)
            // -----------------------------------------------------------------
            for (lvl = 0; lvl < NUM_LEVELS; lvl = lvl + 1) begin
                // 每一层的参与个数是 WINDOW_SIZE / (2^lvl)
                for (p = 0; p < (WINDOW_SIZE >> (lvl + 1)); p = p + 1) begin
                    if (tree_dist[lvl][2*p] <= tree_dist[lvl][2*p+1]) begin
                        tree_dist[lvl+1][p] = tree_dist[lvl][2*p];
                        tree_node[lvl+1][p] = tree_node[lvl][2*p];
                    end else begin
                        tree_dist[lvl+1][p] = tree_dist[lvl][2*p+1];
                        tree_node[lvl+1][p] = tree_node[lvl][2*p+1];
                    end
                end
            end
            
            // 树根（最后一层的第 0 个元素）即为胜者
            best_node <= tree_node[NUM_LEVELS][0];
            valid_out <= 1'b1;

            // -----------------------------------------------------------------
            // 3. 状态更新与 Batch 复位逻辑
            // -----------------------------------------------------------------
            if (batch_cnt == BATCH_SIZE - 1) begin
                batch_cnt <= 0;
                ptr       <= 0;
                for(i = 0; i < WINDOW_SIZE; i = i+1) window[i] <= 0;
            end else begin
                batch_cnt   <= batch_cnt + 1'b1;
                window[ptr] <= current_node; 
                ptr         <= ptr + 1'b1;
            end
        end else begin
            valid_out <= 0;
        end
    end

endmodule