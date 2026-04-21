`timescale 1ns / 1ps

module PingPong_Weight_BRAM #(
    parameter DATA_WIDTH     = 18,     // A_PACKED_WIDTH (10 bit节点 + 8 bit T)
    parameter DEPTH_PER_BANK = 1024    // 单个 Batch 的深度 (BATCH_SIZE)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // --- Ping-Pong 控制信号 ---
    // 0: DMA写Bank 0(Ping), CU读Bank 1(Pong)
    // 1: DMA写Bank 1(Pong), CU读Bank 0(Ping)
    input  wire                      ping_pong_sel, 

    // --- Port A: DMA 写入接口 (Global -> BRAM) ---
    input  wire                      dma_wr_en,
    input  wire [$clog2(DEPTH_PER_BANK)-1:0] dma_wr_addr,
    input  wire [DATA_WIDTH-1:0]     dma_wr_data,

    // --- Port B: CU 读取接口 (BRAM -> CU) ---
    input  wire                      cu_rd_en,
    input  wire [$clog2(DEPTH_PER_BANK)-1:0] cu_rd_addr,
    output wire [DATA_WIDTH-1:0]     cu_rd_data
);

    // =========================================================================
    // 参数与内部信号声明
    // =========================================================================
    localparam ADDR_W = $clog2(DEPTH_PER_BANK);
    
    // 物理地址：最高位用于区分 Ping (0) 和 Pong (1)
    wire [ADDR_W:0] physical_wr_addr;
    wire [ADDR_W:0] physical_rd_addr;

    // =========================================================================
    // 逻辑实现
    // =========================================================================
    
    // --- 地址拼接逻辑 ---
    // DMA 写端：当 sel=0 时写 Bank0，当 sel=1 时写 Bank1
    assign physical_wr_addr = {ping_pong_sel, dma_wr_addr};
    
    // CU 读端：当 sel=0 时读 Bank1，当 sel=1 时读 Bank0 (严格互补)
    assign physical_rd_addr = {~ping_pong_sel, cu_rd_addr};

    // --- 例化 Xilinx 真正的双端口 RAM ---
    xpm_memory_tdpram #(
        .ADDR_WIDTH_A       (ADDR_W + 1),       // 总深度 2048 -> 地址宽 11
        .ADDR_WIDTH_B       (ADDR_W + 1),
        .MEMORY_SIZE        ((DEPTH_PER_BANK * 2) * DATA_WIDTH), 
        .WRITE_DATA_WIDTH_A (DATA_WIDTH),
        .READ_DATA_WIDTH_B  (DATA_WIDTH),
        .READ_LATENCY_B     (1),                // BRAM 读延迟设置为 1 拍
        .WRITE_MODE_A       ("no_change"),      // 推荐的低功耗写模式
        .WRITE_MODE_B       ("read_first"),
        .MEMORY_PRIMITIVE   ("block"),          // 强制使用 BRAM
        .CLOCKING_MODE      ("common_clock"),
        .MEMORY_INIT_PARAM  ("0")
    ) u_xpm_pingpong_ram (
        .clka           (clk),
        .ena            (1'b1),
        .wea            (dma_wr_en),
        .addra          (physical_wr_addr),
        .dina           (dma_wr_data),
        
        .clkb           (clk),
        .enb            (cu_rd_en),
        .addrb          (physical_rd_addr),
        .doutb          (cu_rd_data),
        
        // 未使用引脚绑零/悬空
        .regceb         (1'b1),
        .rstb           (1'b0),
        .sleep          (1'b0),
        .injectsbiterra (1'b0),
        .injectdbiterra (1'b0),
        .sbiterrb       (),
        .dbiterrb       ()
    );

endmodule