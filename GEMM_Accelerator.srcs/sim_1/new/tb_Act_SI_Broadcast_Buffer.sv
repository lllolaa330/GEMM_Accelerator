`timescale 1ns / 1ps

module tb_Act_SI_Broadcast_Buffer();

    reg clk;
    reg rst_n;
    
    reg host_we_act;
    reg [$clog2(11008)-1:0] host_addr_act;
    reg [7:0] host_data_act;
    reg host_we_si;
    reg [7:0] host_addr_si;
    reg [7:0] host_data_si;
    reg start_broadcast;
    
    wire broadcast_done;
    wire cfg_we_b;
    wire [$clog2(256)-1:0] cfg_addr_b;
    wire [43 * 8 - 1 : 0]  cfg_data_b_bus; // 43个数据并行输出
    wire cfg_we_si;
    wire [7:0] cfg_addr_si;
    wire [7:0] cfg_data_si;

    Act_SI_Broadcast_Buffer #(.N(256), .B_VEC_LEN(11008)) dut (.*);

    initial begin
        clk = 0; forever #5 clk = ~clk; 
    end

    integer i;

    initial begin
        rst_n = 0;
        host_we_act = 0; host_we_si = 0; start_broadcast = 0;
        #20 rst_n = 1;
        
        // $display("[%0t] Host Writing B to Buffer...", $time);
        // for(i=0; i<11008; i=i+1) begin
        //     @(posedge clk);
        //     host_we_act <= 1; host_addr_act <= i; host_data_act <= i * 2;
        //     $display("  Wrote act_mem[%0d] = %0d", i, i*2);
        // end
        // @(posedge clk);
        // host_we_act <= 0;

        // $display("[%0t] Host Writing SI to Buffer...", $time);
        // for(i=0; i<256; i=i+1) begin
        //     @(posedge clk);
        //     host_we_si <= 1; host_addr_si <= i; host_data_si <= i * 3;
        //     $display("  Wrote si_mem[%0d] = %0d", i, i*3);
        // end
        // @(posedge clk);
        // host_we_si <= 0;

        $display("[%0t] Host Writing B and SI to Buffer...", $time);
        for(i=0; i<11008; i=i+1) begin
            @(posedge clk);
            host_we_act <= 1; host_addr_act <= i; host_data_act <= i * 2;
            $display("  Wrote act_mem[%0d] = %0d", i, i*2);
            if (i < 256) begin
                host_we_si <= 1; host_addr_si <= i; host_data_si <= i * 3;
                $display("  Wrote si_mem[%0d] = %0d", i, i*3);
            end else begin
                host_we_si <= 0;
            end
        end
        @(posedge clk);
        host_we_act <= 0;
        host_we_si <= 0;
        $display("[%0t] Starting Broadcast...", $time);
        @(posedge clk);
        start_broadcast <= 1;
        @(posedge clk);
        start_broadcast <= 0;
        
        wait(broadcast_done);
        $display("[%0t] Broadcast Finished!", $time);
        #50 $finish;
    end
endmodule