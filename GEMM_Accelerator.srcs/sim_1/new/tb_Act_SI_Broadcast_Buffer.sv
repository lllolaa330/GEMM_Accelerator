`timescale 1ns / 1ps

module tb_Act_SI_Broadcast_Buffer();

    reg clk;
    reg rst_n;
    
    reg host_we_act;
    reg [7:0] host_addr_act;
    reg [7:0] host_data_act;
    reg start_broadcast;
    
    wire broadcast_done;
    wire cfg_we_b;
    wire [7:0] cfg_addr_b;
    wire [7:0] cfg_data_b;

    Act_SI_Broadcast_Buffer #(.MAX_ACT_LEN(16)) dut (.*, .host_we_si(0), .host_addr_si(0), .host_data_si(0));

    initial begin
        clk = 0; forever #5 clk = ~clk; 
    end

    integer i;

    initial begin
        rst_n = 0;
        host_we_act = 0; start_broadcast = 0;
        #20 rst_n = 1;
        
        $display("[%0t] Host Writing to Buffer...", $time);
        for(i=0; i<16; i=i+1) begin
            @(posedge clk);
            host_we_act <= 1; host_addr_act <= i; host_data_act <= i * 10;
        end
        @(posedge clk);
        host_we_act <= 0;
        
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