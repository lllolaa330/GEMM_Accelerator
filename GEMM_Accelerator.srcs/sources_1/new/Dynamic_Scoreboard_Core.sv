`timescale 1ns / 1ps

module Dynamic_Scoreboard_Core #(
    parameter T = 8,
    parameter NUM_NODES = 2**T,
    parameter DIST_WIDTH = 3,    // 0-7, 论文建议记录到 3 
    parameter BATCH_SIZE = 1024
)(
    input  wire                 clk,
    input  wire                 rst_n,
    
    // --- 任务触发控制 ---
    input  wire                 start_load,    // 开始接收 Tile 数据
    input  wire [T-1:0]         in_node,       // 输入的 TransRow 索引
    input  wire                 in_valid,
    
    // --- 计算结果输出 ---
    output reg                  si_update_en,  // SI 写使能
    output reg  [T-1:0]         si_addr,       // Node ID
    output reg  [T-1:0]         si_prefix,     // 算出的最佳 Prefix ID
    output reg                  clear_done,    // si 计算完毕且内存清理完毕
    output reg                  in_ready       // IDLE 状态，准备好接收新 Batch
);

    // =========================================================================
    // 内部存储资源 (使用分布式 RAM / Registers)
    // =========================================================================
     (* ram_style = "distributed" *)
    reg [$clog2(BATCH_SIZE)-1:0]  count_mem [0:NUM_NODES-1];  // 最大计数 1023
    reg [DIST_WIDTH-1:0]          dist_mem  [0:NUM_NODES-1];  // T = 8 时，最大是 7，但是这里只用到 3
    reg [T-1:0]        prefix_candidate_d1 [0:NUM_NODES-1]; 
    reg [T-1:0]        prefix_candidate_d2 [0:NUM_NODES-1]; 
    reg [T-1:0]        prefix_candidate_d3 [0:NUM_NODES-1]; 

    wire [T-1:0] hamming_rom [0:255];

    // =========================================================================
    // 状态机定义
    // =========================================================================
    localparam IDLE     = 3'd0;
    localparam LOAD     = 3'd1;
    localparam FORWARD  = 3'd2;
    localparam BACKWARD = 3'd3;
    localparam WRITE_SI = 3'd4;    
    localparam CLEAR    = 3'd5;
    localparam DONE     = 3'd6;

    reg [T:0]   ptr;      // 遍历指针 0-255
    integer     k;
    integer     i;

    reg [T-1:0]          curr_idx;
    reg [T-1:0]          suffix;
    reg [T-1:0]          selected_p;
    reg [DIST_WIDTH-1:0] temp_dis;
    reg [DIST_WIDTH-1:0] next_dis;

    reg [2:0]   state;
    reg [$clog2(BATCH_SIZE)-1:0]   load_cnt;
    reg [2:0]                      done_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= CLEAR;
            in_ready     <= 1'b0;
            ptr          <= 1'b0;
            si_update_en <= 1'b0;
            clear_done   <= 1'b0;
            load_cnt     <= 1'b0;
            done_cnt     <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    clear_done <= 1'b0;
                    in_ready   <= 1'b1;
                    if (start_load) begin
                         state    <= LOAD;
                         load_cnt <= 1'b0;
                    end
                end

                LOAD: begin
                    in_ready   <= 1'b0;
                    if (in_valid) begin
                        if (load_cnt == BATCH_SIZE - 1) begin
                            state    <= FORWARD;
                            load_cnt <= 1'b0;
                            ptr      <= 1'b0;
                        end else begin
                            count_mem[in_node] <= count_mem[in_node] + 1;
                            load_cnt <= load_cnt + 1'b1;
                        end
                    end
                end

                // --- Forward Pass (Alg 1) ---
                FORWARD: begin
                    if (ptr == NUM_NODES) begin
                        state <= BACKWARD;
                        ptr   <= NUM_NODES-1; // 准备反向遍历
                    end else begin
                        curr_idx = hamming_rom[ptr[T-1:0]];
                        // 确定当前节点的源头距离
                        if (count_mem[curr_idx] > 0 || curr_idx == 0)  temp_dis = 0;
                        else                                           temp_dis = dist_mem[curr_idx];

                        if(temp_dis < 3) begin
                            next_dis = temp_dis + 1;
                            for (k=0; k<T; k=k+1) begin
                                if (!(curr_idx & (1 << k))) begin // 检查第 k 位是否是 0
                                    suffix = curr_idx | (1 << k); // 第 k 位是 0，则置为 1
                                    // 更新后缀节点的距离和对应候选
                                    if (dist_mem[suffix] > next_dis) begin                // 发现距离更小的 prefix 
                                        dist_mem[suffix] <= next_dis;                     // 更新距离
                                    end
                                    // 无论是否是最短，记录当前路径作为该距离下的候选
                                    case (next_dis)                                    
                                            3'd1: prefix_candidate_d1[suffix] <= curr_idx;
                                            3'd2: prefix_candidate_d2[suffix] <= curr_idx;
                                            3'd3: prefix_candidate_d3[suffix] <= curr_idx;
                                    endcase             
                                end
                            end
                        end
                        ptr <= ptr + 1;
                    end 
                end

                // --- Stage 3: Backward Pass (Alg 2) ---
                // 确保被重用的 Prefix 自身在 Count 中被标记，保证其计算不被跳过
                BACKWARD: begin
                    curr_idx = hamming_rom[ptr[T-1:0]];
                    // 如果节点存在且不是根节点，需要向上寻找前缀并“激活”它
                    if (count_mem[curr_idx] > 0 && dist_mem[curr_idx] > 0 && dist_mem[curr_idx] <= 3) begin
                        case (dist_mem[curr_idx])
                            3'd1: selected_p = prefix_candidate_d1[curr_idx];
                            3'd2: selected_p = prefix_candidate_d2[curr_idx];
                            3'd3: selected_p = prefix_candidate_d3[curr_idx];
                            default: selected_p = curr_idx;
                        endcase
                        
                        // 激活前缀节点：即使它原本不在数据中，也要让它参与计算
                        if (count_mem[selected_p] == 0) begin
                            count_mem[selected_p] <= 1; 
                        end
                    end

                    if (ptr == 0) begin
                        state <= WRITE_SI;
                        ptr   <= 0;
                    end else begin
                        ptr <= ptr - 1;
                    end
                end

                // --- Stage 4: Write Result to SI RAM ---
                WRITE_SI: begin
                    if (ptr == NUM_NODES) begin
                        state        <= CLEAR;
                        si_update_en <= 1'b0;
                    end else begin
                        if (count_mem[ptr[T-1:0]] > 0) begin
                            si_update_en <= 1'b1;
                            si_addr      <= ptr[T-1:0];
                            // 确定 Prefix：根节点或不可重用节点前缀设为0
                            if (dist_mem[ptr[T-1:0]] == 0 || dist_mem[ptr[T-1:0]] > 3) begin
                                // 根节点或距离太远无法重用的节点，前缀设为 0（表示不重用）
                                si_prefix <= {T{1'b0}};
                            end else begin
                                // 写入距离最小的那个前缀
                                case (dist_mem[ptr])
                                    3'd1: si_prefix <= prefix_candidate_d1[ptr];
                                    3'd2: si_prefix <= prefix_candidate_d2[ptr];
                                    3'd3: si_prefix <= prefix_candidate_d3[ptr];
                                    default: si_prefix <= {T{1'b0}};
                                endcase
                            end
                        end 
                        else begin
                            si_update_en <= 1'b0;
                        end
                        ptr <= ptr + 1;
                    end
                end

                CLEAR: begin
                    count_mem[ptr[T-1:0]]    <= {(T+1){1'b0}};
                    dist_mem[ptr[T-1:0]]     <= {DIST_WIDTH{1'b1}};  // 设置为最大值
                    prefix_candidate_d1[ptr] <= {T{1'b0}};
                    prefix_candidate_d2[ptr] <= {T{1'b0}};
                    prefix_candidate_d3[ptr] <= {T{1'b0}};
                    if (ptr == NUM_NODES-1) begin
                          // 清理完毕
                          state      <= DONE; 
                          clear_done <= 1'b1;
                          ptr        <= 1'b0;
                          done_cnt   <= 1'b0;
                    end else begin
                          ptr <= ptr +1;
                    end
                end

                DONE: begin
                    clear_done     <= 1'b1;
                    // 留出间隙
                    if (done_cnt == 2'b11) begin
                        state    <= IDLE;
                        done_cnt <= 1'b0;
                    end else begin
                        done_cnt <= done_cnt + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    assign hamming_rom[0] = 8'd0;
    assign hamming_rom[1] = 8'd1;
    assign hamming_rom[2] = 8'd2;
    assign hamming_rom[3] = 8'd4;
    assign hamming_rom[4] = 8'd8;
    assign hamming_rom[5] = 8'd16;
    assign hamming_rom[6] = 8'd32;
    assign hamming_rom[7] = 8'd64;
    assign hamming_rom[8] = 8'd128;
    assign hamming_rom[9] = 8'd3;
    assign hamming_rom[10] = 8'd5;
    assign hamming_rom[11] = 8'd6;
    assign hamming_rom[12] = 8'd9;
    assign hamming_rom[13] = 8'd10;
    assign hamming_rom[14] = 8'd12;
    assign hamming_rom[15] = 8'd17;
    assign hamming_rom[16] = 8'd18;
    assign hamming_rom[17] = 8'd20;
    assign hamming_rom[18] = 8'd24;
    assign hamming_rom[19] = 8'd33;
    assign hamming_rom[20] = 8'd34;
    assign hamming_rom[21] = 8'd36;
    assign hamming_rom[22] = 8'd40;
    assign hamming_rom[23] = 8'd48;
    assign hamming_rom[24] = 8'd65;
    assign hamming_rom[25] = 8'd66;
    assign hamming_rom[26] = 8'd68;
    assign hamming_rom[27] = 8'd72;
    assign hamming_rom[28] = 8'd80;
    assign hamming_rom[29] = 8'd96;
    assign hamming_rom[30] = 8'd129;
    assign hamming_rom[31] = 8'd130;
    assign hamming_rom[32] = 8'd132;
    assign hamming_rom[33] = 8'd136;
    assign hamming_rom[34] = 8'd144;
    assign hamming_rom[35] = 8'd160;
    assign hamming_rom[36] = 8'd192;
    assign hamming_rom[37] = 8'd7;
    assign hamming_rom[38] = 8'd11;
    assign hamming_rom[39] = 8'd13;
    assign hamming_rom[40] = 8'd14;
    assign hamming_rom[41] = 8'd19;
    assign hamming_rom[42] = 8'd21;
    assign hamming_rom[43] = 8'd22;
    assign hamming_rom[44] = 8'd25;
    assign hamming_rom[45] = 8'd26;
    assign hamming_rom[46] = 8'd28;
    assign hamming_rom[47] = 8'd35;
    assign hamming_rom[48] = 8'd37;
    assign hamming_rom[49] = 8'd38;
    assign hamming_rom[50] = 8'd41;
    assign hamming_rom[51] = 8'd42;
    assign hamming_rom[52] = 8'd44;
    assign hamming_rom[53] = 8'd49;
    assign hamming_rom[54] = 8'd50;
    assign hamming_rom[55] = 8'd52;
    assign hamming_rom[56] = 8'd56;
    assign hamming_rom[57] = 8'd67;
    assign hamming_rom[58] = 8'd69;
    assign hamming_rom[59] = 8'd70;
    assign hamming_rom[60] = 8'd73;
    assign hamming_rom[61] = 8'd74;
    assign hamming_rom[62] = 8'd76;
    assign hamming_rom[63] = 8'd81;
    assign hamming_rom[64] = 8'd82;
    assign hamming_rom[65] = 8'd84;
    assign hamming_rom[66] = 8'd88;
    assign hamming_rom[67] = 8'd97;
    assign hamming_rom[68] = 8'd98;
    assign hamming_rom[69] = 8'd100;
    assign hamming_rom[70] = 8'd104;
    assign hamming_rom[71] = 8'd112;
    assign hamming_rom[72] = 8'd131;
    assign hamming_rom[73] = 8'd133;
    assign hamming_rom[74] = 8'd134;
    assign hamming_rom[75] = 8'd137;
    assign hamming_rom[76] = 8'd138;
    assign hamming_rom[77] = 8'd140;
    assign hamming_rom[78] = 8'd145;
    assign hamming_rom[79] = 8'd146;
    assign hamming_rom[80] = 8'd148;
    assign hamming_rom[81] = 8'd152;
    assign hamming_rom[82] = 8'd161;
    assign hamming_rom[83] = 8'd162;
    assign hamming_rom[84] = 8'd164;
    assign hamming_rom[85] = 8'd168;
    assign hamming_rom[86] = 8'd176;
    assign hamming_rom[87] = 8'd193;
    assign hamming_rom[88] = 8'd194;
    assign hamming_rom[89] = 8'd196;
    assign hamming_rom[90] = 8'd200;
    assign hamming_rom[91] = 8'd208;
    assign hamming_rom[92] = 8'd224;
    assign hamming_rom[93] = 8'd15;
    assign hamming_rom[94] = 8'd23;
    assign hamming_rom[95] = 8'd27;
    assign hamming_rom[96] = 8'd29;
    assign hamming_rom[97] = 8'd30;
    assign hamming_rom[98] = 8'd39;
    assign hamming_rom[99] = 8'd43;
    assign hamming_rom[100] = 8'd45;
    assign hamming_rom[101] = 8'd46;
    assign hamming_rom[102] = 8'd51;
    assign hamming_rom[103] = 8'd53;
    assign hamming_rom[104] = 8'd54;
    assign hamming_rom[105] = 8'd57;
    assign hamming_rom[106] = 8'd58;
    assign hamming_rom[107] = 8'd60;
    assign hamming_rom[108] = 8'd71;
    assign hamming_rom[109] = 8'd75;
    assign hamming_rom[110] = 8'd77;
    assign hamming_rom[111] = 8'd78;
    assign hamming_rom[112] = 8'd83;
    assign hamming_rom[113] = 8'd85;
    assign hamming_rom[114] = 8'd86;
    assign hamming_rom[115] = 8'd89;
    assign hamming_rom[116] = 8'd90;
    assign hamming_rom[117] = 8'd92;
    assign hamming_rom[118] = 8'd99;
    assign hamming_rom[119] = 8'd101;
    assign hamming_rom[120] = 8'd102;
    assign hamming_rom[121] = 8'd105;
    assign hamming_rom[122] = 8'd106;
    assign hamming_rom[123] = 8'd108;
    assign hamming_rom[124] = 8'd113;
    assign hamming_rom[125] = 8'd114;
    assign hamming_rom[126] = 8'd116;
    assign hamming_rom[127] = 8'd120;
    assign hamming_rom[128] = 8'd135;
    assign hamming_rom[129] = 8'd139;
    assign hamming_rom[130] = 8'd141;
    assign hamming_rom[131] = 8'd142;
    assign hamming_rom[132] = 8'd147;
    assign hamming_rom[133] = 8'd149;
    assign hamming_rom[134] = 8'd150;
    assign hamming_rom[135] = 8'd153;
    assign hamming_rom[136] = 8'd154;
    assign hamming_rom[137] = 8'd156;
    assign hamming_rom[138] = 8'd163;
    assign hamming_rom[139] = 8'd165;
    assign hamming_rom[140] = 8'd166;
    assign hamming_rom[141] = 8'd169;
    assign hamming_rom[142] = 8'd170;
    assign hamming_rom[143] = 8'd172;
    assign hamming_rom[144] = 8'd177;
    assign hamming_rom[145] = 8'd178;
    assign hamming_rom[146] = 8'd180;
    assign hamming_rom[147] = 8'd184;
    assign hamming_rom[148] = 8'd195;
    assign hamming_rom[149] = 8'd197;
    assign hamming_rom[150] = 8'd198;
    assign hamming_rom[151] = 8'd201;
    assign hamming_rom[152] = 8'd202;
    assign hamming_rom[153] = 8'd204;
    assign hamming_rom[154] = 8'd209;
    assign hamming_rom[155] = 8'd210;
    assign hamming_rom[156] = 8'd212;
    assign hamming_rom[157] = 8'd216;
    assign hamming_rom[158] = 8'd225;
    assign hamming_rom[159] = 8'd226;
    assign hamming_rom[160] = 8'd228;
    assign hamming_rom[161] = 8'd232;
    assign hamming_rom[162] = 8'd240;
    assign hamming_rom[163] = 8'd31;
    assign hamming_rom[164] = 8'd47;
    assign hamming_rom[165] = 8'd55;
    assign hamming_rom[166] = 8'd59;
    assign hamming_rom[167] = 8'd61;
    assign hamming_rom[168] = 8'd62;
    assign hamming_rom[169] = 8'd79;
    assign hamming_rom[170] = 8'd87;
    assign hamming_rom[171] = 8'd91;
    assign hamming_rom[172] = 8'd93;
    assign hamming_rom[173] = 8'd94;
    assign hamming_rom[174] = 8'd103;
    assign hamming_rom[175] = 8'd107;
    assign hamming_rom[176] = 8'd109;
    assign hamming_rom[177] = 8'd110;
    assign hamming_rom[178] = 8'd115;
    assign hamming_rom[179] = 8'd117;
    assign hamming_rom[180] = 8'd118;
    assign hamming_rom[181] = 8'd121;
    assign hamming_rom[182] = 8'd122;
    assign hamming_rom[183] = 8'd124;
    assign hamming_rom[184] = 8'd143;
    assign hamming_rom[185] = 8'd151;
    assign hamming_rom[186] = 8'd155;
    assign hamming_rom[187] = 8'd157;
    assign hamming_rom[188] = 8'd158;
    assign hamming_rom[189] = 8'd167;
    assign hamming_rom[190] = 8'd171;
    assign hamming_rom[191] = 8'd173;
    assign hamming_rom[192] = 8'd174;
    assign hamming_rom[193] = 8'd179;
    assign hamming_rom[194] = 8'd181;
    assign hamming_rom[195] = 8'd182;
    assign hamming_rom[196] = 8'd185;
    assign hamming_rom[197] = 8'd186;
    assign hamming_rom[198] = 8'd188;
    assign hamming_rom[199] = 8'd199;
    assign hamming_rom[200] = 8'd203;
    assign hamming_rom[201] = 8'd205;
    assign hamming_rom[202] = 8'd206;
    assign hamming_rom[203] = 8'd211;
    assign hamming_rom[204] = 8'd213;
    assign hamming_rom[205] = 8'd214;
    assign hamming_rom[206] = 8'd217;
    assign hamming_rom[207] = 8'd218;
    assign hamming_rom[208] = 8'd220;
    assign hamming_rom[209] = 8'd227;
    assign hamming_rom[210] = 8'd229;
    assign hamming_rom[211] = 8'd230;
    assign hamming_rom[212] = 8'd233;
    assign hamming_rom[213] = 8'd234;
    assign hamming_rom[214] = 8'd236;
    assign hamming_rom[215] = 8'd241;
    assign hamming_rom[216] = 8'd242;
    assign hamming_rom[217] = 8'd244;
    assign hamming_rom[218] = 8'd248;
    assign hamming_rom[219] = 8'd63;
    assign hamming_rom[220] = 8'd95;
    assign hamming_rom[221] = 8'd111;
    assign hamming_rom[222] = 8'd119;
    assign hamming_rom[223] = 8'd123;
    assign hamming_rom[224] = 8'd125;
    assign hamming_rom[225] = 8'd126;
    assign hamming_rom[226] = 8'd159;
    assign hamming_rom[227] = 8'd175;
    assign hamming_rom[228] = 8'd183;
    assign hamming_rom[229] = 8'd187;
    assign hamming_rom[230] = 8'd189;
    assign hamming_rom[231] = 8'd190;
    assign hamming_rom[232] = 8'd207;
    assign hamming_rom[233] = 8'd215;
    assign hamming_rom[234] = 8'd219;
    assign hamming_rom[235] = 8'd221;
    assign hamming_rom[236] = 8'd222;
    assign hamming_rom[237] = 8'd231;
    assign hamming_rom[238] = 8'd235;
    assign hamming_rom[239] = 8'd237;
    assign hamming_rom[240] = 8'd238;
    assign hamming_rom[241] = 8'd243;
    assign hamming_rom[242] = 8'd245;
    assign hamming_rom[243] = 8'd246;
    assign hamming_rom[244] = 8'd249;
    assign hamming_rom[245] = 8'd250;
    assign hamming_rom[246] = 8'd252;
    assign hamming_rom[247] = 8'd127;
    assign hamming_rom[248] = 8'd191;
    assign hamming_rom[249] = 8'd223;
    assign hamming_rom[250] = 8'd239;
    assign hamming_rom[251] = 8'd247;
    assign hamming_rom[252] = 8'd251;
    assign hamming_rom[253] = 8'd253;
    assign hamming_rom[254] = 8'd254;
    assign hamming_rom[255] = 8'd255;

endmodule