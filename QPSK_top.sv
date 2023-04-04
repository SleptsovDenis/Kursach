
module QPSK_top(
input clock,
input reset,
input logic random_bit,
output logic dout
);

logic        uart_2_mod;
logic [31:0] mod_2_demod;
logic [31:0] demod_2_uart;



uart_rx uart_rx (
    .s_tick(clock),
    .reset(reset),
    .rx(random_bit),
    .dout(uart_2_mod)
);

modulator_qpsk mod(
.clock(clock),
.reset(reset),
.random_bit(uart_2_mod/*random_bit*/),
.data(mod_2_demod)
);

demodulator_qpsk demod(
.clock(clock),
.reset(reset),
.data(mod_2_demod),
.bipolar(demod_2_uart/*dout*/)
);

uart_tx uart_tx(
    .clk(clock),
    .start(reset),
    .data({ 7'b0,demod_2_uart}),
    .q(dout)
);

endmodule

module uart_rx(
input rx,
input s_tick,
input reset,
output reg dout
);

reg [2:0] state;
reg [3:0] bit_count;
reg [7:0] data_reg;

parameter IDLE = 3'b000,
START = 3'b001,
DATA = 3'b010,
PARITY = 3'b011,
STOP = 3'b100;

always @(posedge s_tick) begin
  if (reset) begin
    state <= IDLE;
    bit_count <= 0;
    data_reg <= 0;
  end else begin
    case (state)
    IDLE: begin
      //if (rx == 0) begin
        state <= START;
        bit_count <= 0;
        data_reg <= rx;
     // end 
    end
    START: begin
      if (bit_count < 8) begin
          bit_count <= bit_count + 1;
          data_reg[bit_count] <= rx;
        end else begin
          state <= PARITY;
          bit_count <= 0;
      end
    end
    PARITY: begin
      state <= DATA;
      bit_count <= 0;
    end
    DATA: begin
      if (bit_count < 8) begin
          bit_count <= bit_count + 1;
          data_reg[bit_count] <= rx;
        end else begin
          state <= STOP;
          bit_count <= 0;
          dout <= data_reg;
      end
    end
    STOP: begin
        if (bit_count < 2) begin
          bit_count <= bit_count + 1;
        end else begin
          state <= IDLE;
          bit_count <= 0;
        end
      end
    default: begin
      state <= IDLE;
      bit_count <= 0;
    end
    endcase
  end
end

endmodule

module uart_tx (
    input clk,
    input start,
    input data,
    output reg q = 1'b0
);

reg [12:0]cnt = 13'b0;
reg [3:0]bit_num = 4'b0000;
wire bit_start = (cnt == /*5208*/1);
wire idle = (bit_num == 4'hF);

always @(posedge clk) begin
    if (start || idle) begin
        bit_num <= 4'h0;
        q <= 1'b0; // Start
        cnt <= 13'b0;
 end
    else if (bit_start) begin
        case (bit_num)
        4'h0: begin bit_num <= 4'h1; q <= data; end
        4'h1: begin bit_num <= 4'h2; q <= data; end
        4'h2: begin bit_num <= 4'h3; q <= data; end
        4'h3: begin bit_num <= 4'h4; q <= data; end
        4'h4: begin bit_num <= 4'h5; q <= data; end
        4'h5: begin bit_num <= 4'h6; q <= data; end
        4'h6: begin bit_num <= 4'h7; q <= data; end
        4'h7: begin bit_num <= 4'h0; q <= data; end
//        4'h8: begin bit_num <= 4'hF; q <= data; end // Stop!!! WTF tam bil???
        default: begin bit_num <= 4'hF; end
        endcase
        cnt <= 13'b0;
 end
    else
        cnt <= cnt + 13'b1;

end

/*always @(posedge clk) begin
    if (start && idle) begin
        bit_num <= 4'h0;
        q <= 1'b0; // Start
    end
    else if (bit_start) begin
        case (bit_num)
        4'h0: begin bit_num <= 4'h1; q <= data; end
        4'h1: begin bit_num <= 4'h2; q <= data; end
        4'h2: begin bit_num <= 4'h3; q <= data; end
        4'h3: begin bit_num <= 4'h4; q <= data; end
        4'h4: begin bit_num <= 4'h5; q <= data; end
        4'h5: begin bit_num <= 4'h6; q <= data; end
        4'h6: begin bit_num <= 4'h7; q <= data; end
        4'h7: begin bit_num <= 4'h8; q <= data; end
        4'h8: begin bit_num <= 4'h9; q <= 1'b1; end // Stop
        default: begin bit_num <= 4'hF; end
        endcase
    end
end*/

endmodule
module modulator_qpsk(
  input logic random_bit,
  input clock,
  input reset,
  output signed [31:0] data
);
 
logic clock_signal;
int regulating_the_frequency_counter = 0;
 
always @(posedge clock or posedge reset)
  begin
     if (reset)
        begin
            regulating_the_frequency_counter <= 0;
            clock_signal <= 1'b0;
        end
     else
        begin
          if (regulating_the_frequency_counter==87)
            begin
                clock_signal <= ~ clock_signal;
                regulating_the_frequency_counter = 0;
            end
            regulating_the_frequency_counter <= regulating_the_frequency_counter + 1;
        end
  end

logic clock_signal_div_two = '1;
    
always @(posedge clock_signal or posedge reset)
  begin
    if(reset)
        clock_signal_div_two <= '1;
    else
        clock_signal_div_two <= ~clock_signal_div_two;    
  end 

logic [31:0] bipolar_signal;
logic [31:0] bipolar_signal_old;
logic [31:0] bipolar_pair_first_member;
logic [31:0] bipolar_pair_second_member;
logic [31:0] odd;
logic [31:0] even;
logic [31:0] stream_1;
logic [31:0] stream_2;
logic signed [31:0] sinus;
logic signed [31:0] cosinus;
 
unipolar_to_bipolar_converter utbc
(
 .random_bit(random_bit),
 .clock_signal(clock_signal),
 .reset(reset),
 .bipolar_signal(bipolar_signal),
 .bipolar_signal_old(bipolar_signal_old)
);

 
/*buffer b
(
 .bipolar_signal(bipolar_signal),
 .bipolar_signal_old(bipolar_signal_old),
 .clock_signal(clock_signal),
 .reset(reset),
 .bipolar_pair_first_member(bipolar_pair_first_member),
 .bipolar_pair_second_member(bipolar_pair_second_member)
);*/
 
deinterlacer d
(
 .bipolar_pair_first_member(bipolar_signal_old),
 .bipolar_pair_second_member(bipolar_signal),
 .clock_signal(clock_signal_div_two),
 .reset(reset),
 .odd(odd),
 .even(even)
);
 
sinus_wave sw
(
 .clock(clock),
 .reset(reset),
 .sinus(sinus)
);
 
cosinus_wave cw
(
 .clock(clock),
 .reset(reset),
 .cosinus(cosinus)
);
 
multiplier_and_summator mas
(
 .stream_1(odd),
 .stream_2(even),
 .sinus(sinus),
 .cosinus(cosinus),
 .clock(clock),
 .reset(reset),
 .data(data)
);
 
endmodule

module unipolar_to_bipolar_converter(
 input logic random_bit,
 input clock_signal,
 input reset,
 output logic signed  [31:0] bipolar_signal,
 output logic signed  [31:0] bipolar_signal_old
);

logic signed [31:0] b_s [2];

initial begin
 b_s [1] = 0;
 b_s [0] = 0;
end
 
always @(posedge clock_signal or posedge reset)
  begin
     if (reset)
 begin 
  bipolar_signal <= 0;
  bipolar_signal_old <= 0;
 end
     else 
 begin 
   if (random_bit == 1) 
     begin
         b_s [0] <= 32'b1;
         b_s [1] <= b_s [0];
       bipolar_signal_old <= b_s[1];
       bipolar_signal <= b_s[0];
     end
       else
     begin
         b_s [0] <= -32'b1;
         b_s [1] <= b_s [0];
      bipolar_signal_old <= b_s[1];
       bipolar_signal <= b_s[0];
    end
       end
  end


 
endmodule

module deinterlacer(
 input logic signed [31:0]  bipolar_pair_first_member,
 input logic signed [31:0] bipolar_pair_second_member,  
 input clock_signal,
 input reset,
 output logic signed [31:0]  odd,
 output logic signed [31:0]  even
);
 
always @(posedge clock_signal or posedge reset)
  begin
    if(reset)
       begin
         odd <= 0;    
         even <= 0;    
        end
    else
       begin
         odd <= bipolar_pair_first_member;    
         even <= bipolar_pair_second_member;    
        end
  end
 
endmodule


module cosinus_wave(
    input clock,
    input reset,
    output signed [31:0] cosinus
);
 
reg [7:0] phase_inc = 8'h9;
reg [9:0] phase_acc = 0;

parameter LUT_SIZE = 256;
reg [9:0] lut [0:LUT_SIZE-1];
 
always @(posedge clock or posedge reset) begin
    if (reset)
      begin
        phase_inc = 8'h9;
	phase_acc = 0;
	cosinus = 0;
	lut[0] = 10'h00;
	lut[1] = 10'h06;
	lut[2] = 10'h0C;
	lut[3] = 10'h12;
	lut[4] = 10'h19;
	lut[5] = 10'h1F;
	lut[6] = 10'h25;
	lut[7] = 10'h2C;
	lut[8] = 10'h32;
	lut[9] = 10'h38;
	lut[10] = 10'h3E;
	lut[11] = 10'h45;
	lut[12] = 10'h4B;
	lut[13] = 10'h51;
	lut[14] = 10'h58;
	lut[15] = 10'h5E;
	lut[16] = 10'h64;
	lut[17] = 10'h6A;
	lut[18] = 10'h71;
	lut[19] = 10'h77;
	lut[20] = 10'h7D;
	lut[21] = 10'h83;
	lut[22] = 10'h8A;
	lut[23] = 10'h90;
	lut[24] = 10'h96;
	lut[25] = 10'h9C;
	lut[26] = 10'hA3;
	lut[27] = 10'hA9;
	lut[28] = 10'hAF;
	lut[29] = 10'hB5;
	lut[30] = 10'hBB;
	lut[31] = 10'hC2;
	lut[32] = 10'hC8;
	lut[33] = 10'hCE;
	lut[34] = 10'hD4;
	lut[35] = 10'hDA;
	lut[36] = 10'hE1;
	lut[37] = 10'hE7;
	lut[38] = 10'hED;
	lut[39] = 10'hF3;
	lut[40] = 10'hF9;
	lut[41] = 10'hFF;
	lut[42] = 10'h105;
	lut[43] = 10'h10B;
	lut[44] = 10'h111;
	lut[45] = 10'h117;
	lut[46] = 10'h11E;
	lut[47] = 10'h124;
	lut[48] = 10'h12A;
	lut[49] = 10'h130;
	lut[50] = 10'h136;
	lut[51] = 10'h13C;
	lut[52] = 10'h142;
	lut[53] = 10'h148;
	lut[54] = 10'h14E;
	lut[55] = 10'h153;
	lut[56] = 10'h159;
	lut[57] = 10'h15F;
	lut[58] = 10'h165;
	lut[59] = 10'h16B;
	lut[60] = 10'h171;
	lut[61] = 10'h177;
	lut[62] = 10'h17D;
	lut[63] = 10'h183;
	lut[64] = 10'h188;
	lut[65] = 10'h18E;
	lut[66] = 10'h194;
	lut[67] = 10'h19A;
	lut[68] = 10'h1A0;
	lut[69] = 10'h1A5;
	lut[70] = 10'h1AB;
	lut[71] = 10'h1B1;
	lut[72] = 10'h1B6;
	lut[73] = 10'h1BC;
	lut[74] = 10'h1C2;
	lut[75] = 10'h1C7;
	lut[76] = 10'h1CD;
	lut[77] = 10'h1D3;
	lut[78] = 10'h1D8;
	lut[79] = 10'h1DE;
	lut[80] = 10'h1E3;
	lut[81] = 10'h1E9;
	lut[82] = 10'h1EF;
	lut[83] = 10'h1F4;
	lut[84] = 10'h1FA;
	lut[85] = 10'h1FF;
	lut[86] = 10'h204;
	lut[87] = 10'h20A;
	lut[88] = 10'h20F;
	lut[89] = 10'h215;
	lut[90] = 10'h21A;
	lut[91] = 10'h21F;
	lut[92] = 10'h225;
	lut[93] = 10'h22A;
	lut[94] = 10'h22F;
	lut[95] = 10'h235;
	lut[96] = 10'h23A;
	lut[97] = 10'h23F;
	lut[98] = 10'h244;
	lut[99] = 10'h249;
	lut[100] = 10'h24F;
	lut[101] = 10'h254;
	lut[102] = 10'h259;
	lut[103] = 10'h25E;
	lut[104] = 10'h263;
	lut[105] = 10'h268;
	lut[106] = 10'h26D;
	lut[107] = 10'h272;
	lut[108] = 10'h277;
	lut[109] = 10'h27C;
	lut[110] = 10'h281;
	lut[111] = 10'h286;
	lut[112] = 10'h28B;
	lut[113] = 10'h28F;
	lut[114] = 10'h294;
	lut[115] = 10'h299;
	lut[116] = 10'h29E;
	lut[117] = 10'h2A3;
	lut[118] = 10'h2A7;
	lut[119] = 10'h2AC;
	lut[120] = 10'h2B1;
	lut[121] = 10'h2B5;
	lut[122] = 10'h2BA;
	lut[123] = 10'h2BF;
	lut[124] = 10'h2C3;
	lut[125] = 10'h2C8;
	lut[126] = 10'h2CC;
	lut[127] = 10'h2D1;
	lut[128] = 10'h2D5;
	lut[129] = 10'h2DA;
	lut[130] = 10'h2DE;
	lut[131] = 10'h2E2;
	lut[132] = 10'h2E7;
	lut[133] = 10'h2EB;
	lut[134] = 10'h2EF;
	lut[135] = 10'h2F4;
	lut[136] = 10'h2F8;
	lut[137] = 10'h2FC;
	lut[138] = 10'h300;
	lut[139] = 10'h304;
	lut[140] = 10'h308;
	lut[141] = 10'h30C;
	lut[142] = 10'h311;
	lut[143] = 10'h315;
	lut[144] = 10'h319;
	lut[145] = 10'h31C;
	lut[146] = 10'h320;
	lut[147] = 10'h324;
	lut[148] = 10'h328;
	lut[149] = 10'h32C;
	lut[150] = 10'h330;
	lut[151] = 10'h334;
	lut[152] = 10'h337;
	lut[153] = 10'h33B;
	lut[154] = 10'h33F;
	lut[155] = 10'h342;
	lut[156] = 10'h346;
	lut[157] = 10'h34A;
	lut[158] = 10'h34D;
	lut[159] = 10'h351;
	lut[160] = 10'h354;
	lut[161] = 10'h358;
	lut[162] = 10'h35B;
	lut[163] = 10'h35F;
	lut[164] = 10'h362;
	lut[165] = 10'h365;
	lut[166] = 10'h369;
	lut[167] = 10'h36C;
	lut[168] = 10'h36F;
	lut[169] = 10'h372;
	lut[170] = 10'h375;
	lut[171] = 10'h379;
	lut[172] = 10'h37C;
	lut[173] = 10'h37F;
	lut[174] = 10'h382;
	lut[175] = 10'h385;
	lut[176] = 10'h388;
	lut[177] = 10'h38B;
	lut[178] = 10'h38E;
	lut[179] = 10'h390;
	lut[180] = 10'h393;
	lut[181] = 10'h396;
	lut[182] = 10'h399;
	lut[183] = 10'h39C;
	lut[184] = 10'h39E;
	lut[185] = 10'h3A1;
	lut[186] = 10'h3A3;
	lut[187] = 10'h3A6;
	lut[188] = 10'h3A9;
	lut[189] = 10'h3AB;
	lut[190] = 10'h3AE;
	lut[191] = 10'h3B0;
	lut[192] = 10'h3B2;
	lut[193] = 10'h3B5;
	lut[194] = 10'h3B7;
	lut[195] = 10'h3B9;
	lut[196] = 10'h3BC;
	lut[197] = 10'h3BE;
	lut[198] = 10'h3C0;
	lut[199] = 10'h3C2;
	lut[200] = 10'h3C4;
	lut[201] = 10'h3C6;
	lut[202] = 10'h3C8;
	lut[203] = 10'h3CA;
	lut[204] = 10'h3CC;
	lut[205] = 10'h3CE;
	lut[206] = 10'h3D0;
	lut[207] = 10'h3D2;
	lut[208] = 10'h3D4;
	lut[209] = 10'h3D6;
	lut[210] = 10'h3D7;
	lut[211] = 10'h3D9;
	lut[212] = 10'h3DB;
	lut[213] = 10'h3DC;
	lut[214] = 10'h3DE;
	lut[215] = 10'h3E0;
	lut[216] = 10'h3E1;
	lut[217] = 10'h3E3;
	lut[218] = 10'h3E4;
	lut[219] = 10'h3E5;
	lut[220] = 10'h3E7;
	lut[221] = 10'h3E8;
	lut[222] = 10'h3E9;
	lut[223] = 10'h3EB;
	lut[224] = 10'h3EC;
	lut[225] = 10'h3ED;
	lut[226] = 10'h3EE;
	lut[227] = 10'h3EF;
	lut[228] = 10'h3F0;
	lut[229] = 10'h3F1;
	lut[230] = 10'h3F2;
	lut[231] = 10'h3F3;
	lut[232] = 10'h3F4;
	lut[233] = 10'h3F5;
	lut[234] = 10'h3F6;
	lut[235] = 10'h3F7;
	lut[236] = 10'h3F8;
	lut[237] = 10'h3F8;
	lut[238] = 10'h3F9;
	lut[239] = 10'h3FA;
	lut[240] = 10'h3FA;
	lut[241] = 10'h3FB;
	lut[242] = 10'h3FB;
	lut[243] = 10'h3FC;
	lut[244] = 10'h3FC;
	lut[245] = 10'h3FD;
	lut[246] = 10'h3FD;
	lut[247] = 10'h3FD;
	lut[248] = 10'h3FE;
	lut[249] = 10'h3FE;
	lut[250] = 10'h3FE;
	lut[251] = 10'h3FE;
	lut[252] = 10'h3FE;
	lut[253] = 10'h3FE;
	lut[254] = 10'h3FE;
	lut[255] = 10'h3FF;
     end
	else 
      begin
        if (phase_acc[9:8] == 2'b00)
           begin
            cosinus <= lut[~phase_acc[7:0]];
         end
        if (phase_acc[9:8] == 2'b01)
           begin
            cosinus <= (-1)*lut[phase_acc[7:0]];
         end
        if (phase_acc[9:8] == 2'b10)
           begin
            cosinus <= (-1)*lut[~phase_acc[7:0]];
         end
        if (phase_acc[9:8] == 2'b11)
           begin
            cosinus <= lut[phase_acc[7:0]];
         end
        phase_acc <= phase_acc + {2'b0,phase_inc};
     end
end
 
endmodule

module sinus_wave(
    input clock,
    input reset,
    output signed [31:0] sinus
);
  
logic [7:0] phase_inc;
logic [9:0] phase_acc;

parameter LUT_SIZE = 256;
logic [9:0] lut [0:LUT_SIZE-1];
 
always @(posedge clock or posedge reset) begin
    if (reset)
      begin
      phase_inc <= 8'h9;
      phase_acc = 0;
      sinus = 0;
      lut[0] = 10'h00;
      lut[1] = 10'h06;
      lut[2] = 10'h0C;
      lut[3] = 10'h12;
      lut[4] = 10'h19;
      lut[5] = 10'h1F;
      lut[6] = 10'h25;
      lut[7] = 10'h2C;
      lut[8] = 10'h32;
      lut[9] = 10'h38;
      lut[10] = 10'h3E;
      lut[11] = 10'h45;
      lut[12] = 10'h4B;
      lut[13] = 10'h51;
      lut[14] = 10'h58;
      lut[15] = 10'h5E;
      lut[16] = 10'h64;
      lut[17] = 10'h6A;
      lut[18] = 10'h71;
      lut[19] = 10'h77;
      lut[20] = 10'h7D;
      lut[21] = 10'h83;
      lut[22] = 10'h8A;
      lut[23] = 10'h90;
      lut[24] = 10'h96;
      lut[25] = 10'h9C;
      lut[26] = 10'hA3;
      lut[27] = 10'hA9;
      lut[28] = 10'hAF;
      lut[29] = 10'hB5;
      lut[30] = 10'hBB;
      lut[31] = 10'hC2;
      lut[32] = 10'hC8;
      lut[33] = 10'hCE;
      lut[34] = 10'hD4;
      lut[35] = 10'hDA;
      lut[36] = 10'hE1;
      lut[37] = 10'hE7;
      lut[38] = 10'hED;
      lut[39] = 10'hF3;
      lut[40] = 10'hF9;
      lut[41] = 10'hFF;
      lut[42] = 10'h105;
      lut[43] = 10'h10B;
      lut[44] = 10'h111;
      lut[45] = 10'h117;
      lut[46] = 10'h11E;
      lut[47] = 10'h124;
      lut[48] = 10'h12A;
      lut[49] = 10'h130;
      lut[50] = 10'h136;
      lut[51] = 10'h13C;
      lut[52] = 10'h142;
      lut[53] = 10'h148;
      lut[54] = 10'h14E;
      lut[55] = 10'h153;
      lut[56] = 10'h159;
      lut[57] = 10'h15F;
      lut[58] = 10'h165;
      lut[59] = 10'h16B;
      lut[60] = 10'h171;
      lut[61] = 10'h177;
      lut[62] = 10'h17D;
      lut[63] = 10'h183;
      lut[64] = 10'h188;
      lut[65] = 10'h18E;
      lut[66] = 10'h194;
      lut[67] = 10'h19A;
      lut[68] = 10'h1A0;
      lut[69] = 10'h1A5;
      lut[70] = 10'h1AB;
      lut[71] = 10'h1B1;
      lut[72] = 10'h1B6;
      lut[73] = 10'h1BC;
      lut[74] = 10'h1C2;
      lut[75] = 10'h1C7;
      lut[76] = 10'h1CD;
      lut[77] = 10'h1D3;
      lut[78] = 10'h1D8;
      lut[79] = 10'h1DE;
      lut[80] = 10'h1E3;
      lut[81] = 10'h1E9;
      lut[82] = 10'h1EF;
      lut[83] = 10'h1F4;
      lut[84] = 10'h1FA;
      lut[85] = 10'h1FF;
      lut[86] = 10'h204;
      lut[87] = 10'h20A;
      lut[88] = 10'h20F;
      lut[89] = 10'h215;
      lut[90] = 10'h21A;
      lut[91] = 10'h21F;
      lut[92] = 10'h225;
      lut[93] = 10'h22A;
      lut[94] = 10'h22F;
      lut[95] = 10'h235;
      lut[96] = 10'h23A;
      lut[97] = 10'h23F;
      lut[98] = 10'h244;
      lut[99] = 10'h249;
      lut[100] = 10'h24F;
      lut[101] = 10'h254;
      lut[102] = 10'h259;
      lut[103] = 10'h25E;
      lut[104] = 10'h263;
      lut[105] = 10'h268;
      lut[106] = 10'h26D;
      lut[107] = 10'h272;
      lut[108] = 10'h277;
      lut[109] = 10'h27C;
      lut[110] = 10'h281;
      lut[111] = 10'h286;
      lut[112] = 10'h28B;
      lut[113] = 10'h28F;
      lut[114] = 10'h294;
      lut[115] = 10'h299;
      lut[116] = 10'h29E;
      lut[117] = 10'h2A3;
      lut[118] = 10'h2A7;
      lut[119] = 10'h2AC;
      lut[120] = 10'h2B1;
      lut[121] = 10'h2B5;
      lut[122] = 10'h2BA;
      lut[123] = 10'h2BF;
      lut[124] = 10'h2C3;
      lut[125] = 10'h2C8;
      lut[126] = 10'h2CC;
      lut[127] = 10'h2D1;
      lut[128] = 10'h2D5;
      lut[129] = 10'h2DA;
      lut[130] = 10'h2DE;
      lut[131] = 10'h2E2;
      lut[132] = 10'h2E7;
      lut[133] = 10'h2EB;
      lut[134] = 10'h2EF;
      lut[135] = 10'h2F4;
      lut[136] = 10'h2F8;
      lut[137] = 10'h2FC;
      lut[138] = 10'h300;
      lut[139] = 10'h304;
      lut[140] = 10'h308;
      lut[141] = 10'h30C;
      lut[142] = 10'h311;
      lut[143] = 10'h315;
      lut[144] = 10'h319;
      lut[145] = 10'h31C;
      lut[146] = 10'h320;
      lut[147] = 10'h324;
      lut[148] = 10'h328;
      lut[149] = 10'h32C;
      lut[150] = 10'h330;
      lut[151] = 10'h334;
      lut[152] = 10'h337;
      lut[153] = 10'h33B;
      lut[154] = 10'h33F;
      lut[155] = 10'h342;
      lut[156] = 10'h346;
      lut[157] = 10'h34A;
      lut[158] = 10'h34D;
      lut[159] = 10'h351;
      lut[160] = 10'h354;
      lut[161] = 10'h358;
      lut[162] = 10'h35B;
      lut[163] = 10'h35F;
      lut[164] = 10'h362;
      lut[165] = 10'h365;
      lut[166] = 10'h369;
      lut[167] = 10'h36C;
      lut[168] = 10'h36F;
      lut[169] = 10'h372;
      lut[170] = 10'h375;
      lut[171] = 10'h379;
      lut[172] = 10'h37C;
      lut[173] = 10'h37F;
      lut[174] = 10'h382;
      lut[175] = 10'h385;
      lut[176] = 10'h388;
      lut[177] = 10'h38B;
      lut[178] = 10'h38E;
      lut[179] = 10'h390;
      lut[180] = 10'h393;
      lut[181] = 10'h396;
      lut[182] = 10'h399;
      lut[183] = 10'h39C;
      lut[184] = 10'h39E;
      lut[185] = 10'h3A1;
      lut[186] = 10'h3A3;
      lut[187] = 10'h3A6;
      lut[188] = 10'h3A9;
      lut[189] = 10'h3AB;
      lut[190] = 10'h3AE;
      lut[191] = 10'h3B0;
      lut[192] = 10'h3B2;
      lut[193] = 10'h3B5;
      lut[194] = 10'h3B7;
      lut[195] = 10'h3B9;
      lut[196] = 10'h3BC;
      lut[197] = 10'h3BE;
      lut[198] = 10'h3C0;
      lut[199] = 10'h3C2;
      lut[200] = 10'h3C4;
      lut[201] = 10'h3C6;
      lut[202] = 10'h3C8;
      lut[203] = 10'h3CA;
      lut[204] = 10'h3CC;
      lut[205] = 10'h3CE;
      lut[206] = 10'h3D0;
      lut[207] = 10'h3D2;
      lut[208] = 10'h3D4;
      lut[209] = 10'h3D6;
      lut[210] = 10'h3D7;
      lut[211] = 10'h3D9;
      lut[212] = 10'h3DB;
      lut[213] = 10'h3DC;
      lut[214] = 10'h3DE;
      lut[215] = 10'h3E0;
      lut[216] = 10'h3E1;
      lut[217] = 10'h3E3;
      lut[218] = 10'h3E4;
      lut[219] = 10'h3E5;
      lut[220] = 10'h3E7;
      lut[221] = 10'h3E8;
      lut[222] = 10'h3E9;
      lut[223] = 10'h3EB;
      lut[224] = 10'h3EC;
      lut[225] = 10'h3ED;
      lut[226] = 10'h3EE;
      lut[227] = 10'h3EF;
      lut[228] = 10'h3F0;
      lut[229] = 10'h3F1;
      lut[230] = 10'h3F2;
      lut[231] = 10'h3F3;
      lut[232] = 10'h3F4;
      lut[233] = 10'h3F5;
      lut[234] = 10'h3F6;
      lut[235] = 10'h3F7;
      lut[236] = 10'h3F8;
      lut[237] = 10'h3F8;
      lut[238] = 10'h3F9;
      lut[239] = 10'h3FA;
      lut[240] = 10'h3FA;
      lut[241] = 10'h3FB;
      lut[242] = 10'h3FB;
      lut[243] = 10'h3FC;
      lut[244] = 10'h3FC;
      lut[245] = 10'h3FD;
      lut[246] = 10'h3FD;
      lut[247] = 10'h3FD;
      lut[248] = 10'h3FE;
      lut[249] = 10'h3FE;
      lut[250] = 10'h3FE;
      lut[251] = 10'h3FE;
      lut[252] = 10'h3FE;
      lut[253] = 10'h3FE;
      lut[254] = 10'h3FE;
      lut[255] = 10'h3FF;
  end
	else 
   begin
        if (phase_acc[9:8] == 2'b00)
           begin
            sinus <= lut[phase_acc[7:0]];
         end
        if (phase_acc[9:8] == 2'b01)
           begin
            sinus <= lut[~phase_acc[7:0]];
         end
        if (phase_acc[9:8] == 2'b10)
           begin
            sinus <= (-1)*lut[phase_acc[7:0]];
         end
        if (phase_acc[9:8] == 2'b11)
           begin
            sinus <= (-1)*lut[~phase_acc[7:0]];
         end
        phase_acc <= phase_acc + {2'b0,phase_inc};
     end
end
 
endmodule
module multiplier_and_summator
(
 input logic signed [31:0]  stream_1,
 input logic signed [31:0]  stream_2,
 input logic signed [31:0] sinus,
 input logic signed [31:0] cosinus,
 input clock,
 input reset,
 output logic signed [31:0] data
);
 
always @(posedge clock or posedge reset)
  begin
    if(reset)
        data <= '0;
    else
        data <= stream_1*cosinus + stream_2*sinus;    
  end
 
endmodule

module demodulator_qpsk(
input clock,
input reset,
input [31:0] data, // input modulated signal
//output [31:0] unipolar_signal
//output logic [31:0] after_sin_data,
//output logic [31:0] after_cos_data,
output logic [31:0] bipolar

);

// clk 50 MHZ, clk 25 MHZ,, 
logic clock_signal;
logic [31:0] regulating_the_frequency_counter;
 
always @(posedge clock or posedge reset)
  begin
     if (reset)
        begin
            regulating_the_frequency_counter <= 0;
            clock_signal <= 1'b0;
        end
     else
        begin
          if (regulating_the_frequency_counter==87)
          begin
            clock_signal <= ~clock_signal;
            regulating_the_frequency_counter = 0;
          end
          regulating_the_frequency_counter <= regulating_the_frequency_counter + 1'b1;
        end
  end
//
//logic clock_signal_div_two = 1'b1;
//    
//always @(posedge clock_signal or posedge reset)
//  begin
//    if(reset)
//        clock_signal_div_two <= 1'b1;
//    else
//        clock_signal_div_two <= ~clock_signal_div_two;    
//  end 
//
logic clk_div_two;
logic [1:0] clk_div_ff;
always @(posedge clock or posedge reset)
  begin
    if(reset)
        clk_div_ff <= 1'b0;
    else
        clk_div_ff <= clk_div_ff + 1'b1;    
  end 
assign clock_div_two = clk_div_ff[0];


logic [31:0] bipolar_signal;
logic [31:0] bipolar_signal_cos;
logic [31:0] bipolar_signal_sin;
//logic [31:0] bipolar;
logic [31:0] odd;
logic [31:0] even;
logic [31:0] stream_1;
logic [31:0] after_sin_data;
logic [31:0] after_cos_data;
logic [31:0] sinus;
logic [31:0] cosinus;

sinus_wave sw
(
.clock(clock),
.reset(reset),
.sinus(sinus)
);

cosinus_wave cw
(
.clock(clock),
.reset(reset),
.cosinus(cosinus)
);

multiply_with_sinus mws(
.clock_signal(clock_div_two),
.reset(reset),
.data(data),
.sinus(sinus),
.after_sin_data(after_sin_data)

);

multiply_with_cosinus mwc(
.clock_signal(clock_div_two),
.reset(reset),
.data(data),
.cosinus(cosinus),
.after_cos_data(after_cos_data)

);

iir1 iir1_sin(
.clock(clock_div_two),
//.clock_signal(clock_signal_div_two),
.reset(reset),
.data_before_iir(after_sin_data),
.data_after_iir(bipolar_signal_sin),
.regulating_the_frequency_counter(regulating_the_frequency_counter)
);
//
iir1 iir2_cos(
.clock(clock_div_two),
//.clock_signal(clock_signal_div_two),
.reset(reset),
.data_before_iir(after_cos_data),
.data_after_iir(bipolar_signal_cos),
.regulating_the_frequency_counter(regulating_the_frequency_counter)
);
//
interlacer d
(
.bipolar_output(bipolar),
.clock_signal(clock_signal),
.reset(reset),
.odd(bipolar_signal_cos),
.even(bipolar_signal_sin)
);
//
//bipolar_to_unipolar_converter btuc
//(
//.random_bit(bipolar),
//.clock_signal(clock),
//.reset(reset),
//.unipolar_signal(unipolar_signal)
//);

endmodule

module multiply_with_cosinus(
input logic clock_signal,
input logic reset,
input logic signed [31:0] data,
input logic signed [31:0] cosinus,
output logic signed [31:0] after_cos_data

);

always @(posedge clock_signal or posedge reset)
begin
     if (reset)
 begin 
 after_cos_data <= 0;
 end
     else begin
        after_cos_data<=data*cosinus;
     end
end

endmodule:multiply_with_cosinus


module multiply_with_sinus(
input logic clock_signal,
input logic reset,
input logic signed [31:0] data,
input logic signed [31:0] sinus,
output logic signed [31:0] after_sin_data

);

always @(posedge clock_signal or posedge reset)
begin
     if (reset)
 begin 
 after_sin_data <= 0;
 end
     else begin
        after_sin_data<=data*sinus;
     end
end

endmodule:multiply_with_sinus


module iir1(
 input logic clock,
 input logic reset,
   input logic signed [31:0] data_before_iir,
 input logic signed [31:0] regulating_the_frequency_counter,
   output logic signed [31:0] data_after_iir
) ;

int count_0 = 0;
int count_minus_1 = 0;

logic signed [31:0] data_after;

// filter coefficients
wire signed [31:0] b1, b2, b3, b4, b5,
a2, a3, a4, a5;

// filter variables
wire signed [63:0] b1_in, b2_in, b3_in, b4_in, b5_in;
wire signed [63:0] a2_out, a3_out, a4_out, a5_out;

// history pipeline regs
reg signed [63:0] f1_n1, f1_n2, f1_n3, f1_n4;
// history pipeline inputs
wire signed [63:0] f1_n1_input, f1_n2_input, f1_n3_input, f1_n4_input, f1_n0; 

// filter coefficients values
assign a2 = -127202; //-3.88192 * 2^15 = -127 202.75... = -127 202 
assign a3 = 185226; //5.65267 * 2^15 = 185 226.69... = 185 226
assign a4 = -119909; //-3.65936 * 2^15 = -119 909.90... = -119 909
assign a5 = 29117; //0.88861 * 2^15 = 29 117.97... = 29 117

assign b1 = 107; //0.00328 * 2^15 = 107.47... = 107
assign b2 = -419; //-0.01279 * 2^15 = -419.10... = -419
assign b3 = 623; //0.01902 * 2^15 = 623.24... = 623
assign b4 = -419; //-0.01279 * 2^15 = -419.10... = -419
assign b5 = 107; //0.00328 * 2^15 = 107.47... = 107

// update filter variables
assign b1_in = b1*data_before_iir;
assign b2_in = b2*data_before_iir;
assign b3_in = b3*data_before_iir;
assign b4_in = b4*data_before_iir;
assign b5_in = b5*data_before_iir;

assign a2_out = a2*f1_n0;
assign a3_out = a3*f1_n0;
assign a4_out = a4*f1_n0;
assign a5_out = a5*f1_n0;

// add operations
assign f1_n1_input = b2_in + f1_n2 - a2_out;
assign f1_n2_input = b3_in + f1_n3 - a3_out;
assign f1_n3_input = b4_in + f1_n4 - a4_out;
assign f1_n4_input = b5_in - a5_out;

// scale the output and turncate for audi[WIDTH-1:0]o codec
assign f1_n0 = (f1_n1 + b1_in) >>> 20;
assign data_after = f1_n0;

// Run the filter state machine at audio sample rate
always @ (posedge clock or posedge reset) 
begin
    if (reset)
    begin
        f1_n1 <= 0;
        f1_n2 <= 0; 
        f1_n3 <= 0;
        f1_n4 <= 0;         
    end
    else 
    begin
        f1_n1 <= f1_n1_input;
        f1_n2 <= f1_n2_input;  
        f1_n3 <= f1_n3_input;
        f1_n4 <= f1_n4_input;     
    end
end 

always @ (posedge clock or posedge reset) 
begin
    if (reset)
      begin
 count_0 <= 0;
        count_minus_1 <= 0;
 data_after_iir <= 0;     
      end
    else 
   begin 
   
     if (regulating_the_frequency_counter==87)
         begin
           if (count_minus_1 <= count_0) 
              begin
              data_after_iir <= -1;
              count_0 <= 0;
              count_minus_1 <= 0; 
              end
           else
               begin
               data_after_iir <= 1;
               count_0 <= 0;
                count_minus_1 <= 0; 
               end     
         end
     else 
        begin
          if (data_after == 1) count_0 += 1;
          if (data_after == -2) count_minus_1 += 1;
        end
   end
end 

endmodule


module interlacer(
output logic signed [31:0] bipolar_output,
input clock_signal,
input reset,
input logic signed [31:0] odd,
input logic signed [31:0] even
);

logic queue = 1;
logic [1:0] b_s ;

always @(posedge clock_signal /*or posedge reset*/)
  begin
  if (queue == 1'b1) 
 begin
  b_s [0] <= ~odd[31];
  b_s [1] <= b_s [0];
      bipolar_output <= b_s[1];
 end
  else
 begin
  b_s [0] <= ~even[31];
  b_s [1] <= b_s [0];
      bipolar_output <= b_s[1];
 end
    queue <= queue + 1'b1;
  end

endmodule:interlacer