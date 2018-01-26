module D_ff(input clk, input reset, input wr_en, input d, output reg q);

	always @(negedge clk)
	begin
		if(reset == 1)
			q = 1'b0;
		else if(wr_en == 1)
			q = d;
	end

endmodule


//comparator module compares input tag with tagRegister tag.
//It returns 1 if tags matches otherwise it returns 0.
module comparator(input [23:0] in1, input [23:0] in2, output reg equal);
		
	//Write your code here
	always @(in1,in2)
	begin
		if(in1 == in2)
			equal = 1;
		else
			equal = 0;
	end
	
endmodule


module mux16to1_24bits(input [23:0] input0, input [23:0] input1, input [23:0] input2, input [23:0] input3, input [23:0] input4, input [23:0] input5, input [23:0] input6, input [23:0] input7, 
					  input [23:0] input8, input [23:0] input9, input [23:0] input10, input [23:0] input11, input [23:0] input12, input [23:0] input13, input [23:0] input14, input [23:0] input15, input [3:0] indexSel, output reg [23:0] muxOut);
	always @(indexSel or input0 or input1 or input2 or input3 or input4 or input5 or input6 or input7 or input8 or input9 or input10 or input11 or input12 or input13 or input14 or input15)
	begin
	case(indexSel)
		4'b0000: muxOut = input0;
		4'b0001: muxOut = input1;
		4'b0010: muxOut = input2;
		4'b0011: muxOut = input3;
		4'b0100: muxOut = input4;
		4'b0101: muxOut = input5;
		4'b0110: muxOut = input6;
		4'b0111: muxOut = input7;
		4'b1000: muxOut = input8;
		4'b1001: muxOut = input9;
		4'b1010: muxOut = input10;
		4'b1011: muxOut = input11;
		4'b1100: muxOut = input12;
		4'b1101: muxOut = input13;
		4'b1110: muxOut = input14;
		4'b1111: muxOut = input15;
	endcase
	end
endmodule

/*
module mux16to1_8bit(input [7:0] outR0, input [7:0] outR1, input [7:0] outR2, input [7:0] outR3,
	input [7:0] outR4, input [7:0] outR5, input [7:0] outR6, input [7:0] outR7,
	input [7:0] outR8, input [7:0] outR9, input [7:0] outR10, input [7:0] outR11,
	input [7:0] outR12, input [7:0] outR13, input [7:0] outR14, input [7:0] outR15,
	input [3:0] Sel, output reg [7:0] outBus);

	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,Sel)
		case(Sel)
			4'd0:outBus=outR0;
			4'd1:outBus=outR1;
			4'd2:outBus=outR2;
			4'd3:outBus=outR3;
			4'd4:outBus=outR4;
			4'd5:outBus=outR5;
			4'd6:outBus=outR6;
			4'd7:outBus=outR7;
			4'd8:outBus=outR8;
			4'd9:outBus=outR9;
			4'd10:outBus=outR10;
			4'd11:outBus=outR11;
			4'd12:outBus=outR12;
			4'd13:outBus=outR13;
			4'd14:outBus=outR14;
			4'd15:outBus=outR15;
		endcase
endmodule
*/
module byte(input clk, input reset, input write, input [7:0] data, output [7:0] outp);
	D_ff bit0(clk,reset,write, data[0],outp[0]);
	D_ff bit1(clk,reset,write,data[1],outp[1]);
	D_ff bit2(clk,reset,write,data[2],outp[2]);
	D_ff bit3(clk,reset,write,data[3],outp[3]);
	D_ff bit4(clk,reset,write, data[4],outp[4]);
	D_ff bit5(clk,reset,write,data[5],outp[5]);
	D_ff bit6(clk,reset,write,data[6],outp[6]);
	D_ff bit7(clk,reset,write,data[7],outp[7]);
endmodule

module reg24bit(input clk, input reset, input write, input [23:0] tag_in,output [23:0] tag_out);
	D_ff t0(clk,reset,write,tag_in[0],tag_out[0]);
	D_ff t1(clk,reset,write,tag_in[1],tag_out[1]);
	D_ff t2(clk,reset,write,tag_in[2],tag_out[2]);
	D_ff t3(clk,reset,write,tag_in[3],tag_out[3]);
	D_ff t4(clk,reset,write,tag_in[4],tag_out[4]);
	D_ff t5(clk,reset,write,tag_in[5],tag_out[5]);
	D_ff t6(clk,reset,write,tag_in[6],tag_out[6]);
	D_ff t7(clk,reset,write,tag_in[7],tag_out[7]);
	D_ff t8(clk,reset,write,tag_in[8],tag_out[8]);
	D_ff t9(clk,reset,write,tag_in[9],tag_out[9]);
	D_ff t10(clk,reset,write,tag_in[10],tag_out[10]);
	D_ff t11(clk,reset,write,tag_in[11],tag_out[11]);
	D_ff t12(clk,reset,write,tag_in[12],tag_out[12]);
	D_ff t13(clk,reset,write,tag_in[13],tag_out[13]);
	D_ff t14(clk,reset,write,tag_in[14],tag_out[14]);
	D_ff t15(clk,reset,write,tag_in[15],tag_out[15]);
	D_ff t16(clk,reset,write,tag_in[16],tag_out[16]);
	D_ff t17(clk,reset,write,tag_in[17],tag_out[17]);
	D_ff t18(clk,reset,write,tag_in[18],tag_out[18]);
	D_ff t19(clk,reset,write,tag_in[19],tag_out[19]);
	D_ff t20(clk,reset,write,tag_in[20],tag_out[20]);
	D_ff t21(clk,reset,write,tag_in[21],tag_out[21]);
	D_ff t22(clk,reset,write,tag_in[22],tag_out[22]);
	D_ff t23(clk,reset,write,tag_in[23],tag_out[23]);
endmodule

//Cache block/line 128bit
module mux8to1_48bit( input [47:0] in0, input [47:0] in1, input [47:0] in2, input [47:0] in3, input [47:0] in4, input [47:0] in5, input [47:0] in6, input [47:0] in7, input [2:0] sel, output reg [47:0] muxOut);
	always @(sel, in0, in1, in2,in3,in4,in5,in6,in7)
		case(sel)
			3'b000: muxOut = in0;
			3'b001: muxOut = in1;
			3'b010: muxOut = in2;
			3'b011: muxOut = in3;
			3'b100: muxOut = in4;
			3'b101: muxOut = in5;
			3'b110: muxOut = in6;
			3'b111: muxOut = in7;
		endcase
endmodule 

module decoder2to4_sp(input [1:0] deco,input missed,output reg [3:0] outp);
	always @(deco,missed)
	begin
		if(missed == 1'b1) begin outp = 4'b0000; end
		else begin
			case(deco)
			2'd0: outp = 4'b0001;
			2'd1: outp = 4'b0010;
			2'd2: outp = 4'b0100;
			2'd3: outp = 4'b1000;
			endcase
		end
	end
endmodule
module cache_line(input clk, input reset,input write,input missed, input [3:0] offset, input [127:0] block_input,input [31:0] write_data, output [47:0] output_data);
	
	wire [3:0] we;
	decoder2to4_sp dmx(offset[3:2], missed, we);
	wire [7:0] mux_in [15:0];

	wire [7:0] tb_write[15:0];

	//first word's data
	mux2to1_8bit mx0 (write_data[31:24],block_input[7:0]    ,missed,tb_write[0]);
	mux2to1_8bit mx1 (write_data[23:16],block_input[15:8]   ,missed,tb_write[1]);
	mux2to1_8bit mx2 (write_data[15:8] ,block_input[23:16]  ,missed,tb_write[2]);
	mux2to1_8bit mx3 (write_data[7:0]  ,block_input[31:24]  ,missed,tb_write[3]);
	//second word's data
	mux2to1_8bit mx4 (write_data[31:24],block_input[39:32]  ,missed,tb_write[4]);
	mux2to1_8bit mx5 (write_data[23:16],block_input[47:40]  ,missed,tb_write[5]);
	mux2to1_8bit mx6 (write_data[15:8] ,block_input[55:48]  ,missed,tb_write[6]);
	mux2to1_8bit mx7 (write_data[7:0]  ,block_input[63:56]  ,missed,tb_write[7]);
	//third word's data
	mux2to1_8bit mx8 (write_data[31:24],block_input[71:64]  ,missed,tb_write[8]);
	mux2to1_8bit mx9 (write_data[23:16],block_input[79:72]  ,missed,tb_write[9]);
	mux2to1_8bit mx10(write_data[15:8] ,block_input[87:80]  ,missed,tb_write[10]);
	mux2to1_8bit mx11(write_data[7:0]  ,block_input[95:88]  ,missed,tb_write[11]);
	//fourth word's data
	mux2to1_8bit mx12(write_data[31:24],block_input[103:96] ,missed,tb_write[12]);
	mux2to1_8bit mx13(write_data[23:16],block_input[111:104],missed,tb_write[13]);
	mux2to1_8bit mx14(write_data[15:8] ,block_input[119:112],missed,tb_write[14]);
	mux2to1_8bit mx15(write_data[7:0]  ,block_input[127:120],missed,tb_write[15]);

	//first word write
	byte bite0(clk,reset,(write & we[0])|missed,  tb_write[0], mux_in[0]);
	byte bite1(clk,reset,(write & we[0])|missed,  tb_write[1], mux_in[1]);
	byte bite2(clk,reset,(write & we[0])|missed,  tb_write[2] , mux_in[2]);
	byte bite3(clk,reset,(write & we[0])|missed,  tb_write[3]  , mux_in[3]);
	//second word write
	byte bite4(clk,reset,(write & we[1])|missed,  tb_write[4], mux_in[4]);
	byte bite5(clk,reset,(write & we[1])|missed,  tb_write[5], mux_in[5]);
	byte bite6(clk,reset,(write & we[1])|missed,  tb_write[6] , mux_in[6]);
	byte bite7(clk,reset,(write & we[1])|missed,  tb_write[7]  , mux_in[7]);
	//third word write
	byte bite8(clk,reset, (write & we[2])|missed,  tb_write[8], mux_in[8]);
	byte bite9(clk,reset, (write & we[2])|missed,  tb_write[9], mux_in[9]);
	byte bite10(clk,reset,(write & we[2])|missed, tb_write[10], mux_in[10]);
	byte bite11(clk,reset,(write & we[2])|missed, tb_write[11] , mux_in[11]);
	//fourrh word write
	byte bite12(clk,reset,(write & we[3])|missed, tb_write[12], mux_in[12]);
	byte bite13(clk,reset,(write & we[3])|missed, tb_write[13], mux_in[13]);
	byte bite14(clk,reset,(write & we[3])|missed, tb_write[14] , mux_in[14]);
	byte bite15(clk,reset,(write & we[3])|missed, tb_write[15]  , mux_in[15]);
	
	mux8to1_48bit mxxx( {mux_in[0], mux_in[1], mux_in[2], mux_in[3], mux_in[4], mux_in[5]},
							  {mux_in[2], mux_in[3], mux_in[4], mux_in[5], mux_in[6],mux_in[7]},
							  {mux_in[4], mux_in[5], mux_in[6], mux_in[7],mux_in[8],mux_in[9]},
							  {mux_in[6], mux_in[7], mux_in[8], mux_in[9],mux_in[10], mux_in[11]},
							  {mux_in[8], mux_in[9], mux_in[10],mux_in[11], mux_in[12], mux_in[13]},
							  {mux_in[10],mux_in[11],mux_in[12], mux_in[13], mux_in[14], mux_in[15]},
							  {mux_in[12],mux_in[13], mux_in[14], mux_in[15], 16'b0000000000000000},
							  {mux_in[14],mux_in[15], 32'b0000_0000_0000_0000_0000_0000_0000_0000},
							  offset[3:1], output_data);
endmodule

module mux16to1_48bit(input [47:0] in0,input [47:0] in1,input [47:0] in2,input [47:0] in3,input [47:0] in4,input [47:0] in5,input [47:0] in6,input [47:0] in7,input [47:0] in8,input [47:0] in9,input [47:0] in10,input [47:0] in11,input [47:0] in12,input [47:0] in13,input [47:0] in14,input [47:0] in15, input [3:0] sel, output reg [47:0] mx_out);
always @(sel,in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15)	
	case(sel)
		4'b0000: mx_out = in0;
		4'b0001: mx_out = in1;
		4'b0010: mx_out = in2;
		4'b0011: mx_out = in3;
		4'b0100: mx_out = in4;
		4'b0101: mx_out = in5;
		4'b0110: mx_out = in6;
		4'b0111: mx_out = in7;
		4'b1000: mx_out = in8;
		4'b1001: mx_out = in9;
		4'b1010: mx_out = in10;
		4'b1011: mx_out = in11;
		4'b1100: mx_out = in12;
		4'b1101: mx_out = in13;
		4'b1110: mx_out = in14;
		4'b1111: mx_out = in15;
	endcase
endmodule


module cache_col(input clk, input reset, input write, input missed_it, input [3:0] index,  input [3:0] offset, input [127:0] block_inp, input [31:0] data_in, output [47:0] data_out);
	
	wire [15:0] atv_line;
	wire [47:0] out[15:0];
	
	decoder4to16 ddmx(index,atv_line);
	
	cache_line line0(clk,reset,write & atv_line[0],  missed_it & atv_line[0], offset, block_inp, data_in, out[0]);
	cache_line line1(clk,reset,write & atv_line[1],  missed_it & atv_line[1], offset, block_inp, data_in, out[1]);
	cache_line line2(clk,reset,write & atv_line[2],  missed_it & atv_line[2], offset, block_inp, data_in, out[2]);
	cache_line line3(clk,reset,write & atv_line[3],  missed_it & atv_line[3], offset, block_inp, data_in, out[3]);
	cache_line line4(clk,reset,write & atv_line[4],  missed_it & atv_line[4], offset, block_inp, data_in, out[4]);
	cache_line line5(clk,reset,write & atv_line[5],  missed_it & atv_line[5], offset, block_inp, data_in, out[5]);
	cache_line line6(clk,reset,write & atv_line[6],  missed_it & atv_line[6], offset, block_inp, data_in, out[6]);
	cache_line line7(clk,reset,write & atv_line[7],  missed_it & atv_line[7], offset, block_inp, data_in, out[7]);
	cache_line line8(clk,reset,write & atv_line[8],  missed_it & atv_line[8], offset, block_inp, data_in, out[8]);
	cache_line line9(clk,reset,write & atv_line[9],  missed_it &  atv_line[9], offset, block_inp, data_in, out[9]);
	cache_line line10(clk,reset,write & atv_line[10],missed_it & atv_line[10], offset, block_inp, data_in, out[10]);
	cache_line line11(clk,reset,write & atv_line[11],missed_it & atv_line[11], offset, block_inp, data_in, out[11]);
	cache_line line12(clk,reset,write & atv_line[12],missed_it & atv_line[12], offset, block_inp, data_in, out[12]);
	cache_line line13(clk,reset,write & atv_line[13],missed_it & atv_line[13], offset, block_inp, data_in, out[13]);
	cache_line line14(clk,reset,write & atv_line[14],missed_it & atv_line[14], offset, block_inp, data_in, out[14]);
	cache_line line15(clk,reset,write & atv_line[15],missed_it & atv_line[15], offset, block_inp, data_in, out[15]);

	mux16to1_48bit mmx(out[0],out[1],out[2],out[3],out[4],out[5],out[6],out[7],out[8],out[9],out[10],out[11],out[12],out[13],out[14],out[15],index,data_out);

endmodule



module valid_array(input clk, input reset, input write_val, input [3:0] index, input valid_in,output valid_out);
	wire [15:0] we;
	wire [15:0] mux_val;

	decoder4to16 dmx_val(index,we);

	D_ff v0(clk, reset,   we[0] & write_val, valid_in, mux_val[0]);
	D_ff v1(clk, reset,   we[1] & write_val, valid_in, mux_val[1]);
	D_ff v2(clk, reset,   we[2] & write_val, valid_in, mux_val[2]);
	D_ff v3(clk, reset,   we[3] & write_val, valid_in, mux_val[3]);
	D_ff v4(clk, reset,   we[4] & write_val, valid_in, mux_val[4]);
	D_ff v5(clk, reset,   we[5] & write_val, valid_in, mux_val[5]);
	D_ff v6(clk, reset,   we[6] & write_val, valid_in, mux_val[6]);
	D_ff v7(clk, reset,   we[7] & write_val, valid_in, mux_val[7]);
	D_ff v8(clk, reset,   we[8] & write_val, valid_in, mux_val[8]);
	D_ff v9(clk, reset,   we[9] & write_val, valid_in, mux_val[9]);
	D_ff v10(clk, reset, we[10] & write_val, valid_in, mux_val[10]);
	D_ff v11(clk, reset, we[11] & write_val, valid_in, mux_val[11]);
	D_ff v12(clk, reset, we[12] & write_val, valid_in, mux_val[12]);
	D_ff v13(clk, reset, we[13] & write_val, valid_in, mux_val[13]);
	D_ff v14(clk, reset, we[14] & write_val, valid_in, mux_val[14]);
	D_ff v15(clk, reset, we[15] & write_val, valid_in, mux_val[15]);

	mux16to1_1bit validbitmux(mux_val[0],mux_val[1],mux_val[2],mux_val[3],mux_val[4],mux_val[5],mux_val[6],mux_val[7],mux_val[8],mux_val[9],mux_val[10],mux_val[11],mux_val[12],mux_val[13],mux_val[14],mux_val[15],index,valid_out);
endmodule

module tag_array(input clk, input reset, input write, input [3:0] index,  input [23:0] tagIn, output [23:0] tagOut);
	wire [15:0] we;
	wire[23:0] out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15;
	
	decoder4to16 dectag(index, we);
	
	reg24bit tr0  (clk, reset, we[0]&write, tagIn, out0);
	reg24bit tr1  (clk, reset, we[1]&write, tagIn, out1);
	reg24bit tr2  (clk, reset, we[2]&write, tagIn, out2);
	reg24bit tr3  (clk, reset, we[3]&write, tagIn, out3);
	reg24bit tr4  (clk, reset, we[4]&write, tagIn, out4);
	reg24bit tr5  (clk, reset, we[5]&write, tagIn, out5);
	reg24bit tr6  (clk, reset, we[6]&write, tagIn, out6);
	reg24bit tr7  (clk, reset, we[7]&write, tagIn, out7);
	reg24bit tr8  (clk, reset, we[8]&write, tagIn, out8);
	reg24bit tr9  (clk, reset, we[9]&write, tagIn, out9);
	reg24bit tr10 (clk, reset, we[10]&write, tagIn, out10);
	reg24bit tr11 (clk, reset, we[11]&write, tagIn, out11);
	reg24bit tr12 (clk, reset, we[12]&write, tagIn, out12);
	reg24bit tr13 (clk, reset, we[13]&write, tagIn, out13);
	reg24bit tr14 (clk, reset, we[14]&write, tagIn, out14);
	reg24bit tr15 (clk, reset, we[15]&write, tagIn, out15);
	mux16to1_24bits muxtag (out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15, index, tagOut);
endmodule


module mux16to1_1bit(input in0,input in1,input in2,input in3,input in4,input in5,input in6,input in7,input in8,input in9,input in10,input in11,input in12,input in13,input in14,input in15, input [3:0] sel, output reg mx_out);
always @(sel,in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15)	
	case(sel)
		4'b0000: mx_out = in0;
		4'b0001: mx_out = in1;
		4'b0010: mx_out = in2;
		4'b0011: mx_out = in3;
		4'b0100: mx_out = in4;
		4'b0101: mx_out = in5;
		4'b0110: mx_out = in6;
		4'b0111: mx_out = in7;
		4'b1000: mx_out = in8;
		4'b1001: mx_out = in9;
		4'b1010: mx_out = in10;
		4'b1011: mx_out = in11;
		4'b1100: mx_out = in12;
		4'b1101: mx_out = in13;
		4'b1110: mx_out = in14;
		4'b1111: mx_out = in15;
	endcase
endmodule

//MRU bit storage
module reg2bit(input clk, input reset, input write_mru, input [1:0] q, output [1:0] d);
	D_ff m0(clk,reset,write_mru,q[0],d[0]);
	D_ff m1(clk,reset,write_mru,q[1],d[1]);
endmodule

module mruArray(input clk, input reset, input [3:0] index, input write, input [1:0] mruIn, output [1:0] mruOut);
	wire [1:0] mr0,mr1,mr2,mr3,mr4,mr5,mr6,mr7,mr8,mr9,mr10,mr11,mr12,mr13,mr14,mr15;
	wire [15:0] we;
	decoder4to16 mrudec(index, we);
	reg2bit v0(clk, reset, we[0] & write, mruIn, mr0);
	reg2bit v1(clk, reset, we[1] & write, mruIn, mr1);
	reg2bit v2(clk, reset, we[2] & write, mruIn, mr2);
	reg2bit v3(clk, reset, we[3] & write, mruIn, mr3);
	reg2bit v4(clk, reset, we[4] & write, mruIn, mr4);
	reg2bit v5(clk, reset, we[5] & write, mruIn, mr5);
	reg2bit v6(clk, reset, we[6] & write, mruIn, mr6);
	reg2bit v7(clk, reset, we[7] & write, mruIn, mr7);
	reg2bit v8(clk, reset, we[8] & write, mruIn, mr8);
	reg2bit v9(clk, reset, we[9] & write, mruIn, mr9);
	reg2bit v10(clk, reset, we[10] & write, mruIn, mr10);
	reg2bit v11(clk, reset, we[11] & write, mruIn, mr11);
	reg2bit v12(clk, reset, we[12] & write, mruIn, mr12);
	reg2bit v13(clk, reset, we[13] & write, mruIn, mr13);
	reg2bit v14(clk, reset, we[14] & write, mruIn, mr14);
	reg2bit v15(clk, reset, we[15] & write, mruIn, mr15);
	mux16to1_2bit mrumux(mr0,mr1,mr2,mr3,mr4,mr5,mr6,mr7,mr8,mr9,mr10,mr11,mr12,mr13,mr14,mr15,index,mruOut);
endmodule

module mux16to1_2bit(input [1:0] in0,input  [1:0] in1,input [1:0] in2,input [1:0] in3,input [1:0] in4,input [1:0] in5,input [1:0] in6,input [1:0] in7,input [1:0] in8,input [1:0] in9,input [1:0] in10,input [1:0] in11,input [1:0] in12,input [1:0] in13,input [1:0] in14,input [1:0] in15, input [3:0] sel, output reg [1:0] mx_out);

always @(sel,in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15)	
	case(sel)
		4'b0000: mx_out = in0;
		4'b0001: mx_out = in1;
		4'b0010: mx_out = in2;
		4'b0011: mx_out = in3;
		4'b0100: mx_out = in4;
		4'b0101: mx_out = in5;
		4'b0110: mx_out = in6;
		4'b0111: mx_out = in7;
		4'b1000: mx_out = in8;
		4'b1001: mx_out = in9;
		4'b1010: mx_out = in10;
		4'b1011: mx_out = in11;
		4'b1100: mx_out = in12;
		4'b1101: mx_out = in13;
		4'b1110: mx_out = in14;
		4'b1111: mx_out = in15;
	endcase
endmodule





module decoder4to16(input [3:0] destReg, output reg [15:0] decOut);
	always @(destReg)
		case(destReg)
			4'd0:  decOut=16'b0000_0000_0000_0001;
			4'd1:  decOut=16'b0000_0000_0000_0010;
			4'd2:  decOut=16'b0000_0000_0000_0100;
			4'd3:  decOut=16'b0000_0000_0000_1000;
			4'd4:  decOut=16'b0000_0000_0001_0000;
			4'd5:  decOut=16'b0000_0000_0010_0000;
			4'd6:  decOut=16'b0000_0000_0100_0000;
			4'd7:  decOut=16'b0000_0000_1000_0000;
			4'd8:  decOut=16'b0000_0001_0000_0000;
			4'd9:  decOut=16'b0000_0010_0000_0000;
			4'd10: decOut=16'b0000_0100_0000_0000;
			4'd11: decOut=16'b0000_1000_0000_0000;
			4'd12: decOut=16'b0001_0000_0000_0000;
			4'd13: decOut=16'b0010_0000_0000_0000;
			4'd14: decOut=16'b0100_0000_0000_0000;
			4'd15: decOut=16'b1000_0000_0000_0000;
		endcase
endmodule

module decoder2to4(input [1:0] inp, output reg [3:0] deco);
	always @(inp)
		case(inp)
			2'd0: deco = 4'b0001;
			2'd1: deco = 4'b0010;
			2'd2: deco = 4'b0100;
			2'd3: deco = 4'b1000;
		endcase
endmodule

module encoder4to2(input [3:0] inp, output reg [1:0] outp);
	always @ (inp)
	 casex(inp)
	 	/*4'b1xxx: outp = 2'b00;
	 	4'b01xx: outp = 2'b01;
	 	4'b001x: outp = 2'b10;
	 	4'b0001: outp = 2'b11;
	 	default: outp = 2'b00;*/
		4'b0001: outp = 2'b00;
	 	4'b0010: outp = 2'b01;
	 	4'b0100: outp = 2'b10;
	 	4'b1000: outp = 2'b11;
	 	default: outp = $random%2;
	 endcase
endmodule

// encoder for replcement takes  inp == validity;
module encoder4to2x(input [3:0] inp, output reg [1:0] outp);
	always @ (inp)
	 casex(inp)
	 	4'b1111: outp = 2'b00;
	 	4'b0xxx: outp = 2'b00;
	 	4'b10xx: outp = 2'b01;
	 	4'b110x: outp = 2'b10;
	 	4'b1110: outp = 2'b11;
	 endcase
endmodule


module mux4to1_1bit(input in0, input in1, input in2, input in3, input [1:0] sel, output reg outp);
	always @ (in0,in1,in2,in3,sel)
		case(sel)
			2'b00: outp = in0;
			2'b01: outp = in1;
			2'b10: outp = in2;
			2'b11: outp = in3;
		endcase
endmodule

module mux4to1_48bit(input [47:0] in0, input [47:0] in1, input [47:0] in2, input [47:0] in3, input [1:0] sel, output reg [47:0] outp);
	always @(sel,in0,in1,in2,in3)
	case(sel)
		2'b00: outp = in0;
		2'b01: outp = in1;
		2'b10: outp = in2;
		2'b11: outp = in3;
	endcase
endmodule

module mux4to1_2bit(input [1:0] in0, input [1:0] in1, input [1:0] in2, input [1:0] in3, input [1:0] sel, output reg [1:0] outp);
	always @(sel,in0,in1,in2,in3)
	case(sel)
		2'b00: outp = in0;
		2'b01: outp = in1;
		2'b10: outp = in2;
		2'b11: outp = in3;
	endcase
endmodule

module mux2to1_2bit(input [1:0] in0, input [1:0] in1, input sel, output reg [1:0] outp);
	always @(sel,in0,in1)
	case(sel)
		1'b0: outp = in0;
		2'b1: outp = in1;
	endcase
endmodule

module mux2to1_8bit(input [7:0] in0, input [7:0] in1, input sel, output reg [7:0] outp);
	always @(in0,in1,sel)
		case(sel)
			2'b0: outp = in0;
			2'b1: outp = in1;
		endcase
endmodule

module encoder4to2z(input [3:0] val, output reg [1:0] m_temp);
	always @(val)
		casex(val)
			4'bxxx0: m_temp = 2'b00;
			4'bxx01: m_temp = 2'b01;
			4'bx011: m_temp = 2'b10;
			4'b0111: m_temp = 2'b11;
			default: m_temp = 2'b00;
		endcase
endmodule

module decoder4to2buffer(input [1:0] inp, input [1:0] last, input chk, output reg [3:0] outp);
	always @(inp)begin
	
	if(chk == 1'b1) begin
			case(inp)
			2'b00: outp = 4'b0001 | (4'b1 <<< last);
			2'b01: outp = 4'b0010 | (4'b1 <<< last);
			2'b10: outp = 4'b0100 | (4'b1 <<< last);
			2'b11: outp = 4'b1000 | (4'b1 <<< last);
			endcase
		end
	else begin
		case(inp)
			2'b00: outp = 4'b0001;
			2'b01: outp = 4'b0010;
			2'b10: outp = 4'b0100;
			2'b11: outp = 4'b1000;
			
		endcase
	end
		
	end
endmodule

module help(input [127:0] block_in, input [31:0] data_w, input write , input [1:0] offset, output reg [127:0] block_out);
	always @(block_in,data_w,offset) begin
			block_out = block_in;
		if(write == 1'b1) begin
			case(offset)
			2'b00:block_out[127:96] = data_w;
			2'b01:block_out[95:64] = data_w;
			2'b10:block_out[63:32] = data_w;
			2'b11: block_out[31:0] = data_w;
		endcase
		end
		end
endmodule


module buffer(input clk, input reset, input write,input [31:0] address, input [127:0] block_in, wire [31:0] data_in,output [127:0] block_out);

	wire [3:0] valid;
	reg  [3:0] valid_in;
	wire [23:0] tagg[3:0];
	wire [3:0] valid_in_wr;
	assign valid_in_wr = valid_in;

	reg missed;
	wire missed_wr;
	assign missed_wr = missed;

	reg [3:0] we ;
	reg [1:0] now,last;

	initial begin
		now = 4'b00;
		valid_in = 4'b0000;
		missed = 1'b1;
	end

	wire [1:0] now_wr;
	assign now_wr = now;
	wire [47:0] data_out[3:0];
	wire [3:0] eq;
	//reg chk; 
	//always @(*) chk = address[31:8] == tagg[last];
	D_ff vi0(clk,reset,we[0],valid_in[0],valid[0]);
	D_ff vi1(clk,reset,we[1],valid_in[1],valid[1]);
	D_ff vi2(clk,reset,we[2],valid_in[2],valid[2]);
	D_ff vi3(clk,reset,we[3],valid_in[3],valid[3]);

	reg24bit tag_0(clk,reset, we[0], address[31:8], tagg[0]);
	reg24bit tag_1(clk,reset, we[1], address[31:8], tagg[1]);
	reg24bit tag_2(clk,reset, we[2], address[31:8], tagg[2]);
	reg24bit tag_3(clk,reset, we[3], address[31:8], tagg[3]);
	//reg24bit(input clk, input reset, input write, input [23:0] tag_in,output [23:0] tag_out);

	cache_line line0(clk,reset,write & we[0] ,1'b1 & we[0],4'b0000, block_out, data_in,  data_out[0]);
	cache_line line1(clk,reset,write & we[1] ,1'b1 & we[1],4'b0000, block_out, data_in,  data_out[1]);
	cache_line line2(clk,reset,write & we[2] ,1'b1 & we[2],4'b0000, block_out, data_in,  data_out[2]);
	cache_line line3(clk,reset,write & we[3] ,1'b1 & we[3],4'b0000, block_out, data_in,  data_out[3]);
	//module cache_line(input clk, input reset,input write,input missed, input [3:0] offset, input [127:0] block_input,input [31:0] write_data, output [47:0] output_data);


	comparator cc0(tagg[0],address[31:8],eq[0]);
	comparator cc1(tagg[1],address[31:8],eq[1]);
	comparator cc2(tagg[2],address[31:8],eq[2]);
	comparator cc3(tagg[3],address[31:8],eq[3]);
	//decoder4to2buffer dee(now,last,valid[last] == 1'b1 && address[31:8] == tagg[last],we);
	help help_inst(block_in, data_in,write,address[3:2],block_out);

	always @(negedge clk) begin
	if(reset == 1'b0) begin
		last = now;
		now = now + 1;
		case(now)
			2'b00: we = 4'b0001;
			2'b01: we = 4'b0010;
			2'b10: we = 4'b0100;
			2'b11: we = 4'b1000;
		endcase
	end
	end

	always @(now) begin
		if(reset == 1'b0) begin
			valid_in[now] = 1'b1;
		
			if(valid[0] == 1'b1 & eq[0] == 1) begin
				valid_in[0] = 1'b0;
				if(now == 2'b00)
					valid_in[0] = 1'b1;
				we[0] = 1'b1;
			end
			
			if(valid[1] == 1'b1 & eq[1] == 1) begin
				valid_in[1] = 1'b0;
				if(now == 2'b01)
					valid_in[1] = 1'b1;
				we[1]  = 1'b1;
			end
			
			if(valid[2] == 1'b1 & eq[2] == 1) begin
				valid_in[2] = 1'b0;
				if(now == 2'b10)
					valid_in[2] = 1'b1;
				we[2] = 1'b1;
			end
			
			if(valid[3] == 1'b1 & eq[3] == 1) begin
				valid_in[3] = 1'b0;
				if(now == 2'b11)
					valid_in[3] = 1'b1;
				we[3] = 1'b1;
			end
		end
		
	end

endmodule

//module ctrl_ckt(input clk, input reset, input way_hit, input cache_hit, input write, input 
module cache(input clk,input reset, input write, input [31:0] address, input [127:0] block_tb,input [31:0] data_in, output [47:0] data_out, output [7:0] lbu, output way_hit_out,output ready);

	wire [47:0] col_data[3:0];
	wire [3:0] validity;
	wire [3:0] valid_tag_we,eq;
	wire [23:0] new_tag[3:0];
	wire [3:0] and_out;
	wire [1:0] mru_out1, mru_out2;
	wire way_hit;
	wire cache_hit;
	wire [1:0] final_mru_wr;
	
	//CONTROL SIGNAL
	wire [3:0] way_wr;
	wire mru_upd;
	wire state_write;
	wire block_replace;
	wire [1:0]mru_temp, mru_out3;
	assign way_hit_out = way_hit;
	wire [127:0] block_out;

	mruArray mru0(clk, reset, address[7:4], mru_upd, final_mru_wr, mru_out1);
	
	decoder2to4 dec_fin(final_mru_wr,way_wr);
	cache_col col0(clk,reset, (write & state_write & way_wr[0]) ,block_replace & way_wr[0],address[7:4],address[3:0],block_tb,data_in,col_data[0]);
	cache_col col1(clk,reset, (write & state_write & way_wr[1]) ,block_replace & way_wr[1],address[7:4],address[3:0],block_tb,data_in,col_data[1]);
	cache_col col2(clk,reset, (write & state_write & way_wr[2]) ,block_replace & way_wr[2],address[7:4],address[3:0],block_tb,data_in,col_data[2]);
	cache_col col3(clk,reset, (write & state_write & way_wr[3]) ,block_replace & way_wr[3],address[7:4],address[3:0],block_tb,data_in,col_data[3]);

	valid_array validd0(clk,reset, block_replace & way_wr[0],address[7:4], 1'b1, validity[0]);
	valid_array validd1(clk,reset, block_replace & way_wr[1],address[7:4], 1'b1, validity[1]);
	valid_array validd2(clk,reset, block_replace & way_wr[2],address[7:4], 1'b1, validity[2]);
	valid_array validd3(clk,reset, block_replace & way_wr[3],address[7:4], 1'b1, validity[3]);

	tag_array tagg0(clk,reset, block_replace & way_wr[0],address[7:4],address[31:8],new_tag[0]);
	tag_array tagg1(clk,reset, block_replace & way_wr[1],address[7:4],address[31:8],new_tag[1]);
	tag_array tagg2(clk,reset, block_replace & way_wr[2],address[7:4],address[31:8],new_tag[2]);
	tag_array tagg3(clk,reset, block_replace & way_wr[3],address[7:4],address[31:8],new_tag[3]);

	comparator c0(address[31:8], new_tag[0],eq[0]);
	comparator c1(address[31:8], new_tag[1],eq[1]);
	comparator c2(address[31:8], new_tag[2],eq[2]);
	comparator c3(address[31:8], new_tag[3],eq[3]);

	and a0(and_out[0],eq[0],validity[0]);
	and a1(and_out[1],eq[1],validity[1]);
	and a2(and_out[2],eq[2],validity[2]);
	and a3(and_out[3],eq[3],validity[3]);

	mux4to1_1bit check_way_hit_mux(and_out[0],and_out[1],and_out[2],and_out[3],mru_out1,way_hit);

	assign cache_hit = (and_out[0] | and_out[1] | and_out[2] | and_out[3]);

	encoder4to2 enc_cache_hit_mru(and_out, mru_out2);
	encoder4to2z mru_t(validity,mru_temp);
	mux2to1_2bit xxmuxx(mru_temp, mru_out2,cache_hit, mru_out3);
	//encoder4to2x enc1(validity, mru_out3);

	//mux4to1_2bit mru_fin(mru_out1,mru_out2,mru_out3,2'b00, sel_mru_wr, final_mru_wr);
	
	buffer buff(clk,reset,write,address,block_tb,data_in,block_out);

	
	mux2to1_2bit mru_fn(mru_out1,mru_out3,~way_hit,final_mru_wr);
	
	mux4to1_48bit mx4t48(col_data[0], col_data[1], col_data[2], col_data[3], final_mru_wr, data_out);

	mux2to1_8bit lbu_mx(data_out[39:32], data_out[47:40], address[0],lbu);
	
	ctrl_ckt cc(clk, reset, write, way_hit, cache_hit, state_write, block_replace, mru_upd,ready);


endmodule

module ctrl_ckt(input clk, input reset, input write, input way_hit, input cache_hit, output reg state_write, output reg block_replace, output reg mru_upd,output reg ready);

	reg [1:0] state, nxt_state;
	
	always @( negedge clk) begin
		if(reset == 1'b0)
			state = nxt_state;
		else begin
			nxt_state = 0;
		end
	end
	
	always @ (state, way_hit, cache_hit, write) begin
		case(state)
			3'b000: begin   // Check for way hit
				ready =  1'b0;
				state_write = 1'b0;
				block_replace = 1'b0;
				mru_upd = 1'b0;
				if(way_hit == 1'b1) begin 	// if way hit then jump to write state
					nxt_state = 3'b011;
				end
				else begin
					nxt_state = 3'b001; 		// if way miss then check cache hit;
				end
			end

			3'b001: begin		// State to check cache hit
				state_write = 1'b0;
				block_replace = 1'b0;
				mru_upd = 1'b0;
				if(cache_hit == 1'b1) begin				//if cache hit jump to write state
					nxt_state = 3'b011;
				end
				else begin
					nxt_state = 3'b010; // if cache miss jmp to miss state
				end
			end

			3'b010: begin			// missed
				state_write = 1'b0;
				block_replace = 1'b1;
				mru_upd = 1'b0;
				nxt_state = 3'b011;
			end

			3'b011: begin		// write state
				ready = 1'b1;
				mru_upd = 1'b1;
				if(write == 1'b1)
					state_write = 1'b1;
				else
					state_write = 1'b0;
				block_replace = 1'b0;
				nxt_state = 3'b000;
			end
		endcase
	end
endmodule

module Cache_testBench;
	
	reg clk;
	reg reset;
	reg [31:0] address;
	reg [31:0] data_in;
	reg [127:0] block_tb;
	reg write;
	wire [47:0] im_out;
	wire [7:0] lbu_out;
	wire ready;
	cache cRead(.clk(clk),
			 .reset(reset), 
			.write(write), 
			.address(address), 
			.data_in(data_in), 
			.block_tb(block_tb), 
			.way_hit_out(hit), 
			.lbu(lbu_out), 
			.data_out(im_out),
			.ready(ready));
	initial
	begin
		reset=1;
		clk=0;
		#20 
		reset = 0;           // First Write MISS
		address = 32'ha0000;
		write = 1'b1;
		data_in = 32'h22222222;
		block_tb = 128'b11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111;
		#80		// Same line as last so HIT
		address = 32'ha0000;
		write = 1'b0;
		data_in = 32'd0;
		#80		//Again same line  but differetn offset as last WAY HIT;
		address = 32'ha0004;
		write = 1'b1;
		data_in = 32'habcdef;
		#80		// Same index as last but different tag , so MISS but write in another column
		address = 32'h10000;
		write = 1'b0;
		block_tb = 128'h33333333333333333333333333333333;
		#80				// Third  Column Write with same index MISS
		address = 32'h30000;
		write = 1'b0;
		block_tb = 128'hbbbbbbbbbbbbbbbbbbbbbbbbb;
		#80				// Fourth Column write with same index as prev; MISSS
		address = 32'h40000;
		write = 1'b0;
		block_tb = 128'hccccccccccccccccccccccccc;
		#80			// Access First column,CACHE HIT(not way hit)
		address = 32'ha0000;
		write =1'b0;
		#80			// Fifth write with same index, MISS Since Set is full Replace one of column line randomly;
		address = 32'h50000;
		write = 1'b0;
		block_tb = 128'haaaaaaaaaaaaaaaaaaaaaaaaaaa;
		#80			// Again access the REPLACED BLOCK, MISS
		address = 32'ha0000;
		write = 1'b0;
		block_tb = 128'hddddddddddddddddddddddddddddddd;
		#100
		$finish;
	end
	
	always
	#10 clk =~clk;
endmodule


//module Cache_testBench;
//	
//	reg clk;
//	reg reset;
//	reg [31:0] address;
//	reg [31:0] data_in;
//	reg [127:0] block_tb;
//	reg write;
//	wire [47:0] im_out;
//	wire [7:0] lbu_out;
//	cache cRead(.clk(clk),
//			 .reset(reset), 
//			.write(write), 
//			.address(address), 
//			.data_in(data_in), 
//			.block_tb(block_tb), 
//			.way_hit_out(hit), 
//			.lbu(lbu_out), 
//			.data_out(im_out));
//	initial
//	begin
//		reset=1;
//		clk=0;
//		#20 
//		reset = 0;
//		address = 32'd0;
//		write = 1'b1;
//		data_in = 32'h22222222;
//		block_tb = 128'b11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111;
//		#80
//		address = 32'd0;
//		write = 1'b0;
//		data_in = 32'd0;
//		#40
//		address = 32'b10000;
//		write = 1'b0;
//		block_tb = 128'h33333333333333333333333333333333;
//		#80		
//		$finish;
//	end
//	
//	always
//	#10 clk =~clk;
//endmodule