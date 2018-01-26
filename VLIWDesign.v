// Register File Starts
//Reg DFF
module D_ff(input clk, input reset, input regWrite, input decOut1b , input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=0;
	else
		if(regWrite == 1 && decOut1b==1)
		begin
			q=d;
		end
	end
endmodule
 
//Reg File DFF
module D_ff_2 (input clk, input reset, input regWrite1, input regWrite2, input decOut1b1, input decOut1b2, input d1, input d2, output reg	 q);
	always @ (negedge clk,reset)
	begin
	if(reset==1'b1)
		q=1'b0;
	else begin
			if(regWrite1 == 1'b1 && decOut1b1==1'b1) 
				q=d1;
			if(regWrite2 == 1'b1 && decOut1b2==1'b1) 
				q=d2;
		end
	end
endmodule

module D_ff_IM(input clk, input reset, input d, output reg q);
	always@(reset or posedge clk)
	if(reset)
		q=d;
endmodule


module D_ff_Mem (input clk, input reset, input regWrite, input dec_Out,input Ini, input d, output reg q);
	always @ (posedge clk)
	begin
	if(reset==1)
		q=Ini;
	else
		if(regWrite == 1 && dec_Out==1) begin q=d; end
	end
endmodule


/*module register_Mem(input clk,input reset,input regWrite,input dec_Out1b,input [31:0]Ini, input [31:0] data_In, output [31:0] result);
 	
	D_ff_Mem d_ff_Mem0  (clk, reset, regWrite, dec_Out1b, Ini[0], data_In[0], result[0]);
	D_ff_Mem d_ff_Mem1  (clk, reset, regWrite, dec_Out1b, Ini[1], data_In[1], result[1]);
	D_ff_Mem d_ff_Mem2  (clk, reset, regWrite, dec_Out1b, Ini[2], data_In[2], result[2]);
	D_ff_Mem d_ff_Mem3  (clk, reset, regWrite, dec_Out1b, Ini[3], data_In[3], result[3]);
	D_ff_Mem d_ff_Mem4  (clk, reset, regWrite, dec_Out1b, Ini[4], data_In[4], result[4]);
	D_ff_Mem d_ff_Mem5  (clk, reset, regWrite, dec_Out1b, Ini[5], data_In[5], result[5]);
	D_ff_Mem d_ff_Mem6  (clk, reset, regWrite, dec_Out1b, Ini[6], data_In[6], result[6]);
	D_ff_Mem d_ff_Mem7  (clk, reset, regWrite, dec_Out1b, Ini[7], data_In[7], result[7]);
 
	D_ff_Mem d_ff_Mem8  (clk, reset, regWrite, dec_Out1b, Ini[8], data_In[8], result[8]);
	D_ff_Mem d_ff_Mem9  (clk, reset, regWrite, dec_Out1b, Ini[9], data_In[9], result[9]);
	D_ff_Mem d_ff_Mem10 (clk, reset, regWrite, dec_Out1b, Ini[10], data_In[10], result[10]);
	D_ff_Mem d_ff_Mem11 (clk, reset, regWrite, dec_Out1b, Ini[11], data_In[11], result[11]);
	D_ff_Mem d_ff_Mem12 (clk, reset, regWrite, dec_Out1b, Ini[12], data_In[12], result[12]);
	D_ff_Mem d_ff_Mem13 (clk, reset, regWrite, dec_Out1b, Ini[13], data_In[13], result[13]);
	D_ff_Mem d_ff_Mem14 (clk, reset, regWrite, dec_Out1b, Ini[14], data_In[14], result[14]);
	D_ff_Mem d_ff_Mem15 (clk, reset, regWrite, dec_Out1b, Ini[15], data_In[15], result[15]);
 
	D_ff_Mem d_ff_Mem16 (clk, reset, regWrite, dec_Out1b, Ini[16], data_In[16], result[16]);
	D_ff_Mem d_ff_Mem17 (clk, reset, regWrite, dec_Out1b, Ini[17], data_In[17], result[17]);
	D_ff_Mem d_ff_Mem18 (clk, reset, regWrite, dec_Out1b, Ini[18], data_In[18], result[18]);
	D_ff_Mem d_ff_Mem19 (clk, reset, regWrite, dec_Out1b, Ini[19], data_In[19], result[19]);
	D_ff_Mem d_ff_Mem20 (clk, reset, regWrite, dec_Out1b, Ini[20], data_In[20], result[20]);
	D_ff_Mem d_ff_Mem21 (clk, reset, regWrite, dec_Out1b, Ini[21], data_In[21], result[21]);
	D_ff_Mem d_ff_Mem22 (clk, reset, regWrite, dec_Out1b, Ini[22], data_In[22], result[22]);
	D_ff_Mem d_ff_Mem23 (clk, reset, regWrite, dec_Out1b, Ini[23], data_In[23], result[23]);
 
	D_ff_Mem d_ff_Mem24 (clk, reset, regWrite, dec_Out1b, Ini[24], data_In[24], result[24]);
	D_ff_Mem d_ff_Mem25 (clk, reset, regWrite, dec_Out1b, Ini[25], data_In[25], result[25]);
	D_ff_Mem d_ff_Mem26 (clk, reset, regWrite, dec_Out1b, Ini[26], data_In[26], result[26]);
	D_ff_Mem d_ff_Mem27 (clk, reset, regWrite, dec_Out1b, Ini[27], data_In[27], result[27]);
	D_ff_Mem d_ff_Mem28 (clk, reset, regWrite, dec_Out1b, Ini[28], data_In[28], result[28]);
	D_ff_Mem d_ff_Mem29 (clk, reset, regWrite, dec_Out1b, Ini[29], data_In[29], result[29]);
	D_ff_Mem d_ff_Mem30 (clk, reset, regWrite, dec_Out1b, Ini[30], data_In[30], result[30]);
	D_ff_Mem d_ff_Mem31 (clk, reset, regWrite, dec_Out1b, Ini[31], data_In[31], result[31]);
 	
endmodule */
 
module register_Mem_data(input clk,input reset,input regWrite,input dec_Out1b,input [31:0]Ini, input [31:0] data_In, output [31:0] result);
 
	D_ff_Mem d_ff_Mem0 (clk, reset, regWrite, dec_Out1b, Ini[0], data_In[0], result[0]);
	D_ff_Mem d_ff_Mem1 (clk, reset, regWrite, dec_Out1b, Ini[1], data_In[1], result[1]);
	D_ff_Mem d_ff_Mem2 (clk, reset, regWrite, dec_Out1b, Ini[2], data_In[2], result[2]);
	D_ff_Mem d_ff_Mem3 (clk, reset, regWrite, dec_Out1b, Ini[3], data_In[3], result[3]);
 	
	D_ff_Mem d_ff_Mem4 (clk, reset, regWrite, dec_Out1b, Ini[4], data_In[4], result[4]);
	D_ff_Mem d_ff_Mem5 (clk, reset, regWrite, dec_Out1b, Ini[5], data_In[5], result[5]);
	D_ff_Mem d_ff_Mem6 (clk, reset, regWrite, dec_Out1b, Ini[6], data_In[6], result[6]);
	D_ff_Mem d_ff_Mem7 (clk, reset, regWrite, dec_Out1b, Ini[7], data_In[7], result[7]);
 
 
	D_ff_Mem d_ff_Mem8 (clk, reset, regWrite, dec_Out1b, Ini[8], data_In[8], result[8]);
	D_ff_Mem d_ff_Mem9 (clk, reset, regWrite, dec_Out1b, Ini[9], data_In[9], result[9]);
	D_ff_Mem d_ff_Mem10 (clk, reset, regWrite, dec_Out1b, Ini[10], data_In[10], result[10]);
	D_ff_Mem d_ff_Mem11 (clk, reset, regWrite, dec_Out1b, Ini[11], data_In[11], result[11]);
	
	D_ff_Mem d_ff_Mem12 (clk, reset, regWrite, dec_Out1b, Ini[12], data_In[12], result[12]);
	D_ff_Mem d_ff_Mem13 (clk, reset, regWrite, dec_Out1b, Ini[13], data_In[13], result[13]);
	D_ff_Mem d_ff_Mem14 (clk, reset, regWrite, dec_Out1b, Ini[14], data_In[14], result[14]);
	D_ff_Mem d_ff_Mem15 (clk, reset, regWrite, dec_Out1b, Ini[15], data_In[15], result[15]);
	
	D_ff_Mem d_ff_Mem16 (clk, reset, regWrite, dec_Out1b, Ini[16], data_In[16], result[16]);
	D_ff_Mem d_ff_Mem17 (clk, reset, regWrite, dec_Out1b, Ini[17], data_In[17], result[17]);
	D_ff_Mem d_ff_Mem18 (clk, reset, regWrite, dec_Out1b, Ini[18], data_In[18], result[18]);
	D_ff_Mem d_ff_Mem19 (clk, reset, regWrite, dec_Out1b, Ini[19], data_In[19], result[19]);
 	
	D_ff_Mem d_ff_Mem20 (clk, reset, regWrite, dec_Out1b, Ini[20], data_In[20], result[20]);
	D_ff_Mem d_ff_Mem21 (clk, reset, regWrite, dec_Out1b, Ini[21], data_In[21], result[21]);
	D_ff_Mem d_ff_Mem22 (clk, reset, regWrite, dec_Out1b, Ini[22], data_In[22], result[22]);
	D_ff_Mem d_ff_Mem23 (clk, reset, regWrite, dec_Out1b, Ini[23], data_In[23], result[23]);
 
	D_ff_Mem d_ff_Mem24 (clk, reset, regWrite, dec_Out1b, Ini[24], data_In[24], result[24]);
	D_ff_Mem d_ff_Mem25 (clk, reset, regWrite, dec_Out1b, Ini[25], data_In[25], result[25]);
	D_ff_Mem d_ff_Mem26 (clk, reset, regWrite, dec_Out1b, Ini[26], data_In[26], result[26]);
	D_ff_Mem d_ff_Mem27 (clk, reset, regWrite, dec_Out1b, Ini[27], data_In[27], result[27]);
	
	D_ff_Mem d_ff_Mem28 (clk, reset, regWrite, dec_Out1b, Ini[28], data_In[28], result[28]);
	D_ff_Mem d_ff_Mem29 (clk, reset, regWrite, dec_Out1b, Ini[29], data_In[29], result[29]);
	D_ff_Mem d_ff_Mem30 (clk, reset, regWrite, dec_Out1b, Ini[30], data_In[30], result[30]);
	D_ff_Mem d_ff_Mem31 (clk, reset, regWrite, dec_Out1b, Ini[31], data_In[31], result[31]);
	
endmodule
 
module data_Mem(input clk, input reset,input mem_Write,input memRead, 
input [31:0] pc, input [31:0] data_In,output [31:0] IR );

	wire [31:0] Q_out0, Q_out1, Q_out2, Q_out3, Q_out4, Q_out5, Q_out6, Q_out7, Q_out8, Q_out9, Q_out10, Q_out11, Q_out12, Q_out13, Q_out14, Q_out15,Q_out16, Q_out17, Q_out18, Q_out19, Q_out20, Q_out21, Q_out22, Q_out23, Q_out24, Q_out25, Q_out26, Q_out27, Q_out28, Q_out29, Q_out30, Q_out31,dec_Out;
	
 
	decoder5to32 dec0( pc[4:0], dec_Out);
 
	register_Mem_data r_00(clk, reset, mem_Write, dec_Out[0], 32'd00, data_In, Q_out0); 
	register_Mem_data r_01(clk, reset, mem_Write, dec_Out[1], 32'd01, data_In, Q_out1); 
	register_Mem_data r_02(clk, reset, mem_Write, dec_Out[2], 32'd02, data_In, Q_out2); 
	register_Mem_data r_03(clk, reset, mem_Write, dec_Out[3], 32'd03, data_In, Q_out3); 
	
	register_Mem_data r_04(clk, reset, mem_Write, dec_Out[4], 32'd04, data_In, Q_out4); 
	register_Mem_data r_05(clk, reset, mem_Write, dec_Out[5], 32'd05, data_In, Q_out5); 
	register_Mem_data r_06(clk, reset, mem_Write, dec_Out[6], 32'd06, data_In, Q_out6); 
	register_Mem_data r_07(clk, reset, mem_Write, dec_Out[7], 32'd07, data_In, Q_out7); 
	
	register_Mem_data r_08(clk, reset, mem_Write, dec_Out[8], 32'd08, data_In, Q_out8); 
	register_Mem_data r_09(clk, reset, mem_Write, dec_Out[9], 32'd09, data_In, Q_out9); 
	register_Mem_data r_10(clk, reset, mem_Write, dec_Out[10],32'd10, data_In, Q_out10);
	register_Mem_data r_11(clk, reset, mem_Write, dec_Out[11],32'd11, data_In, Q_out11);
	
	register_Mem_data r_12(clk, reset, mem_Write, dec_Out[12],32'd12, data_In, Q_out12);
	register_Mem_data r_13(clk, reset, mem_Write, dec_Out[13],32'd13, data_In, Q_out13);
	register_Mem_data r_14(clk, reset, mem_Write, dec_Out[14],32'd14, data_In, Q_out14);
	register_Mem_data r_15(clk, reset, mem_Write, dec_Out[15],32'd15, data_In, Q_out15);
	
	register_Mem_data r_16(clk, reset, mem_Write, dec_Out[16], 32'd16, data_In, Q_out16); 
	register_Mem_data r_17(clk, reset, mem_Write, dec_Out[17], 32'd17, data_In, Q_out17); 
	register_Mem_data r_18(clk, reset, mem_Write, dec_Out[18], 32'd18, data_In, Q_out18); 
	register_Mem_data r_19(clk, reset, mem_Write, dec_Out[19], 32'd19, data_In, Q_out19); 
	
	register_Mem_data r_20(clk, reset, mem_Write, dec_Out[20], 32'd20, data_In, Q_out20); 
	register_Mem_data r_21(clk, reset, mem_Write, dec_Out[21], 32'd21, data_In, Q_out21); 
	register_Mem_data r_22(clk, reset, mem_Write, dec_Out[22], 32'd22, data_In, Q_out22); 
	register_Mem_data r_23(clk, reset, mem_Write, dec_Out[23], 32'd23, data_In, Q_out23); 
	
	register_Mem_data r_24(clk, reset, mem_Write, dec_Out[24], 32'd24, data_In, Q_out24); 
	register_Mem_data r_25(clk, reset, mem_Write, dec_Out[25], 32'd25, data_In, Q_out25); 
	register_Mem_data r_26(clk, reset, mem_Write, dec_Out[26], 32'd26, data_In, Q_out26);
	register_Mem_data r_27(clk, reset, mem_Write, dec_Out[27], 32'd27, data_In, Q_out27);
	
	register_Mem_data r_28(clk, reset, mem_Write, dec_Out[28], 32'd28, data_In, Q_out28);
	register_Mem_data r_29(clk, reset, mem_Write, dec_Out[29], 32'd29, data_In, Q_out29);
	register_Mem_data r_30(clk, reset, mem_Write, dec_Out[30], 32'd30, data_In, Q_out30);
	register_Mem_data r_31(clk, reset, mem_Write, dec_Out[31], 32'd31, data_In, Q_out31);
	
	
	mux32to1 mux_Mem (Q_out0, Q_out1, Q_out2, Q_out3, Q_out4, Q_out5, Q_out6, Q_out7, Q_out8, Q_out9, Q_out10, Q_out11, Q_out12, Q_out13, Q_out14, Q_out15,Q_out16, Q_out17, Q_out18, Q_out19, Q_out20, Q_out21, Q_out22, Q_out23, Q_out24, Q_out25, Q_out26, Q_out27, Q_out28, Q_out29, Q_out30, Q_out31, pc[4:0],IR);
endmodule



module ALU(input signed [31:0] aluIn1, input signed [31:0] aluIn2, input [1:0] aluOp, output reg [31:0] aluOut, output reg signflag);
	always@(aluIn1 or aluIn2 or aluOp)
	begin
		assign signflag = 1'b0;
		case(aluOp)
			2'd0: aluOut = aluIn1 + aluIn2;
			2'd1:	begin 
						aluOut = aluIn1 - aluIn2;
						if(aluIn1 < aluIn2) signflag = 1'b1;
						else signflag = 1'b0;
					end
			2'd2: aluOut = aluIn1 >> aluIn2;
			2'd3: aluOut = aluIn1 ^ aluIn2;
		endcase
	end
endmodule
 
module register_IM16(input clk, input reset, input [15:0] d_in, output [15:0] q_out);
	D_ff_IM dIM0 (clk, reset, d_in[0], q_out[0]);
	D_ff_IM dIM1 (clk, reset, d_in[1], q_out[1]);
	D_ff_IM dIM2 (clk, reset, d_in[2], q_out[2]);
	D_ff_IM dIM3 (clk, reset, d_in[3], q_out[3]);
	D_ff_IM dIM4 (clk, reset, d_in[4], q_out[4]);
	D_ff_IM dIM5 (clk, reset, d_in[5], q_out[5]);
	D_ff_IM dIM6 (clk, reset, d_in[6], q_out[6]);
	D_ff_IM dIM7 (clk, reset, d_in[7], q_out[7]);
	D_ff_IM dIM8 (clk, reset, d_in[8], q_out[8]);
	D_ff_IM dIM9 (clk, reset, d_in[9], q_out[9]);
	D_ff_IM dIM10 (clk, reset, d_in[10], q_out[10]);
	D_ff_IM dIM11 (clk, reset, d_in[11], q_out[11]);
	D_ff_IM dIM12 (clk, reset, d_in[12], q_out[12]);
	D_ff_IM dIM13 (clk, reset, d_in[13], q_out[13]);
	D_ff_IM dIM14 (clk, reset, d_in[14], q_out[14]);
	D_ff_IM dIM15 (clk, reset, d_in[15], q_out[15]);
endmodule                         
 
 
module mux8to1_IM(input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
	outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
	input [2:0] Sel, output reg [47:0] outBus );
	
	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
		outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
		Sel)
		begin
		case (Sel)
				3'b000 : outBus = {outR0,outR1,outR2};
				3'b001 : outBus = {outR3,outR4,outR5};
				3'b010 : outBus = {outR6,outR7,outR8};
				3'b011 : outBus = {outR9,outR10,outR11};
				3'b100 : outBus = {outR12,outR13,outR14};
				3'b101 : outBus = {outR15,outR16,outR17};
				3'b110 : outBus = {outR18,outR19,outR20};
				3'b111 : outBus = {outR21,outR22,outR23};
		endcase
		end
endmodule
 
module IM(input clk, input reset, input [2:0] pc_3bits, output [47:0] IR);
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,
					Qout16, Qout17, Qout18, Qout19, Qout20, Qout21, Qout22, Qout23,
					Qout24, Qout25, Qout26, Qout27, Qout28, Qout29, Qout30, Qout31;
	register_IM16 rIM0 (clk, reset, 16'h0012, Qout0);//ADD 1 TO REG 4 AND SAVE IN REG 2
	register_IM16 rIM1 (clk, reset, 16'h0113, Qout1);
	register_IM16 rIM2 (clk, reset, 16'h0000, Qout2);
	register_IM16 rIM3 (clk, reset, 16'h0051, Qout3);// ADD 5 TO REG 3 AND SAVE IN REG 5
	register_IM16 rIM4 (clk, reset, 16'h8293, Qout4);
	register_IM16 rIM5 (clk, reset, 16'h0000, Qout5);
	register_IM16 rIM6 (clk, reset, 16'h00D6, Qout6); // XOR 00D TO REG 12 AND SAVE IN REG 13
	register_IM16 rIM7 (clk, reset, 16'h4693, Qout7);
	register_IM16 rIM8 (clk, reset, 16'h0000, Qout8);
	register_IM16 rIM9 (clk, reset, 16'h0046, Qout9);  // ADD 4 TO REG 12 AND SAVE IN REG 10
	register_IM16 rIM10 (clk, reset, 16'h0513, Qout10); 
	register_IM16 rIM11 (clk, reset, 16'h0000, Qout11); 
	register_IM16 rIM12 (clk, reset, 16'h0001, Qout12); //AUIPC 17 TO PC AND SAVE IN 9
	register_IM16 rIM13 (clk, reset, 16'h7497, Qout13); 
	register_IM16 rIM14 (clk, reset, 16'h0000, Qout14); 
	register_IM16 rIM15 (clk, reset, 16'h0022, Qout15);// SLTU REG 2 AND REG 5 AND 0 WILL BE SAVED IN REG 8
	register_IM16 rIM16 (clk, reset, 16'hB433, Qout16);
	register_IM16 rIM17 (clk, reset, 16'h0000, Qout17); // JUMP TO REG 7
	register_IM16 rIM18 (clk, reset, 16'h0000, Qout18);
	register_IM16 rIM19 (clk, reset, 16'h0000, Qout19);
	register_IM16 rIM20 (clk, reset, 16'h9702, Qout20);
	register_IM16 rIM21 (clk, reset, 16'h0000, Qout21);
	register_IM16 rIM22 (clk, reset, 16'h0000, Qout22);
	register_IM16 rIM23 (clk, reset, 16'h0000, Qout23);
	register_IM16 rIM24 (clk, reset, 16'h0000, Qout24);
	register_IM16 rIM25 (clk, reset, 16'h0000, Qout25); 		
	register_IM16 rIM26 (clk, reset, 16'h0000, Qout26); 	
	register_IM16 rIM27 (clk, reset, 16'h0000, Qout27); 	
	register_IM16 rIM28 (clk, reset, 16'h0000, Qout28); 
	register_IM16 rIM29 (clk, reset, 16'h0000, Qout29); 
	register_IM16 rIM30 (clk, reset, 16'h0000, Qout30); 	
	register_IM16 rIM31 (clk, reset, 16'h0000, Qout31); 		
	
	mux8to1_IM mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,
		Qout16,Qout17,Qout18,Qout19,Qout20,Qout21,Qout22,Qout23,Qout24,Qout25,Qout26,Qout27,Qout28,Qout29,Qout30,Qout31,
		pc_3bits[2:0],IR);
		
endmodule
 
module register32bit_2( input clk, input reset, input regWrite1, input regWrite2, input decOut1b1, input decOut1b2, input [31:0] writeData1, input [31:0] writeData2, output [31:0] outR);
	D_ff_2 d0(clk, reset, regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[0], writeData2[0], outR[0]);
	D_ff_2 d1(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[1], writeData2[1], outR[1]);
	D_ff_2 d2(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[2], writeData2[2], outR[2]);
	D_ff_2 d3(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[3], writeData2[3], outR[3]);
	D_ff_2 d4(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[4], writeData2[4], outR[4]);
	D_ff_2 d5(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[5], writeData2[5], outR[5]);
	D_ff_2 d6(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[6], writeData2[6], outR[6]);
	D_ff_2 d7(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[7], writeData2[7], outR[7]);
	D_ff_2 d8(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[8], writeData2[8], outR[8]);
	D_ff_2 d9(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[9], writeData2[9], outR[9]);
	D_ff_2 d10(clk, reset,regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[10], writeData2[10], outR[10]);
	D_ff_2 d11(clk, reset,regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[11], writeData2[11], outR[11]);
	D_ff_2 d12(clk, reset,regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[12], writeData2[12], outR[12]);
	D_ff_2 d13(clk, reset,regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[13], writeData2[13], outR[13]);
	D_ff_2 d14(clk, reset,regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[14], writeData2[14], outR[14]);
	D_ff_2 d15(clk, reset,regWrite1, regWrite2, decOut1b1, decOut1b2,  writeData1[15], writeData2[15], outR[15]);
	D_ff_2 d16(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[16], writeData2[16], outR[16]);
	D_ff_2 d17(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[17], writeData2[17], outR[17]);
	D_ff_2 d18(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[18], writeData2[18], outR[18]);
	D_ff_2 d19(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[19], writeData2[19], outR[19]);
	D_ff_2 d20(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[20], writeData2[20], outR[20]);
	D_ff_2 d21(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[21], writeData2[21], outR[21]);
	D_ff_2 d22(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[22], writeData2[22], outR[22]);
	D_ff_2 d23(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[23], writeData2[23], outR[23]);
	D_ff_2 d24(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[24], writeData2[24], outR[24]);
	D_ff_2 d25(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[25], writeData2[25], outR[25]);
	D_ff_2 d26(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[26], writeData2[26], outR[26]);
	D_ff_2 d27(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[27], writeData2[27], outR[27]);
	D_ff_2 d28(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[28], writeData2[28], outR[28]);
	D_ff_2 d29(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[29], writeData2[29], outR[29]);
	D_ff_2 d30(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[30], writeData2[30], outR[30]);
	D_ff_2 d31(clk, reset, regWrite1, regWrite2,decOut1b1, decOut1b2,  writeData1[31], writeData2[31], outR[31]);
endmodule
	
module register32bit( input clk, input reset, input regWrite, input decOut1b, input [31:0] writeData, output [31:0] outR);
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
	D_ff d16(clk, reset, regWrite, decOut1b, writeData[16], outR[16]);
	D_ff d17(clk, reset, regWrite, decOut1b, writeData[17], outR[17]);
	D_ff d18(clk, reset, regWrite, decOut1b, writeData[18], outR[18]);
	D_ff d19(clk, reset, regWrite, decOut1b, writeData[19], outR[19]);
	D_ff d20(clk, reset, regWrite, decOut1b, writeData[20], outR[20]);
	D_ff d21(clk, reset, regWrite, decOut1b, writeData[21], outR[21]);
	D_ff d22(clk, reset, regWrite, decOut1b, writeData[22], outR[22]);
	D_ff d23(clk, reset, regWrite, decOut1b, writeData[23], outR[23]);
	D_ff d24(clk, reset, regWrite, decOut1b, writeData[24], outR[24]);
	D_ff d25(clk, reset, regWrite, decOut1b, writeData[25], outR[25]);
	D_ff d26(clk, reset, regWrite, decOut1b, writeData[26], outR[26]);
	D_ff d27(clk, reset, regWrite, decOut1b, writeData[27], outR[27]);
	D_ff d28(clk, reset, regWrite, decOut1b, writeData[28], outR[28]);
	D_ff d29(clk, reset, regWrite, decOut1b, writeData[29], outR[29]);
	D_ff d30(clk, reset, regWrite, decOut1b, writeData[30], outR[30]);
	D_ff d31(clk, reset, regWrite, decOut1b, writeData[31], outR[31]);
endmodule
 
//check for pc register 
module register16bit(input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR);
	D_ff d0(clk,reset,regWrite,decOut1b,writeData[0],outR[0]);
	D_ff d1(clk,reset,regWrite,decOut1b,writeData[1],outR[1]);
	D_ff d2(clk,reset,regWrite,decOut1b,writeData[2],outR[2]);
	D_ff d3(clk,reset,regWrite,decOut1b,writeData[3],outR[3]);
	D_ff d4(clk,reset,regWrite,decOut1b,writeData[4],outR[4]);
	D_ff d5(clk,reset,regWrite,decOut1b,writeData[5],outR[5]);
	D_ff d6(clk,reset,regWrite,decOut1b,writeData[6],outR[6]);
	D_ff d7(clk,reset,regWrite,decOut1b,writeData[7],outR[7]);
	D_ff d8(clk,reset,regWrite,decOut1b,writeData[8],outR[8]);
	D_ff d9(clk,reset,regWrite,decOut1b,writeData[9],outR[9]);
	D_ff d10(clk,reset,regWrite,decOut1b,writeData[10],outR[10]);
	D_ff d11(clk,reset,regWrite,decOut1b,writeData[11],outR[11]);
	D_ff d12(clk,reset,regWrite,decOut1b,writeData[12],outR[12]);
	D_ff d13(clk,reset,regWrite,decOut1b,writeData[13],outR[13]);
	D_ff d14(clk,reset,regWrite,decOut1b,writeData[14],outR[14]);
	D_ff d15(clk,reset,regWrite,decOut1b,writeData[15],outR[15]);
endmodule
 
module registerSet(input clk, input reset, input regWrite1, input regWrite2,
   input [31:0] decOut1, input [31:0] decOut2, input [31:0] writeData1, input [31:0] writeData2,
	output [31:0] outR0, output [31:0] outR1, output [31:0] outR2, output [31:0] outR3,
	output [31:0] outR4, output [31:0] outR5, output [31:0] outR6, output [31:0] outR7,
	output [31:0] outR8, output [31:0] outR9, output [31:0] outR10, output [31:0] outR11,
	output [31:0] outR12, output [31:0] outR13, output [31:0] outR14, output [31:0] outR15,
	output [31:0] outR16, output [31:0] outR17, output [31:0] outR18, output [31:0] outR19,
	output [31:0] outR20, output [31:0] outR21, output [31:0] outR22, output [31:0] outR23,
	output [31:0] outR24, output [31:0] outR25, output [31:0] outR26, output [31:0] outR27,
	output [31:0] outR28, output [31:0] outR29, output [31:0] outR30, output [31:0] outR31);
 
	register32bit_2 r00 (clk, reset, regWrite1, regWrite2, decOut1[0], decOut2[0],  writeData1 ,writeData2, outR0); 	
	//check regWrite here
	register32bit_2 r01 (clk, reset, regWrite1, regWrite2, decOut1[1], decOut2[1], writeData1 , writeData2, outR1 );
	register32bit_2 r02 (clk, reset, regWrite1, regWrite2, decOut1[2], decOut2[2], writeData1 , writeData2 , outR2 );
	register32bit_2 r03 (clk, reset, regWrite1, regWrite2, decOut1[3], decOut2[3], writeData1 , writeData2 , outR3 );
	register32bit_2 r04 (clk, reset, regWrite1, regWrite2, decOut1[4], decOut2[4], writeData1 , writeData2 , outR4 );
	register32bit_2 r05 (clk, reset, regWrite1, regWrite2, decOut1[5], decOut2[5], writeData1 , writeData2 , outR5 );
	register32bit_2 r06 (clk, reset, regWrite1, regWrite2, decOut1[6], decOut2[6], writeData1 , writeData2 , outR6 );
	register32bit_2 r07 (clk, reset, regWrite1, regWrite2, decOut1[7], decOut2[7], writeData1 , writeData2 , outR7 );
	register32bit_2 r08 (clk, reset, regWrite1, regWrite2, decOut1[8], decOut2[8], writeData1 , writeData2 , outR8);
	register32bit_2 r09 (clk, reset, regWrite1, regWrite2, decOut1[9], decOut2[9], writeData1 , writeData2 , outR9 );
	register32bit_2 r10 (clk, reset, regWrite1, regWrite2, decOut1[10], decOut2[10], writeData1 , writeData2 , outR10 );
	register32bit_2 r11 (clk, reset, regWrite1, regWrite2, decOut1[11], decOut2[11], writeData1 , writeData2 , outR11 );
	register32bit_2 r12 (clk, reset, regWrite1, regWrite2, decOut1[12], decOut2[12], writeData1 , writeData2 , outR12 );
	register32bit_2 r13 (clk, reset, regWrite1, regWrite2, decOut1[13], decOut2[13], writeData1 , writeData2 , outR13 );
	register32bit_2 r14 (clk, reset, regWrite1, regWrite2, decOut1[14], decOut2[14], writeData1 , writeData2 , outR14 );
	register32bit_2 r15 (clk, reset, regWrite1, regWrite2, decOut1[15], decOut2[15], writeData1 , writeData2 , outR15 );
	register32bit_2 r16 (clk, reset, regWrite1, regWrite2, decOut1[16], decOut2[16],  writeData1 ,writeData2 , outR16 );
	register32bit_2 r17 (clk, reset, regWrite1, regWrite2, decOut1[17], decOut2[17], writeData1 , writeData2 , outR17 );
	register32bit_2 r18 (clk, reset, regWrite1, regWrite2, decOut1[18], decOut2[18], writeData1 , writeData2 , outR18 );
	register32bit_2 r19 (clk, reset, regWrite1, regWrite2, decOut1[19], decOut2[19], writeData1 , writeData2 , outR19 );
	register32bit_2 r20 (clk, reset, regWrite1, regWrite2, decOut1[20], decOut2[20], writeData1 , writeData2 , outR20 );
	register32bit_2 r21 (clk, reset, regWrite1, regWrite2, decOut1[21], decOut2[21], writeData1 , writeData2 , outR21 );
	register32bit_2 r22 (clk, reset, regWrite1, regWrite2, decOut1[22], decOut2[22], writeData1 , writeData2 , outR22 );
	register32bit_2 r23 (clk, reset, regWrite1, regWrite2, decOut1[23], decOut2[23], writeData1 , writeData2 , outR23 );
	register32bit_2 r24 (clk, reset, regWrite1, regWrite2, decOut1[24], decOut2[24], writeData1 , writeData2 , outR24 );
	register32bit_2 r25 (clk, reset, regWrite1, regWrite2, decOut1[25], decOut2[25], writeData1 , writeData2 , outR25 );
	register32bit_2 r26 (clk, reset, regWrite1, regWrite2, decOut1[26], decOut2[26], writeData1 , writeData2 , outR26 );
	register32bit_2 r27 (clk, reset, regWrite1, regWrite2, decOut1[27], decOut2[27], writeData1 , writeData2 , outR27 );
	register32bit_2 r28 (clk, reset, regWrite1, regWrite2, decOut1[28], decOut2[28], writeData1 , writeData2 , outR28 );
	register32bit_2 r29 (clk, reset, regWrite1, regWrite2, decOut1[29], decOut2[29], writeData1 , writeData2 , outR29 );
	register32bit_2 r30 (clk, reset, regWrite1, regWrite2, decOut1[30], decOut2[30], writeData1 , writeData2 , outR30 );
	register32bit_2 r31 (clk, reset, regWrite1, regWrite2, decOut1[31], decOut2[31], writeData1 , writeData2 , outR31 );
	
endmodule
 
 
module decoder5to32(input [4:0] destReg, output reg [31:0] decOut);
	always @(destReg)
		case(destReg)
			5'd00 : decOut = 32'b0000_0000_0000_0000_0000_0000_0000_0001;
			5'd01 : decOut = 32'b0000_0000_0000_0000_0000_0000_0000_0010;
			5'd02 : decOut = 32'b0000_0000_0000_0000_0000_0000_0000_0100;
			5'd03 : decOut = 32'b0000_0000_0000_0000_0000_0000_0000_1000;
			5'd04 : decOut = 32'b0000_0000_0000_0000_0000_0000_0001_0000;
			5'd05 : decOut = 32'b0000_0000_0000_0000_0000_0000_0010_0000;
			5'd06 : decOut = 32'b0000_0000_0000_0000_0000_0000_0100_0000;
			5'd07 : decOut = 32'b0000_0000_0000_0000_0000_0000_1000_0000;
			5'd08 : decOut = 32'b0000_0000_0000_0000_0000_0001_0000_0000;
			5'd09 : decOut = 32'b0000_0000_0000_0000_0000_0010_0000_0000;
			5'd10 : decOut = 32'b0000_0000_0000_0000_0000_0100_0000_0000;
			5'd11 : decOut = 32'b0000_0000_0000_0000_0000_1000_0000_0000;
			5'd12 : decOut = 32'b0000_0000_0000_0000_0001_0000_0000_0000;
			5'd13 : decOut = 32'b0000_0000_0000_0000_0010_0000_0000_0000;
			5'd14 : decOut = 32'b0000_0000_0000_0000_0100_0000_0000_0000;
			5'd15 : decOut = 32'b0000_0000_0000_0000_1000_0000_0000_0000;
			
			5'd16 : decOut = 32'b0000_0000_0000_0001_0000_0000_0000_0000;
			5'd17 : decOut = 32'b0000_0000_0000_0010_0000_0000_0000_0000;
			5'd18 : decOut = 32'b0000_0000_0000_0100_0000_0000_0000_0000;
			5'd19 : decOut = 32'b0000_0000_0000_1000_0000_0000_0000_0000;
			5'd20 : decOut = 32'b0000_0000_0001_0000_0000_0000_0000_0000;
			5'd21 : decOut = 32'b0000_0000_0010_0000_0000_0000_0000_0000;
			5'd22 : decOut = 32'b0000_0000_0100_0000_0000_0000_0000_0000;
			5'd23 : decOut = 32'b0000_0000_1000_0000_0000_0000_0000_0000;
			5'd24 : decOut = 32'b0000_0001_0000_0000_0000_0000_0000_0000;
			5'd25 : decOut = 32'b0000_0010_0000_0000_0000_0000_0000_0000;
			5'd26 : decOut = 32'b0000_0100_0000_0000_0000_0000_0000_0000;
			5'd27 : decOut = 32'b0000_1000_0000_0000_0000_0000_0000_0000;
			5'd28 : decOut = 32'b0001_0000_0000_0000_0000_0000_0000_0000;
			5'd29 : decOut = 32'b0010_0000_0000_0000_0000_0000_0000_0000;
			5'd30 : decOut = 32'b0100_0000_0000_0000_0000_0000_0000_0000;
			5'd31 : decOut = 32'b1000_0000_0000_0000_0000_0000_0000_0000;
			
		endcase
endmodule

module decoder4to16(input [3:0] destReg, output reg [15:0] decOut);
	always @(destReg)
		case(destReg)
			4'd00 : decOut = 16'b0000_0000_0000_0001;
			4'd01 : decOut = 16'b0000_0000_0000_0010;
			4'd02 : decOut = 16'b0000_0000_0000_0100;
			4'd03 : decOut = 16'b0000_0000_0000_1000;
			4'd04 : decOut = 16'b0000_0000_0001_0000;
			4'd05 : decOut = 16'b0000_0000_0010_0000;
			4'd06 : decOut = 16'b0000_0000_0100_0000;
			4'd07 : decOut = 16'b0000_0000_1000_0000;
			4'd08 : decOut = 16'b0000_0001_0000_0000;
			4'd09 : decOut = 16'b0000_0010_0000_0000;
			4'd10 : decOut = 16'b0000_0100_0000_0000;
			4'd11 : decOut = 16'b0000_1000_0000_0000;
			4'd12 : decOut = 16'b0001_0000_0000_0000;
			4'd13 : decOut = 16'b0010_0000_0000_0000;
			4'd14 : decOut = 16'b0100_0000_0000_0000;
			4'd15 : decOut = 16'b1000_0000_0000_0000;
			
		endcase
endmodule
 
module mux32to1(input [31:0] outR0, input [31:0] outR1, input [31:0] outR2, input [31:0] outR3,
	input [31:0] outR4, input [31:0] outR5, input [31:0] outR6, input [31:0] outR7,
	input [31:0] outR8, input [31:0] outR9, input [31:0] outR10, input [31:0] outR11,
	input [31:0] outR12, input [31:0] outR13, input [31:0] outR14, input [31:0] outR15,
	input [31:0] outR16, input [31:0] outR17, input [31:0] outR18, input [31:0] outR19,
	input [31:0] outR20, input [31:0] outR21, input [31:0] outR22, input [31:0] outR23,
	input [31:0] outR24, input [31:0] outR25, input [31:0] outR26, input [31:0] outR27,
	input [31:0] outR28, input [31:0] outR29, input [31:0] outR30, input [31:0] outR31,
	input [4:0] Sel, output reg [31:0] outBus);
 
	always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
	outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR31,Sel)
		case(Sel)
			5'd0 : outBus = outR0;
			5'd1 : outBus = outR1;
			5'd2 : outBus = outR2;
			5'd3 : outBus = outR3;
			5'd4 : outBus = outR4;
			5'd5 : outBus = outR5;
			5'd6 : outBus = outR6;
			5'd7 : outBus = outR7;
			5'd8 : outBus = outR8;
			5'd9 : outBus = outR9;
			5'd10 : outBus = outR10;
			5'd11 : outBus = outR11;
			5'd12 : outBus = outR12;
			5'd13 : outBus = outR13;
			5'd14 : outBus = outR14;
			5'd15 : outBus = outR15;
			
			5'd16 : outBus = outR16;
			5'd17 : outBus = outR17;
			5'd18 : outBus = outR18;
			5'd19 : outBus = outR19;
			5'd20 : outBus = outR20;
			5'd21 : outBus = outR21;
			5'd22 : outBus = outR22;
			5'd23 : outBus = outR23;
			5'd24 : outBus = outR24;
			5'd25 : outBus = outR25;
			5'd26 : outBus = outR26;
			5'd27 : outBus = outR27;
 			5'd28 : outBus = outR28;
			5'd29 : outBus = outR29;
			5'd30 : outBus = outR30;
			5'd31 : outBus = outR31;
		endcase                 
endmodule                 
  
//Check  
module mux2to1_32bits(input [31:0] in0, input [31:0] in1, input Sel, output reg [31:0] outBus);
	always@(in0,in1,Sel)
		case(Sel)
			1'd0 : outBus=in0;
			1'd1:outBus=in1;
		endcase
endmodule
 
module sub(input [31:0] in1, input [31:0] in2, output reg [31:0] sub_out);
	always@(in1 or in2)
	begin
		sub_out=in1-in2;
	end
endmodule

module registerFile(input clk, input reset, input regWriteA, input regWriteB,
	input [4:0] rs1A, input [4:0] rs2A,
	input [4:0] rs1B, input [4:0] rs2B,
	input [4:0] rdA, input [4:0] rdB,
	input [31:0] writeDataA,input [31:0] writeDataB,
	output [31:0] regRs1A, output [31:0] regRs2A,
	output [31:0] regRs1B, output [31:0] regRs2B);
	
	wire [31:0] decOutA, decOutB;
	wire [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
					outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR31;
	

	//For writing into register file
	decoder5to32 d0(rdA,decOutA);
	decoder5to32 d1(rdB,decOutB);
	
	registerSet rSet (clk, reset, regWriteA, regWriteB, decOutA, decOutB, writeDataA, writeDataB, 
							outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,
							outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
							outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,
							outR24,outR25,outR26,outR27,outR28,outR29,outR0,outR31);
							
	mux32to1 m00(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,
							outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
							outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,
							outR24,outR25,outR26,outR27,outR28,outR29,outR0,outR31,rs1A,regRs1A);
							
	mux32to1 m01(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,
							outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
							outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,
							outR24,outR25,outR26,outR27,outR28,outR29,outR0,outR31,rs2A,regRs2A);
							
	mux32to1 m02(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,
							outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
							outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,
							outR24,outR25,outR26,outR27,outR28,outR29,outR0,outR31,rs1B,regRs1B);
							
	mux32to1 m03(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,
							outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
							outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,
							outR24,outR25,outR26,outR27,outR28,outR29,outR0,outR31,rs2B,regRs2B);
endmodule	
 
 
// Register Memory Ends
 
//6 bit register

 
//5 bit register
module register5bit( input clk, input reset, input regWrite, input decOut1b, input [4:0] writeData, output  [4:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
endmodule
 
//10 bit register
module register10bit( input clk, input reset, input regWrite, input decOut1b, input [9:0] writeData, output  [9:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
endmodule
 
//2 bit register
module register2bit( input clk, input reset, input regWrite, input decOut1b, input [1:0] writeData, output  [1:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
endmodule
 
//1 bit register
module register1bit( input clk, input reset, input regWrite, input decOut1b, input writeData, output outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData, outR);
endmodule
 
module zeroExt5to32(input [4:0] in, output reg [31:0] zeroExtin);
	always@(in)
		zeroExtin={27'b0,in};
endmodule
 
module zeroExt8to32(input [7:0] in, output reg [31:0] zeroExtin);
	always@(in)
		zeroExtin={24'b0,in};
endmodule

 
 //cancat for Auipc
module zeroExt20to32(input [19:0] in, output reg [31:0] zeroExtin);
	always@(in)
		zeroExtin={in,12'b0};
endmodule
 
module zeroExt22to32(input [21:0] in, output reg [31:0] zeroExtin);
	always@(in)
		zeroExtin={10'b0,in};
endmodule
 
module signExt12to32(input [11:0] in, output reg [31:0] signExtin);
	always@(in)
		signExtin={{20{in[11]}},in};
endmodule
 
module signExt8to32(input [7:0] in, output reg [31:0] signExtin);
	always@(in)
		signExtin={{24{in[7]}},in};
endmodule
 
 
module mux4to1_32bits(input [31:0] in0, input [31:0] in1, input [31:0] in2, input [31:0] in3, input [1:0] sel, output reg [31:0] muxOut);
    always@(in0, in1, in2, in3, sel)
		begin
			case(sel)
				2'd0: muxOut=in0;
				2'd1: muxOut=in1;
				2'd2: muxOut=in2;
				2'd3: muxOut=in3;
			endcase
		end
endmodule
 
//check flush
module IF_ID(input clk, input reset,input pregWrite, input decOut1b, input [31:0] pc, input [31:0] instr1, input [15:0] instr2, output [31:0] p0_instr1, output [15:0] p0_instr2, output [31:0] p0_pc);
	register32bit reginstr1( clk, reset , pregWrite, decOut1b, instr1, p0_instr1 );
	register16bit reginstr2( clk, reset , pregWrite, decOut1b, instr2, p0_instr2 );
	register32bit regpc( clk, reset , pregWrite, decOut1b, pc, p0_pc );
endmodule
 
//aluSrc1 for pipeline 1 and aluSrc2 for pileline 2
 
module ID_EX(input clk, input reset,input pregWrite, input ID_Flush, input decOut1b,
	input [31:0] PC,
	input [31:0] IRA, input [15:0] IRB,
	input flagWriteA,	input flagWriteB,
	input [31:0] reg_rs1A,input [31:0] reg_rs2A, input [31:0] reg_rs1B, input [31:0] reg_rs2B,
	input [4:0] rdA, input [4:0] rdB,
	input [4:0] rs1A, input [4:0] rs1B,
	input [4:0] rs2A, input [4:0] rs2B,
	input aluSrc1A, input [1:0] aluSrc2A, 
	input memReadA, input memWriteA,	input memReadB, input memWriteB,
	input [1:0] aluOpA,
	input [31:0] zeroExtend5to32A, input[31:0] signExtend12to32A,input [31:0] zeroExtend22to32B , input [31:0] signExtend20to32A,
	input [9:0] opcodeA, input [1:0] opcodeB, input [1:0] MemToReg,
	input [31:0] branchadderoutB,
	
	output [31:0] PC_out,
	output [31:0] IRA_out, output [15:0] IRB_out,
	output flagWriteA_out, output flagWriteB_out,
	output [4:0] rs2A_out, output [4:0] rs2B_out,
	output [31:0] reg_rs1A_out,output [31:0] reg_rs2A_out, output [31:0] reg_rs1B_out, output [31:0] reg_rs2B_out,
	output [4:0] rdA_out,output [4:0] rdB_out,
	output [4:0] rs1A_out, output [4:0] rs1B_out,
	output aluSrc1A_out, output [1:0] aluSrc2A_out, 
	output memReadA_out, output memWriteA_out, output memReadB_out, output memWriteB_out,
	output [1:0] aluOpA_out,
	output [31:0] zeroExtend5to32A_out, output[31:0] signExtend12to32A_out, output [31:0] zeroExtend22to32B_out, output [31:0] signExtend20to32A_out,
	output [9:0] opcodeA_out, output [1:0] opcodeB_out, output [1:0] MemToReg_out, output [31:0] branchadderoutB_out);
	
	register32bit Regpc( clk, reset, pregWrite, decOut1b, PC, PC_out );
	register32bit Regbranch( clk, reset, pregWrite, decOut1b, branchadderoutB, branchadderoutB_out );	//forwarding the branch register value
	
	register1bit RegDestA(clk, reset, pregWrite, decOut1b, DestA, DestA_out);
	register1bit RegDestB(clk, reset, pregWrite, decOut1b, DestB, DestB_out);
 
	
	register32bit RegIRA( clk, reset, pregWrite, decOut1b, IRA, IRA_out );
	register16bit RegIRB( clk, reset, pregWrite, decOut1b, IRB, IRB_out );
	
	register1bit RegflagWriteA(clk, reset | ID_Flush, pregWrite, decOut1b, flagWriteA, flagWriteA_out);
	register1bit RegflagWriteB(clk, reset | ID_Flush, pregWrite, decOut1b, flagWriteB, flagWriteB_out);
	
	register32bit regrs1A( clk, reset , pregWrite, decOut1b, reg_rs1A, reg_rs1A_out );
	register32bit regrs2A( clk, reset , pregWrite, decOut1b, reg_rs2A, reg_rs2A_out );
	register32bit regrs1B( clk, reset , pregWrite, decOut1b, reg_rs1B, reg_rs1B_out );
	register32bit regrs2B( clk, reset , pregWrite, decOut1b, reg_rs2B, reg_rs2B_out );
	
	register5bit RdA( clk, reset , pregWrite, decOut1b, rdA, rdA_out );
	register5bit RdB( clk, reset , pregWrite, decOut1b, rdA, rdB_out );
	
	register1bit RegaluSrc1A(clk, reset | ID_Flush, pregWrite, decOut1b, aluSrc1A, aluSrc1A_out);
	register2bit RegaluSrc2A(clk, reset | ID_Flush, pregWrite, decOut1b, aluSrc2A, aluSrc2A_out);
	
	register2bit RegaluOpA(clk, reset, pregWrite, decOut1b, aluOpA, aluOpA_out);
 
 
	register5bit Rs2A( clk, reset | ID_Flush, pregWrite, decOut1b, rs2A, rs2A_out );
	register5bit Rs2B(clk, reset | ID_Flush, pregWrite, decOut1b, rs2B, rs2B_out );
	register5bit Rs1A( clk, reset | ID_Flush, pregWrite, decOut1b, rs1A, rs1A_out );
	register5bit Rs1B(clk, reset | ID_Flush, pregWrite, decOut1b, rs1B, rs1B_out );
	
	register1bit RegmemReadA(clk, reset, pregWrite, decOut1b, memReadA, memReadA_out);
	register1bit RegmemReadB(clk, reset, pregWrite, decOut1b, memReadB, memReadB_out);
	register1bit RegmemWriteA(clk, reset, pregWrite, decOut1b, memWriteA, memWriteA_out);
	register1bit RegmemWriteB(clk, reset, pregWrite, decOut1b, memWriteB, memWriteB_out);
	
	register32bit RegzeroExtend5to32A( clk, reset | ID_Flush, pregWrite, decOut1b, zeroExtend5to32A, zeroExtend5to32A_out );
	register32bit RegsignExtend12to32A( clk, reset | ID_Flush, pregWrite, decOut1b, signExtend12to32A, signExtend12to32A_out );
	register32bit RegzeroExtend22to32B( clk, reset | ID_Flush, pregWrite, 1'b1, zeroExtend22to32B, zeroExtend22to32B_out );
//	register32bit RegsignExtend8to32B( clk, reset | ID_Flush, pregWrite, decOut1b, signExtend8to32B, signExtend8to32B_out );
	register32bit RegsignExtend20to32A( clk, reset | ID_Flush, pregWrite, decOut1b, signExtend20to32A, signExtend20to32A_out );
 
	
	register2bit RegopcodeB(clk, reset | ID_Flush, pregWrite, decOut1b, opcodeB, opcodeB_out);
	register10bit RegopcodeA( clk, reset | ID_Flush, pregWrite, decOut1b, opcodeA, opcodeA_out );
	
	register2bit RegMemToReg(clk, reset | ID_Flush, pregWrite, decOut1b, MemToReg, MemToReg_out);
	
	
endmodule
	
	//takebranch flag needed for branch to take or not
module EX_MEM(
	input clk, input reset, input regWrite, input decOut1b,
	input signFlagA,
	input [31:0] aluoutA,input [31:0] aluoutB, input [31:0] branchadderoutB,
	input [31:0] regrs2B, input [4:0] rdA, 
	input memReadA, input memWriteA,	input memReadB, input memWriteB, 
	input flagWriteA,	input flagWriteB, input [1:0] MemToReg,
 
	output signFlagA_out,
	output [31:0] aluoutA_out, output [31:0] aluoutB_out, output [31:0] branchadderoutB_out,
	output [31:0] regrs2B_out, output [4:0] rdA_out,
	output memReadA_out, output memWriteA_out, output memReadB_out, output memWriteB_out,
	output flagWriteA_out, output flagWriteB_out, output [1:0] MemToReg_out);
	
	register1bit RegsignFlagA(clk, reset, regWrite, decOut1b, signFlagA, signFlagA_out);
	
	register32bit RegaluoutA( clk, reset , regWrite, decOut1b, aluoutA, aluoutA_out );
	register32bit RegaluoutB( clk, reset , regWrite, decOut1b, aluoutB, aluoutB_out );
	register32bit RegbranchadderoutB( clk, reset , regWrite, decOut1b, branchadderoutB, branchadderoutB_out );
	
	register32bit Regrs2B( clk, reset , regWrite, decOut1b, regrs2B, regrs2B_out );
	register5bit RegrdA( clk, reset , regWrite, decOut1b, rdA, rdA_out );
	
	register1bit RegmemReadA(clk, reset, regWrite, decOut1b, memReadA, memReadA_out);
	register1bit RegmemReadB(clk, reset, regWrite, decOut1b, memReadB, memReadB_out);
	register1bit RegmemWriteA(clk, reset, regWrite, decOut1b, memWriteA, memWriteA_out);
	register1bit RegmemWriteB(clk, reset, regWrite, decOut1b, memWriteB, memWriteB_out);
	
	register1bit RegflagWriteA(clk, reset, regWrite, decOut1b, flagWriteA, flagWriteA_out);
	register1bit RegflagWriteB(clk, reset, regWrite, decOut1b, flagWriteB, flagWriteB_out);
	
	register2bit RegMemToReg(clk, reset, regWrite, decOut1b, MemToReg, MemToReg_out);
	endmodule
 
module MEM_WB(input clk, input reset,input regWrite,  input decOut1b,
					input [4:0] rdA, input [31:0] writeDataA,input [31:0] writeDataB, input regWriteAout_EXMEM,input regWriteBout_EXMEM,
					output [4:0] rdA_out, output [31:0] writeData_outA,output [31:0] writeData_outB, output regWriteAout_MEMWB, output regWriteBout_MEMWB);
	
	register32bit RegwriteDataA( clk, reset , regWrite, decOut1b, writeDataA, writeData_outA);
	register32bit RegwriteDataB( clk, reset , regWrite, decOut1b, writeDataB, writeData_outB);
	register5bit RegrdA( clk, reset , regWrite, decOut1b, rdA, rdA_out );
	register1bit RegwriteA( clk, reset , regWrite, decOut1b, regWriteAout_EXMEM, regWriteAout_MEMWB );
	register1bit RegwriteB( clk, reset , regWrite, decOut1b, regWriteBout_EXMEM, regWriteBout_MEMWB);
endmodule
 

 
module PC_Adder(input [31:0] in1, input [31:0] in2, output reg [31:0] adder_out);
	always@(in1 or in2)
	begin
		adder_out = in1 + in2;
	end
endmodule

module comparator(input [31:0] in1, input [31:0] in2, output reg comparator_out);
	always@(in1 or in2)
	begin
		if(in1 == in2)
			comparator_out = 1'b1;
		else
			comparator_out = 1'b0;
	end
endmodule

//opcode1 for 32 bit instr combined for opcode+function
module ctrlCkt (input [9:0] opcode1, input [1:0] opcode2, output reg aluSrc1A, output reg [1:0] aluSrc2A, output reg regWrite,
 output reg memReadA, output reg memWriteA, output reg memReadB, output reg memWriteB,
 output reg regWriteA, output reg regWriteB, output reg [1:0] aluOp1, output reg [1:0] memToReg,
 output reg PCSel, output reg branchSel,
 output reg ID_Flush,output  reg dmsel);
	initial
	begin
	aluSrc1A = 1'b0;
	aluSrc2A = 2'b00;
	memReadA = 1'b0;
	memReadB = 1'b0;
	memWriteA = 1'b0;
	memWriteB = 1'b0;
	regWriteA = 1'b0;
	regWriteB = 1'b0;
	aluOp1 = 2'b00;
	memToReg = 2'b00;
	PCSel = 1'b0;
	branchSel = 1'b0;
	ID_Flush = 1'b0;
	regWrite = 1'b1;
	dmsel =1'b0;
	end
	always@(opcode1)
	begin
	aluSrc1A = 1'b0;
	aluSrc2A = 2'b00;
	memReadA = 1'b0;
	memReadB = 1'b0;
	regWriteA = 1'b0;
	regWriteB = 1'b0;
	aluOp1 = 2'b00;
	memToReg = 2'b00;
	ID_Flush = 1'b0;
	regWrite = 1'b1;
		if(opcode1 == 10'b0000010011) //addi
			begin
				aluSrc1A = 1'b1;
				aluOp1 = 2'b00;
				memToReg = 2'b00;
				aluSrc2A = 2'b10;
				regWriteA = 1'b1;
				branchSel = 1'b0;
			end
		else if(opcode1 == 10'b1010010011) //SRLI
			begin
				aluSrc1A = 1'b1;
				aluOp1 = 2'd2;
				memToReg = 2'b00;
				aluSrc2A = 2'b01;
				regWriteA = 1'b1;
				branchSel = 1'b0;
				memToReg=2'd0;
				
				
			end
		else if(opcode1 == 10'b0110110011) //sltu
			begin
				aluSrc1A = 1'b1;
				aluOp1 = 2'b01;
				memToReg = 2'b01;
				aluSrc2A = 2'b00;
				regWriteA = 1'b1;
				branchSel = 1'b0;
			end
		else if(opcode1 == 10'b1000000011) //lbu
			begin
				aluSrc1A = 1'b1;
				aluOp1 = 2'b00;
				memToReg = 2'b10;
				aluSrc2A = 2'b10;
				memReadA=1'b1;
				regWriteA = 1'b1;
				branchSel = 1'b0;
				dmsel=1'b0;

			end
		else if(opcode1 == 10'b1000010011) //xori
			begin
				aluSrc1A = 1'b1;
				aluOp1 = 2'b11;
				memToReg = 2'b00;
				aluSrc2A = 2'b10;
				regWriteA = 1'b1;
				branchSel = 1'b0;
			end
		else if(opcode1[6:0]==7'b0010111) //auipc 
			begin
				aluSrc1A = 1'b0;
				aluOp1 = 2'b00;
				memToReg = 2'b00;
				aluSrc2A = 2'b11;
				regWriteA = 1'b1;
				branchSel = 1'b0;
			end	
	end
	
	always@(opcode2)
	begin 
	aluSrc1A = 1'b0;
	aluSrc2A = 2'b00;
	memReadA = 1'b0;
	memReadB = 1'b0;
	memWriteA = 1'b0;
	memWriteB = 1'b0;
	regWriteA = 1'b0;
	regWriteB = 1'b0;
	aluOp1 = 2'b00;
	memToReg = 2'b00;
	PCSel = 1'b0;
	branchSel = 1'b0;
	ID_Flush = 1'b0;
	regWrite = 1'b1;
	dmsel=1'b0;

		if (opcode2==2'b10)  //JALR
			begin
				PCSel = 1'b1;
				branchSel = 1'b0;
				ID_Flush=1'b1;
				regWriteA = 1'b0;
				regWriteB = 1'b1;
			end
		else  if (opcode2==2'b01) //beq
			begin
				branchSel = 1'b1;

			end
		else  if (opcode2==2'b00)  //sw
			begin
				//lite lete
			dmsel=1'b1;
			end
	
	end
endmodule

/*module WB_END(input clk, input reset, input write,input decout1b, input [4:0] rdA_out, input [31:0] writeData_outA,input [4:0] rdB_out,input [31:0] writeData_outB, input regWriteAout_MEMWB, input regWriteBout_MEMWB,
output [4:0] rdA_END, output [31:0] writeData_A_END, output [4:0] rdB_END,output [31:0] writeData_B_END, output regWriteAout_END, output regWriteBout_END);

	register32bit dataA(clk,reset,write,decout1b, writeData_outA,writeData_A_END);
	register32bit dataB(clk,reset,write,decout1b,writeData_outB, writeData_B_END);
	register5bit regrdAA(clk,reset,write,decout1b, rdA_out, rdA_END);
	register5bit regrdBB(clk,reset,write,decout1b, rdB_out, rdB_END);
	register1bit regwr_enA(clk,reset,write,decout1b,regWriteAout_MEMWB, regWriteAout_END);
	register1bit regwr_enB(clk,reset,write,decout1b,regWriteBout_MEMWB, regWriteBout_END);

endmodule*/

/* module forward(input clk, input reset, 
	input ex_mem_reg_wr_A, input [4:0] ex_mem_regrd_A,
	input mem_wb_reg_wr_A, input [4:0] mem_wb_regrd_A,
	input wb_end_reg_wr_A, input [4:0] wb_end_regrd_A,
	input ex_mem_reg_wr_B, input [4:0] ex_mem_regrd_B,
	input mem_wb_reg_wr_B, input [4:0] mem_wb_regrd_B,
	input wb_end_reg_wr_B, input [4:0] wb_end_regrd_B,
	
	input [4:0] rs1A, input [4:0] rs2A, input [4:0] rs1B, input [4:0] rs2B,
	output reg [2:0] selrs1_A, output reg [2:0] selrs2_A, output reg [2:0] selrs1_B, output reg [2:0] selrs2_B);
	
	always @(negedge clk) begin
		// RS1_A forward signal decision rs1A
		
		if(ex_mem_reg_wr_A == 1'b1 && ex_mem_regrd_A == rs1A)
			selrs1_A = 3'b001; // select from ex_mem pipeline of 32bit inst;
		else if(ex_mem_reg_wr_B == 1'b1 && ex_mem_regrd_B == rs1A)
			selrs1_A = 3'b010;		// select from ex_mem pipeline of 16 bit
		else if(mem_wb_reg_wr_A == 1'b1 && mem_wb_regrd_A == rs1A)
			selrs1_A = 3'b011;		// select from mem_wb pipeline of 32bit
		else if(mem_wb_reg_wr_B == 1'b1 && mem_wb_regrd_B == rs1A)
			selrs1_A = 3'b100;		// select from mem_wb of 16bit
		else if(wb_end_reg_wr_A == 1'b1 && wb_end_regrd_A == rs1A)
			selrs1_A = 3'b101;		// select from last ie WB_end stage of 32 bit
		else if(wb_end_reg_wr_B == 1'b1 && wb_end_regrd_B == rs1A)
			selrs1_A = 3'b110;		// select from last we_end stage of 16 bit
		else
			selrs1_A = 3'b000;	// NO conflict Ignore and take default  value from id_ex pipeline;
	end
	
	// RS2_A selection
	always @(negedge clk) begin
		if(ex_mem_reg_wr_A == 1'b1 && ex_mem_regrd_A == rs2A)
			selrs2_A = 3'b001;
		else if(ex_mem_reg_wr_B == 1'b1 && ex_mem_regrd_B == rs2A)
			selrs2_A = 3'b010;
		else if(mem_wb_reg_wr_A == 1'b1 && mem_wb_regrd_A == rs2A)
			selrs2_A = 3'b011;
		else if(mem_wb_reg_wr_B == 1'b1 && mem_wb_regrd_B == rs2A)
			selrs2_A = 3'b100;
		else if(wb_end_reg_wr_A == 1'b1 && wb_end_regrd_A == rs2A)
			selrs2_A = 3'b101;
		else if(wb_end_reg_wr_B == 1'b1 && wb_end_regrd_B == rs2A)
			selrs2_A = 3'b110;
		else
			selrs2_A = 3'b000;
	end
	
	// RS1_B selection
	
	always @(negedge clk) begin
		
		if(ex_mem_reg_wr_A == 1'b1 && ex_mem_regrd_A == rs1B)
			selrs1_B = 3'b001;
		else if(ex_mem_reg_wr_B == 1'b1 && ex_mem_regrd_B == rs1B)
			selrs1_B = 3'b010;
		else if(mem_wb_reg_wr_A == 1'b1 && mem_wb_regrd_A == rs1B)
			selrs1_B = 3'b011;
		else if(mem_wb_reg_wr_B == 1'b1 && mem_wb_regrd_B == rs1B)
			selrs1_B = 3'b100;
		else if(wb_end_reg_wr_A == 1'b1 && wb_end_regrd_A == rs1B)
			selrs1_B = 3'b101;
		else if(wb_end_reg_wr_B == 1'b1 && wb_end_regrd_B == rs1B)
			selrs1_B = 3'b110;
		else
			selrs1_B = 3'b000;
	end
	
	// RS2_B selection
	always @(negedge clk) begin
		if(ex_mem_reg_wr_A == 1'b1 && ex_mem_regrd_A == rs2B)
			selrs2_A = 3'b001;
		else if(ex_mem_reg_wr_B == 1'b1 && ex_mem_regrd_B == rs2B)
			selrs2_A = 3'b010;
		else if(mem_wb_reg_wr_A == 1'b1 && mem_wb_regrd_A == rs2B)
			selrs2_A = 3'b011;
		else if(mem_wb_reg_wr_B == 1'b1 && mem_wb_regrd_B == rs2B)
			selrs2_A = 3'b100;
		else if(wb_end_reg_wr_A == 1'b1 && wb_end_regrd_A == rs2B)
			selrs2_A = 3'b101;
		else if(wb_end_reg_wr_B == 1'b1 && wb_end_regrd_B == rs2B)
			selrs2_A = 3'b110;
		else
			selrs2_A = 3'b000;
	end
	
endmodule
*/
 
module VLIW_Architecture(input clk, input reset, output [31:0] Result1,output [31:0] Result2);
	
	wire regWrite;
	wire PCwrite, branch, PCSel;
	wire [31:0] PCin, PCout, PCadderOut, PCbranchOut, branchMuxOut, PCmuxOut;
	wire [47:0] IR;
	wire [2:0] InstrSel;
	
	wire [31:0] IRA, IRA_outP_IFID, IRA_outP_IDEX, PC_outP_IFID, PC_outP_IDEX;
	wire [15:0] IRB, IRB_outP_IFID, IRB_outP_IDEX;
	wire regWriteA, regWriteB;
	
	wire [31:0] regFileWriteDataA, regFileWriteDataB;
	wire [31:0] Regrs1AP_IFID, Regrs1BP_IFID, Regrs2AP_IFID, Regrs2BP_IFID;
	
	wire regWriteAout_IDEX, regWriteBout_IDEX;
	wire [31:0] Regrs1AP_IDEX, Regrs1BP_IDEX, Regrs2AP_IDEX, Regrs2BP_IDEX;
	wire [4:0] DestA_IDEX, DestB_IDEX;
	
	wire aluSrc1A;
	wire [1:0] aluSrc2A;
	
	wire aluSrc1A_IDEX;
	wire [1:0] aluSrc2A_IDEX;
	
	wire memReadA,  memWriteA,	 memReadB,  memWriteB;
	wire  memReadA_IDEX,  memWriteA_IDEX,	 memReadB_IDEX,  memWriteB_IDEX;

	 wire [31:0] zeroExtend5to32A, signExtend12to32A ,zeroExtend22to32B, signExtend20to32A;
	 wire [1:0] MemToReg, aluOpA;
	 
	 
	 wire [31:0] zeroExtend5to32A_IDEX,  signExtend12to32A_IDEX,  zeroExtend22to32B_IDEX, signExtend20to32A_IDEX,signExtend8to32B;
	 wire [1:0] MemToReg_IDEX, aluOpA_IDEX;
	 
	 wire branchtakeFlagB_in;
	 wire signFlagA, branchtakeFlagB_IDEX;
	 wire signFlagA_EXMEM, branchtakeFlagB_EXMEM;

	 wire [4:0] rs1A_IDEX, rs1B_IDEX, rs2A_IDEX, rs2B_IDEX;
	 
	 wire [9:0] opcodeA_IDEX;
	 wire [1:0] opcodeB_IDEX;
	 
	 wire [31:0] branchadderoutB;
	 wire [31:0] branchadderoutB_IDEX;
	 wire [31:0] branchadderoutB_EXMEM;
	 wire [31:0] adderoutB;
	 
	 wire memReadAout_EXMEM, memWriteAout_EXMEM, memReadBout_EXMEM, memWriteBout_EXMEM,	regWriteAout_EXMEM ,regWriteBout_EXMEM;
	 wire [1:0]	MemToReg_EXMEM;
	 wire [31:0] aluoutA, aluoutB, aluoutA_EXMEM, aluoutB_EXMEM;
	 
	 wire [4:0] DestA_EXMEM;
	 wire [31:0] Regrs2BP_EXMEM;
	 
	 wire [4:0] DestA_MEMWB;
	 wire [31:0] writeDataA, writeDataA_MEMWB, writeDataB, writeDataB_MEMWB;	//these are the inputs to the last pipeline register
	 
	 wire [31:0] ALUAin1, ALUAin2;
	 wire regWriteAout_MEMWB,regWriteBout_MEMWB;
	 wire branch_signal ;
	 wire branch_signal_ID_EX;
	 wire takebranchflagB_EXMEM,branch_signal_EXMEM;
	 wire [31:0] reduced_pc;
	 reg ID_Flush;
	register32bit PCreg(clk,  reset,  1'b1, 1'b1, PCmuxOut , PCout);
	
	PC_Adder PCadder1(PCout, 32'd1,PCadderOut);
	PC_Adder PCadder2(PCadderOut, signExtend8to32B, PCbranchOut);
	sub subber (PCadderOut, 32'd1,reduced_pc);
	
	mux2to1_32bits MuxPC1(PCadderOut, PCbranchOut,branch,branchMuxOut);	//for branch instruction
	mux2to1_32bits MuxPC2(branchMuxOut, Regrs1BP_IFID ,PCSel,PCmuxOut);	//for jump and link register pipeline2
	
	IM IMblock(clk, reset, PCout[2:0], IR);
   register32bit IMregA(clk, reset,1'b1, 1'b1, IR[47:16], IRA);	
   register16bit IMregB(clk, reset,1'b1, 1'b1, IR[15:0], IRB);
	
	IF_ID IF_IDpipeReg(clk, reset, 1'b1, 1'b1, PCadderOut, IRA, IRB, IRA_outP_IFID, IRB_outP_IFID, PC_outP_IFID);

	//check for C.JALR after sign ext
	registerFile rFile(clk, reset, regWriteAout_MEMWB, regWriteBout_MEMWB, IRA[19:15], IRA[24:20], {2'b00,IRB[9:7]}, {2'b00,IRB[4:2]}, DestA_MEMWB, 5'd1,
							 writeDataA_MEMWB, writeDataB_MEMWB,
							 Regrs1AP_IFID, Regrs1BP_IFID, Regrs2AP_IFID, Regrs2BP_IFID);
	
	
	
	
	
	zeroExt5to32 modZeroExt5to32A(IRA_outP_IFID[24:20],zeroExtend5to32A);
	signExt12to32 modSignExt12to32A(IRA_outP_IFID[31:20],signExtend12to32A);
	zeroExt20to32 modZeroExt20to32A(IRA_outP_IFID[31:12],signExtend20to32A);
	
	signExt8to32 mod8to33B({IRB_outP_IFID[12],IRB_outP_IFID[6:5],IRB_outP_IFID[2],IRB_outP_IFID[11:10], IRB_outP_IFID[4:3]}, signExtend8to32B);
	zeroExt22to32 mod22to32B({15'b0,IRB_outP_IFID[5],IRB_outP_IFID[12:10],IRB_outP_IFID[6],2'b0}, zeroExtend22to32B);
	
	comparator comp_B(Regrs2AP_IFID, 32'd0, branchtakeFlagB_in);
	
	
	ctrlCkt ckt1({IRA_outP_IFID[14:12],IRA_outP_IFID[6:0]}, IRB_outP_IFID[1:0], aluSrc1A, aluSrc2A,
						 regWrite, memReadA, memWriteA, memReadB, memWriteB,
						 regWriteA, regWriteB, aluOpA, MemToReg,
					    PCSel, branch_signal,ID_Flush,dmsel);

	and a0(branch,branchtakeFlagB_in, branch_signal);
	
	always@(branch)
	begin
	assign ID_Flush=branch;
	end
	
	ID_EX ID_EXpipReg(
	 //inputs
	 
	 clk, reset, 1'b1, ID_Flush, 1'b1,
	 PC_outP_IFID,
	 IRA_outP_IFID, IRB_outP_IFID,
	 regWriteA, regWriteB,
	 Regrs1AP_IFID, Regrs2AP_IFID, Regrs1BP_IFID, Regrs2BP_IFID,
	 IRA_outP_IFID[11:7], 5'd1,		//rdA,rdB
	 IRA_outP_IFID[19:15], IRA_outP_IFID [24:20] ,	//rs1A, rs2A
	 {2'b0,IRB_outP_IFID[9:7]}, {2'b0,IRB_outP_IFID[4:2]},	//rs1B, rs2B
	 aluSrc1A,   aluSrc2A, 
	 memReadA,  memWriteA,	 memReadB,  memWriteB,
	 aluOpA,
	 zeroExtend5to32A, signExtend12to32A, zeroExtend22to32B, signExtend20to32A,
	 {IRA_outP_IFID[14:12],IRA_outP_IFID[6:0]}, IRB_outP_IFID[1:0],
	 MemToReg, branchadderoutB,
	
	 //outputs
	 PC_outP_IDEX,
	 IRA_outP_IDEX, IRB_outP_IDEX,
	 regWriteAout_IDEX, regWriteBout_IDEX,
	 rs2A_IDEX,rs2B_IDEX,	
	 Regrs1AP_IDEX, Regrs2AP_IDEX, Regrs1BP_IDEX, Regrs2BP_IDEX,
	 DestA_IDEX, DestB_IDEX,
	 rs1A_IDEX, rs1B_IDEX,
	 aluSrc1A_IDEX, aluSrc2A_IDEX, 
	 memReadA_IDEX, memWriteA_IDEX, memReadB_IDEX, memWriteB_IDEX,
	 aluOpA_IDEX,
	 zeroExtend5to32A_IDEX,  signExtend12to32A_IDEX,  zeroExtend22to32B_IDEX, signExtend20to32A_IDEX,
	 opcodeA_IDEX, opcodeB_IDEX, MemToReg_IDEX, branchadderoutB_IDEX);
	 
	 wire [31:0] muxAoutin1,muxAoutin2;
	 //pipeline A ALU
	 mux2to1_32bits aluAin1(PC_outP_IDEX, Regrs1AP_IDEX, aluSrc1A_IDEX, muxAoutin1);
	 mux4to1_32bits aluAin2(Regrs2AP_IDEX, zeroExtend5to32A_IDEX, signExtend12to32A_IDEX, signExtend20to32A_IDEX,
	 aluSrc2A_IDEX, muxAoutin2);
	 ALU aluA(muxAoutin1, muxAoutin2, aluOpA_IDEX, aluoutA, signFlagA);
	 
	 //pipeline B adder
	PC_Adder pcadder1(Regrs1BP_IDEX, zeroExtend22to32B_IDEX, adderoutB);
	
	EX_MEM EX_MEMpipReg(
	clk, reset, 1'b1, 1'b1,
	signFlagA  ,
	aluoutA, adderoutB, branchadderoutB_IDEX,	//adderoutB output of adder of pipleline B
	Regrs2BP_IDEX, DestA_IDEX, 
	memReadA_IDEX, memWriteA_IDEX, memReadB_IDEX, memWriteB_IDEX, 
	regWriteAout_IDEX, regWriteBout_IDEX, MemToReg_IDEX,
 
	signFlagA_EXMEM, aluoutA_EXMEM, aluoutB_EXMEM, branchadderoutB_EXMEM, Regrs2BP_EXMEM, DestA_EXMEM,
	memReadAout_EXMEM, memWriteAout_EXMEM, memReadBout_EXMEM, memWriteBout_EXMEM,
	regWriteAout_EXMEM, regWriteBout_EXMEM, MemToReg_EXMEM);
	
	wire dmsel;
	wire [31:0] address;
	wire [31:0] data_out;
	mux2to1_32bits muuxx(aluoutA_EXMEM, aluoutB_EXMEM, dmsel, address);
	
	data_Mem dmm( clk,reset, memWriteAout_EXMEM | memWriteBout_EXMEM, memReadBout_EXMEM | memReadAout_EXMEM , address, Regrs2BP_EXMEM, data_out);
	
	wire [31:0] zero_data_out;
	zeroExt8to32 zeroo( data_out[7:0],zero_data_out );
	
	wire [31:0] muxInsltu;
	mux2to1_32bits sltuu(32'd0, 32'd1, signFlagA_EXMEM, muxInsltu);
	mux4to1_32bits writeDataMux(aluoutA_EXMEM, muxInsltu,zero_data_out, 32'd0, MemToReg_EXMEM, writeDataA);
	
	
	
	assign writeDataB = Regrs2BP_EXMEM;
	
	MEM_WB MEM_WBpipreg(clk, reset, 1'b1,1'b1,
					DestA_EXMEM, writeDataA, writeDataB,regWriteAout_EXMEM,regWriteBout_EXMEM,
					DestA_MEMWB, writeDataA_MEMWB, writeDataB_MEMWB,regWriteAout_MEMWB,regWriteBout_MEMWB);
					
	assign Result1 = writeDataA_MEMWB;
	assign Result2 = writeDataB_MEMWB;

	/* wire [31:0] muxAoutin1,muxAoutin2,writeDataB_WBEND,writeDataA_WBEND;
	 wire [31:0] real_regrs1A, real_regrs2A, real_regrs1B;
	 mux8to1_32bit mux_rs1(Regrs1AP_IDEX, aluoutA_EXMEM, aluoutB_EXMEM,
							writeDataA_MEMWB,writeDataB_MEMWB,
							writeDataA_WBEND,writeDataB_WBEND, 32'b0, selrs1_A,real_regrs1A);

	mux8to1_32bit mux_rs2(Regrs2AP_IDEX, aluoutA_EXMEM, aluoutB_EXMEM,
							writeDataA_MEMWB,writeDataB_MEMWB,
							writeDataA_WBEND,writeDataB_WBEND, 32'b0, selrs2_A,real_regrs2A);

	mux8to1_32bit mux_rs3(Regrs1BP_IDEX, aluoutA_EXMEM, aluoutB_EXMEM,
							writeDataA_MEMWB,writeDataB_MEMWB,
							writeDataA_WBEND,writeDataB_WBEND, 32'b0, selrs1_B,real_regrs1B);
	 //pipeline A ALU
	 mux2to1_32bits aluAin1(PC_outP_IDEX, real_regrs1A, aluSrc1A_IDEX, muxAoutin1);
	 mux4to1_32bits aluAin2(real_regrs2A, zeroExtend5to32A_IDEX, signExtend12to32A_IDEX, signExtend20to32A_IDEX,
	 aluSrc2A_IDEX, muxAoutin2);
	 ALU aluA(muxAoutin1, muxAoutin2, aluOpA_IDEX, aluoutA, signFlagA);
	 
	 //pipeline B adder
	PC_Adder pcadder1(real_regrs1B, zeroExtend22to32B_IDEX, adderoutB); */

/*	wire [2:0] selrs1_A, selrs2_A, selrs1_B, selrs2_B;
	forward ffo(clk,reset, regWriteAout_EXMEM, DestA_EXMEM,
					   regWriteAout_MEMWB, DestA_MEMWB,
	 				   regWriteAout_END, rdA_END, // TO BO DONE
	
					   regWriteBout_EXMEM, 5'd1,
					   regWriteBout_MEMWB, 5'd1,
					  regWriteBout_END, rdB_END, //TO BE DONE
	
					rs1A_IDEX, rs2A_IDEX, rs1B_IDEX, rs2B_IDEX,
					//Outputs(select lines)
					selrs1_A, selrs2_A, selrs1_B, selrs2_B);
	
	wire [31:0] muxInsltu;
	mux2to1_32bits sltuu(32'd0, 32'd1, signFlagA_EXMEM, muxInsltu);
	mux4to1_32bits writeDataMux(aluoutA_EXMEM, muxInsltu,zero_data_out, 32'd0, MemToReg_EXMEM, writeDataA);*/

/*	WB_END lastpipe(clk,reset,1'b1,1'b1,
	DestA_MEMWB,writeDataA_MEMWB,
	5'd1,writeDataB_MEMWB,
	regWriteAout_MEMWB,regWriteBout_MEMWB,
	rdA_END,writeData_A_END,rdB_END,writeData_B_END,regWriteAout_END,regWriteBout_END);		*/		
		
endmodule
 
module testbench;
	reg clk;
	reg reset;
	wire [31:0] Result1,Result2;
	VLIW_Architecture vliw(clk, reset, Result1, Result2);
 
	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#2  reset=0;	
		#300 $finish;
	end
endmodule