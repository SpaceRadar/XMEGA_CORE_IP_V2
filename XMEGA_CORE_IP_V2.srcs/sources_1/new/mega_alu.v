`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/02/2017 01:18:44 PM
// Design Name: 
// Module Name: mega_alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`include "mega_core.vh"
`include "mega_alu.vh"

module mega_alu(
	input [15:0]inst,
	input [4:0]in_addr_1,
	input [4:0]in_addr_2,
	input [15:0]in_1,
	input [15:0]in_2,
	output reg [15:0]out,
	//output c_out,
	input ALU_FLAG_C_IN,	//Zero Flag
	input ALU_FLAG_Z_IN,	//Zero Flag
	input ALU_FLAG_N_IN, //Negative Flag
	input ALU_FLAG_V_IN, //Two's complement overflow indicator 
	input ALU_FLAG_S_IN,	//N?V for signed tests
	input ALU_FLAG_H_IN,	//Half Carry Flag
	input ALU_FLAG_T_IN,	//Transfer bit used by BLD and BST instructions
	input ALU_FLAG_I_IN,	//Global Interrupt Enable/Disable Flag
	output reg ALU_FLAG_C_OUT,	//Carry Flag
	output reg ALU_FLAG_Z_OUT,	//Zero Flag
	output reg ALU_FLAG_N_OUT, //Negative Flag
	output reg ALU_FLAG_V_OUT, //Two's complement overflow indicator 
	output reg ALU_FLAG_S_OUT,	//N?V for signed tests
	output reg ALU_FLAG_H_OUT,	//Half Carry Flag
	output reg ALU_FLAG_T_OUT,	//Transfer bit used by BLD and BST instructions
	output reg ALU_FLAG_I_OUT	//Global Interrupt Enable/Disable Flag
    );
    
reg [`ALU_SELECT_BUS_SIZE - 1:0]inst_dec_out;

initial
begin
	ALU_FLAG_C_OUT = 0;
	ALU_FLAG_Z_OUT = 0;
	ALU_FLAG_N_OUT = 0;
	ALU_FLAG_V_OUT = 0;
	ALU_FLAG_S_OUT = 0;
	ALU_FLAG_H_OUT = 0;
	ALU_FLAG_T_OUT = 0;
	ALU_FLAG_I_OUT = 0;
end

wire in_addr_1_and_2_equal = in_addr_1 == in_addr_2;

always @ (*)
begin
	casex(inst)
		`INSTRUCTION_ADD: 
		begin
			if(in_addr_1_and_2_equal)
				inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000;
			else
				inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1;
		end
		`INSTRUCTION_ADC: 
		begin
			if(in_addr_1_and_2_equal)
				inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000;
			else
				inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10;
		end
		`INSTRUCTION_SUB: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100;
		`INSTRUCTION_SBC: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000;
		`INSTRUCTION_LSR: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000;
		//`ALU_LSR: 			inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100000;
		`INSTRUCTION_ROR: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000000;
		//`ALU_ROL: 			inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000;
		`INSTRUCTION_AND: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100000000;
		`INSTRUCTION_OR: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000000000;
		`INSTRUCTION_EOR: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000000;
		`INSTRUCTION_MOV: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100000000000;
		`INSTRUCTION_MOVW: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000000000000;
		`INSTRUCTION_COM: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000000000;
		`INSTRUCTION_NEG: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100000000000000;
		`INSTRUCTION_ADIW: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000000000000000;
		`INSTRUCTION_SBIW: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000000000000;
		`INSTRUCTION_MUL: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100000000000000000;
		`INSTRUCTION_ASR: 		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000000000000000000;
		`INSTRUCTION_CP,
		`INSTRUCTION_CPI:		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000000000000000;
		`INSTRUCTION_CPC:		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b100000000000000000000;
		`INSTRUCTION_SWAP:		inst_dec_out <= `ALU_SELECT_BUS_SIZE'b1000000000000000000000;
		`INSTRUCTION_SEx_CLx:	inst_dec_out <= `ALU_SELECT_BUS_SIZE'b10000000000000000000000;
		default: 				inst_dec_out <= `ALU_SELECT_BUS_SIZE'h00000000;
	endcase
end

reg [15:0]in_2_int_1;
reg [15:0]in_2_int;
reg cin_int;

always @ (*)
begin
	in_2_int_1 <= ~in_2;
	in_2_int <= in_1;
	cin_int <= in_1;
	casex(inst)
	`INSTRUCTION_ADD,
	`INSTRUCTION_ADC:
	begin
		if(!in_addr_1_and_2_equal)
			in_2_int_1 <= in_2;
	end
	`INSTRUCTION_ADIW: in_2_int_1 <= in_2;
	//default: in_2_int_1 <= ~in_2;
	endcase

	casex(inst)
	`INSTRUCTION_ADD,
	`INSTRUCTION_ADC:
	begin
		if(!in_addr_1_and_2_equal)
			in_2_int <= in_2_int_1;
	end
	`INSTRUCTION_ADIW,
	`INSTRUCTION_SBIW,
	`INSTRUCTION_SUB,
	`INSTRUCTION_SBC: in_2_int <= in_2_int_1;
	//default: in_2_int <= in_1;
	endcase


	casex(inst)
	`INSTRUCTION_ADD,
	`INSTRUCTION_SUB,
	`INSTRUCTION_LSR,
	`INSTRUCTION_NEG: cin_int <= 1'b0;
	//default: cin_int <= in_1;
	endcase
end
//assign in_2_int_1 = (inst_dec_out[`ALU_ADD] || inst_dec_out[`ALU_ADC] || inst_dec_out[`ALU_ADIW]) ? in_2 : ~in_2;
//assign in_2_int = (inst_dec_out[`ALU_ADD] || inst_dec_out[`ALU_ADC] || inst_dec_out[`ALU_SUB] || inst_dec_out[`ALU_SBC] || inst_dec_out[`ALU_ADIW] || inst_dec_out[`ALU_SBIW]) ? in_2_int_1 : in_1;
wire [15:0]in_1_int;
assign in_1_int = (inst_dec_out[`ALU_NEG]) ? 16'hFFFF : in_1;
//assign cin_int = (inst_dec_out[`ALU_ADD] || inst_dec_out[`ALU_SUB] || inst_dec_out[`ALU_LSL] || inst_dec_out[`ALU_LSR] || inst_dec_out[`ALU_NEG]) ? 0 : ALU_FLAG_C_IN;
wire [16:0] add_result_int_w_c;
`ifndef USE_CUSTOM_ADDER
wire [17:0] add_result_int_w_c_tmp;
assign add_result_int_w_c_tmp = {in_1_int, 1'b1} + {in_2_int, cin_int};
assign add_result_int_w_c = add_result_int_w_c_tmp[17:1];
`endif
`ifdef USE_MULTIPLYER
`ifdef USE_CUSTOM_MULTIPLYER
wire [15:0]mul_result_int;
multiplyer MUL(in_1, in_2, mul_result_int);
`else
wire [15:0]mul_result_int = in_1 * in_2;
`endif
`endif
wire carry_8bit = in_1 < in_2;
wire carry_8bit_plus_carry = in_1 < in_2 + ALU_FLAG_C_IN;

`ifdef USE_CUSTOM_ADDER
add_w_carry # (.WIDTH(16)) alu_add(
		.c_in(cin_int),
		.in_1(in_1_int),
		.in_2(in_2_int),
		.out(add_result_int_w_c),
		.c_out()	
	);
`endif

always @ (*)
begin
	{ALU_FLAG_C_OUT, out} <= 17'h00000;
	casex(inst)
		`INSTRUCTION_ADD: 
		begin
			if(in_addr_1_and_2_equal)
				{ALU_FLAG_C_OUT, out} <= {in_1_int[7], 7'h00, in_1_int[6:0], cin_int};//LSR
			else
				{ALU_FLAG_C_OUT, out} <= {add_result_int_w_c[8], 8'h00, add_result_int_w_c[7:0]};
		end
		`INSTRUCTION_ADC: 
		begin
			if(in_addr_1_and_2_equal)
				{ALU_FLAG_C_OUT, out} <= {in_1_int[7], 7'h00, in_1_int[6:0], cin_int};//ROL
			else
				{ALU_FLAG_C_OUT, out} <= {add_result_int_w_c[8], 8'h00, add_result_int_w_c[7:0]};
		end
		`INSTRUCTION_SUB,
		`INSTRUCTION_SBC: 		{ALU_FLAG_C_OUT, out} <= {add_result_int_w_c[8], 8'h00, add_result_int_w_c[7:0]};
		`INSTRUCTION_LSR,
		`INSTRUCTION_ROR: 		{ALU_FLAG_C_OUT, out} <= {in_1_int[0], 7'h00, cin_int, in_1_int[7:1]};
		`INSTRUCTION_AND: 		{ALU_FLAG_C_OUT, out} <= {9'h00, (in_1_int[7:0] & in_2[7:0])};
		`INSTRUCTION_OR: 		{ALU_FLAG_C_OUT, out} <= {9'h00, (in_1_int[7:0] | in_2[7:0])};
		`INSTRUCTION_EOR: 		{ALU_FLAG_C_OUT, out} <= {9'h00, (in_1_int[7:0] ^ in_2[7:0])};
		`INSTRUCTION_MOV: 		{ALU_FLAG_C_OUT, out} <= {9'h00, in_1_int[7:0]};
		`INSTRUCTION_MOVW: 		{ALU_FLAG_C_OUT, out} <= {1'h00, in_1_int};
		`INSTRUCTION_COM: 		{ALU_FLAG_C_OUT, out} <= {1'b1, 8'h00, (~in_1_int[7:0])};
		`INSTRUCTION_NEG: 		{ALU_FLAG_C_OUT, out} <= {|add_result_int_w_c[7:0], 8'h00, add_result_int_w_c[7:0]};
		`INSTRUCTION_ADIW,
		`INSTRUCTION_SBIW: 		{ALU_FLAG_C_OUT, out} <= add_result_int_w_c;
`ifdef USE_MULTIPLYER
		`INSTRUCTION_MUL: 		{ALU_FLAG_C_OUT, out} <= {mul_result_int[15], mul_result_int};
`endif
		`INSTRUCTION_ASR: 		{ALU_FLAG_C_OUT, out} <= {in_1[0], 7'h00, in_1[7], in_1[7:1]};
		`INSTRUCTION_CP,
		`INSTRUCTION_CPI:		{ALU_FLAG_C_OUT, out} <= {carry_8bit, 16'h0000};
		`INSTRUCTION_CPC:		{ALU_FLAG_C_OUT, out} <= {carry_8bit_plus_carry, 16'h0000};
		`INSTRUCTION_SWAP:		{ALU_FLAG_C_OUT, out} <= {1'b0, in_1[3:0], in_1[7:4]};
		`INSTRUCTION_SEx_CLx:	{ALU_FLAG_C_OUT, out} <= {inst[7], {16{1'b0}}};
	endcase
end

/*assign {ALU_FLAG_C_OUT, out} = (inst_dec_out[`ALU_ADD] || inst_dec_out[`ALU_SUB]) ? {add_result_int_w_c[8], 8'h00, add_result_int_w_c[7:0]} :
			(inst_dec_out[`ALU_ADC]|| inst_dec_out[`ALU_SBC]) ? {add_result_int_w_c[8], 8'h00, add_result_int_w_c[7:0]} :
			(inst_dec_out[`ALU_ADIW] || inst_dec_out[`ALU_SBIW]) ? add_result_int_w_c :
			(inst_dec_out[`ALU_LSL] || inst_dec_out[`ALU_ROL]) ? {in_1_int[7], 7'h00, in_1_int[6:0], cin_int} :
			(inst_dec_out[`ALU_LSR] || inst_dec_out[`ALU_ROR]) ? {in_1_int[0], 7'h00, cin_int, in_1_int[7:1]} :
			(inst_dec_out[`ALU_AND]) ? {9'h00, (in_1_int[7:0] & in_2[7:0])} :
			(inst_dec_out[`ALU_OR]) ? {9'h00, (in_1_int[7:0] | in_2[7:0])} :
			(inst_dec_out[`ALU_EOR]) ? {9'h00, (in_1_int[7:0] ^ in_2[7:0])} :
			(inst_dec_out[`ALU_COM]) ? {1'b1, 8'h00, (~in_1_int[7:0])} :
			(inst_dec_out[`ALU_NEG]) ? {|add_result_int_w_c[7:0], 8'h00, add_result_int_w_c[7:0]} :
			(inst_dec_out[`ALU_MOV]) ? {9'h00, in_1_int[7:0]} : 
			(inst_dec_out[`ALU_MOVW]) ? {1'h00, in_1_int} : 
`ifdef USE_MULTIPLYER
			(inst_dec_out[`ALU_MUL]) ? {mul_result_int[15], mul_result_int} : 
`endif
			(inst_dec_out[`ALU_ASR]) ? {in_1[0], 7'h00, in_1[7], in_1[7:1]} : 
			(inst_dec_out[`ALU_CP_CPI]) ? {carry_8bit, 16'h0000} : 
			(inst_dec_out[`ALU_CPC]) ? {carry_8bit_plus_carry, 16'h0000} : 
			(inst_dec_out[`ALU_SWAP]) ? {1'b0, in_1[3:0], in_1[7:4]} : 
			(inst_dec_out[`SEx_CLx]) ? {inst[7], {16{1'b0}}} :
			17'h00000;
*/	
/*
 * ALU FLAG effect for each instruction.
 */
always @ (*)
begin
	ALU_FLAG_Z_OUT <= ALU_FLAG_Z_IN;
	ALU_FLAG_N_OUT <= ALU_FLAG_N_IN;
	ALU_FLAG_V_OUT <= ALU_FLAG_V_IN;
	ALU_FLAG_S_OUT <= ALU_FLAG_S_IN;
	ALU_FLAG_H_OUT <= ALU_FLAG_H_IN;
	ALU_FLAG_T_OUT <= ALU_FLAG_T_IN;
	ALU_FLAG_I_OUT <= ALU_FLAG_I_IN;
	casex(inst)
	`INSTRUCTION_ADD:
	begin
		if(in_addr_1_and_2_equal)
		begin
			ALU_FLAG_H_OUT <= in_1[3];
			ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
			ALU_FLAG_V_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_C_OUT;
			ALU_FLAG_N_OUT <= out[7];
			ALU_FLAG_Z_OUT <= &(~out[7:0]);
		end
		else
		begin
			ALU_FLAG_H_OUT <= (in_1[3] & in_2[3])|(in_2[3] & ~out[3])|(~out[3] & in_1[3]);
			ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
			ALU_FLAG_V_OUT <= (in_1[7] & in_2[7] & ~out[7])|(~in_1[7] & ~in_2[7] & out[7]);
			ALU_FLAG_N_OUT <= out[7];
			ALU_FLAG_Z_OUT <= &(~out[7:0]);
		end
	end
	`INSTRUCTION_ADC:
	if(in_addr_1_and_2_equal)
	begin
		ALU_FLAG_H_OUT <= in_1[3];
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_C_OUT;
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~out[7:0]);
	end
	else
	begin
		ALU_FLAG_H_OUT <= (in_1[3] & in_2[3])|(in_2[3] & ~out[3])|(~out[3] & in_1[3]);
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= (in_1[7] & in_2[7] & ~out[7])|(~in_1[7] & ~in_2[7] & out[7]);
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~{out[7:0], ALU_FLAG_Z_IN});
	end
	`INSTRUCTION_SUB,
	`INSTRUCTION_CP,
	`INSTRUCTION_CPI:
	begin
		ALU_FLAG_H_OUT <= (in_1[3] & in_2[3])|(in_2[3] & ~out[3])|(~out[3] & in_1[3]);
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= (in_1[7] & in_2[7] & ~out[7])|(~in_1[7] & ~in_2[7] & out[7]);
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~out[7:0]);
	end
	`INSTRUCTION_SBC,
	`INSTRUCTION_CPC:
	begin
		ALU_FLAG_H_OUT <= (in_1[3] & in_2[3])|(in_2[3] & ~out[3])|(~out[3] & in_1[3]);
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= (in_1[7] & in_2[7] & ~out[7])|(~in_1[7] & ~in_2[7] & out[7]);
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~{out[7:0], ALU_FLAG_Z_IN});
	end
	`INSTRUCTION_ADIW,
	`INSTRUCTION_SBIW:
	begin
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= ALU_FLAG_C_OUT;
		ALU_FLAG_N_OUT <= out[15];
		ALU_FLAG_Z_OUT <= &(~out[15:0]);
	end
	`INSTRUCTION_AND,
	`INSTRUCTION_OR,
	`INSTRUCTION_COM,
	`INSTRUCTION_EOR:
	begin
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= 1'b0;
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~out[7:0]);
	end
	`INSTRUCTION_NEG:
	begin
		ALU_FLAG_H_OUT <= out[3] + ~in_1[3];
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= &{out[7], ~out[6:0]};
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~out[7:0]);
	end
	`INSTRUCTION_ASR:
	begin
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= 1'b0;
		ALU_FLAG_N_OUT <= out[7];
		ALU_FLAG_Z_OUT <= &(~out[7:0]);
	end
	`INSTRUCTION_LSR,
	`INSTRUCTION_ROR:
	begin
		ALU_FLAG_H_OUT <= in_1[3];
		ALU_FLAG_S_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_V_OUT;
		ALU_FLAG_V_OUT <= ALU_FLAG_N_OUT & ALU_FLAG_C_OUT;
		ALU_FLAG_N_OUT <= 0;
		ALU_FLAG_Z_OUT <= &(~out[7:0]);
	end
	`INSTRUCTION_SEx_CLx:
	begin
		case(inst[6:4])
		3'd1: ALU_FLAG_Z_OUT <= inst[7];
		3'd2: ALU_FLAG_N_OUT <= inst[7];
		3'd3: ALU_FLAG_V_OUT <= inst[7];
		3'd4: ALU_FLAG_S_OUT <= inst[7];
		3'd5: ALU_FLAG_H_OUT <= inst[7];
		3'd6: ALU_FLAG_T_OUT <= inst[7];
		3'd7: ALU_FLAG_I_OUT <= inst[7];
		endcase
	end
`ifdef USE_MULTIPLYER
	`INSTRUCTION_MUL:
	begin
		ALU_FLAG_Z_OUT <= &(~out[15:0]);
	end
`endif
	endcase
end

endmodule
