`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Iulian Gheorghiu
// 
// Create Date: 08/15/2017 12:36:59 AM
// Design Name: 
// Module Name: add_w_carry.
// Project Name: Add with carry.
// Target Devices: All
// Tool Versions: 
// Description: This is a custom add with carry IP.
// 
// Dependencies: None
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module add_w_carry # (
	parameter WIDTH = 8)(
		input c_in,
		input [WIDTH-1:0]in_1,
		input [WIDTH-1:0]in_2,
		output [WIDTH-1:0]out,
		output c_out
    );
wire [WIDTH-1:0]carry;
wire [WIDTH-1:0]p;
wire [WIDTH-1:0]r;
wire [WIDTH-1:0]s;
assign c_out = carry[WIDTH-1];
genvar count_generate;
generate
	for (count_generate = 0; count_generate < WIDTH; count_generate = count_generate + 1)
	begin: ADD
		xor (p[count_generate], in_1[count_generate], in_2[count_generate]);
		xor (out[count_generate], p[count_generate], count_generate ? carry[count_generate - 1] : c_in);
	 
		and(r[count_generate], p[count_generate], count_generate ? carry[count_generate - 1] : c_in);
		and(s[count_generate], in_1[count_generate], in_2[count_generate]);
		or(carry[count_generate], r[count_generate], s[count_generate]);

	end
endgenerate
endmodule
