`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/30/2017 06:57:13 PM
// Design Name: 
// Module Name: mega_regs
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


module mega_regs (
	input clk,
	input [4:0]rw_addr,
	input [15:0]rw_data,
	input rw_16bit,
	input write,
	input [4:0]rd_addr_d,
	output [15:0]rd_data_d,
	input rd_16bit_d,
	input read_d,
	input [4:0]rd_addr_r,
	output [15:0]rd_data_r,
	input rd_16bit_r,
	input read_r
);

reg [7:0]REGL[0:15];
reg [7:0]REGH[0:15];

reg [4:0]k;

initial
begin
	for (k = 0; k < 16 - 1; k = k + 1)
	begin
		REGL[k] = 0;
		REGH[k] = 0;
	end
end

always @ (posedge clk)
begin
	if(write)
	begin
		if(!rw_16bit & !rw_addr[0])
			REGL[rw_addr[4:1]] <= rw_data[7:0];
		else if(!rw_16bit & rw_addr[0])
			REGH[rw_addr[4:1]] <= rw_data[7:0];
		else
		begin
			REGL[rw_addr[4:1]] <= rw_data[7:0];
			REGH[rw_addr[4:1]] <= rw_data[15:8];
		end
	end
end

assign rd_data_d = (read_d) ? (rd_16bit_d) ? {REGH[rd_addr_d[4:1]], REGL[rd_addr_d[4:1]]} : (rd_addr_d[0]) ? REGH[rd_addr_d[4:1]] : REGL[rd_addr_d[4:1]] : 16'bz;
assign rd_data_r = (read_r) ? (rd_16bit_r) ? {REGH[rd_addr_r[4:1]], REGL[rd_addr_r[4:1]]} : (rd_addr_r[0]) ? REGH[rd_addr_r[4:1]] : REGL[rd_addr_r[4:1]] : 16'bz;

endmodule
