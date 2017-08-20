`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/10/2017 02:06:55 PM
// Design Name: 
// Module Name: top
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
`define BUS_ADDR_PGM_LEN	11 /* < in 16-bit instructions */
`define BUS_ADDR_DATA_LEN	8  /* < in bytes */

module top(
	input ck_rst,
	input CLK,
	output reg RGB0_Green,
	output reg RGB1_Green,
	output reg RGB2_Green,
	output reg RGB3_Green,
	output reg [3:0]LED,
	input [3:0]SW,
	input [3:0]BTN
	);

//wire pgm_re;
wire [`BUS_ADDR_PGM_LEN-1:0] pgm_addr;
wire [15:0] pgm_data;
wire data_re;
wire data_we;
wire [`BUS_ADDR_DATA_LEN-1:0] data_addr;
wire [7:0]data_in;
wire [7:0]data_out;

wire io_re;
wire io_we;
wire [5:0] io_addr;
wire [7:0] io_out;
wire [7:0] io_in;


rom  #(
.bus_addr_pgm_width(`BUS_ADDR_PGM_LEN),
.rom_path("core1ROM.mem")
)rom(
	.pmem_a(pgm_addr),
	.pmem_d(pgm_data)
	
);

ram  #(
.bus_addr_data_width(`BUS_ADDR_DATA_LEN),
.ram_path("NONE")
)ram(
	.dmem_re(data_re),
	.dmem_we(data_we),
	.dmem_a(data_addr),
	.dmem_r(data_in),
	.dmem_w(data_out)
);

reg [7:0]out_led;
wire io_select_0 = io_addr == 0;

always @ (*)
begin
	if(io_select_0 & io_we)
		{RGB0_Green, RGB1_Green, RGB2_Green, RGB3_Green, LED} <= io_out;
end

assign io_in = (io_re & io_select_0) ? {BTN, SW} : 8'bz; // 

reg RST = 0;

initial begin
	RST = 0;
	#1;
	RST = 1;
	#1000;
	$finish;
end

mega_core #(
.bus_addr_pgm_width(`BUS_ADDR_PGM_LEN),
.bus_addr_data_width(`BUS_ADDR_DATA_LEN)
)core(
	.rst(~ck_rst),
	.clk(CLK),
	
	//.pgm_re(1'b1),
	.pgm_addr(pgm_addr),
	.pgm_data(pgm_data),
	
	.data_re(data_re),
	.data_we(data_we),
	.data_addr(data_addr),
	.data_in(data_in),
	.data_out(data_out),
	
	.io_re(io_re),
	.io_we(io_we),
	.io_addr(io_addr),
	.io_out(io_out),
	.io_in(io_in)
);

//fft fft_inst(RST, CLK);

endmodule
