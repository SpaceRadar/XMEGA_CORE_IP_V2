`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2017 01:56:25 PM
// Design Name: 
// Module Name: xmega_core_v2_sim
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

module xmega_core_v2_sim(
	//output reg [7:0]port_out,
	//input [7:0]port_in
    );

reg [7:0]port_out;
wire [7:0]port_in = port_out;


reg ck_rst = 0;
reg CLK = 0;
always	#(1)	CLK	<=	~CLK;	//	clocking	device

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
.rom_path("core1ROMsym.mem")
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
wire io_select_0 = (io_addr == 0 & (io_we | io_re)) ? 1'b1:1'b0;

always @ (*)
begin
	if(io_select_0 & io_we)
		port_out <= io_out;
end

assign io_in = (io_re & io_select_0) ? port_in : 8'bz; // 

initial begin
	ck_rst = 0;
	#1;
	ck_rst = 1;
	#1;
	ck_rst = 0;
	
	port_out = 8'haa;
	#14000;
	$finish;
end

mega_core #(
.bus_addr_pgm_width(`BUS_ADDR_PGM_LEN),
.bus_addr_data_width(`BUS_ADDR_DATA_LEN)
)core(
	.rst(ck_rst),
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

endmodule
	 
	 