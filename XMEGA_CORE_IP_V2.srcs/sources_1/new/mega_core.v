`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/10/2017 02:33:11 PM
// Design Name: 
// Module Name: mega_core
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

module mega_core # (
	parameter bus_addr_pgm_width = 11,
	parameter bus_addr_data_width = 12
)(
	input rst,
	input clk,
	
	output [bus_addr_pgm_width-1:0]pgm_addr,
	input [15:0]pgm_data,
	//output reg pgm_re,
	
	output reg [bus_addr_data_width-1:0]data_addr,
	input [7:0]data_in,
	output reg [7:0]data_out,
	output reg data_we,
	output data_re,
	
	output reg [5:0]io_addr,
	input [7:0]io_in,
	output reg [7:0]io_out,
	output io_we,
	output reg io_re
    );

assign data_re = 1'b1;

reg [7:0]ALU_FLAGS = 0;	//Carry Flag
wire ALU_FLAG_C_OUT;	//Carry Flag
wire ALU_FLAG_Z_OUT;	//Zero Flag
wire ALU_FLAG_N_OUT;	//Negative Flag
wire ALU_FLAG_V_OUT;	//Two's complement overflow indicator 
wire ALU_FLAG_S_OUT;	//N?V for signed tests
wire ALU_FLAG_H_OUT;	//Half Carry Flag
wire ALU_FLAG_T_OUT;	//Transfer bit used by BLD and BST instructions
wire ALU_FLAG_I_OUT;	//Global Interrupt Enable/Disable Flag
reg [15:0]PC = 0;
assign pgm_addr = PC;
reg [15:0]SP = 0;
reg [2:0]step_cnt = 0;
reg [15:0]tmp_pgm_data = 0;

// REG aditional
reg write_to_reg = 0;
reg write_to_io = 0;
reg write_to_io_tmp = 0;
// REG wires
reg [4:0]rw_addr = 0;
reg [15:0]rw_data = 0;
reg rw_16bit = 0;
//wire write;
reg [4:0]rd_addr_d = 0;
wire [15:0]rd_data_d;
reg rd_16bit_d = 0;
wire read_d;
reg [4:0]rd_addr_r = 0;
wire [15:0]rd_data_r;
reg rd_16bit_r = 0;
wire read_r;

/*reg_write_generator io_write(
	.rst(rst),
	.clk(clk),
	.wtite(write_to_io),
	.pulse(io_we)
);*/


// ALU wires
reg [15:0]alu_in_1 = 0;
reg [15:0]alu_in_2 = 0;
wire [15:0]alu_out;
wire alu_c_out;
reg [15:0]indirect_addr_offset = 0;
wire [15:0]indirect_addr_offset_res = rd_addr_d + indirect_addr_offset;

/*
 * Translate core instructions to REG adresses, and connect busses to ALU.
 * Arithmetic instructions.
 */
always @ (*)
begin
	rw_addr <= {5{1'b00}};
	rw_16bit <= 1'b0;
	rd_addr_d <= {5{1'b00}};
	rd_16bit_d <= 1'b0;
	rd_addr_r <= {5{1'b00}};
	rd_16bit_r <= 1'b0;
	// Connect busses
	rw_data <= {16{1'b00}};
	alu_in_1 <= {16{1'b00}};
	alu_in_2 <= {16{1'b00}};
	write_to_reg <= 1'b0;
	io_re <= 1'b0;
	indirect_addr_offset <= {16{1'b00}};
	case(step_cnt)
	`STEP1:
	begin
		casex(pgm_data)
		`INSTRUCTION_MOVW:
		begin
			rw_addr <= {pgm_data[7:4], 1'b0};
			rw_16bit <= 1'b1;
			rd_addr_d <= {pgm_data[7:4], 1'b0};
			rd_16bit_d <= 1'b1;
			rd_addr_r <= {pgm_data[3:0], 1'b0};
			rd_16bit_r <= 1'b1;
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			alu_in_2 <= rd_data_r;
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		`INSTRUCTION_CPC_CP:
		begin
			rw_addr <= pgm_data[8:4];
			rd_addr_d <= pgm_data[8:4];
			rd_addr_r <= {pgm_data[9], pgm_data[3:0]};
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			alu_in_2 <= rd_data_r;
		end
		`INSTRUCTION_SBC_SUB_ADD_ADC,
		`INST_AND_EOR_OR_MOV:
		begin
			rw_addr <= pgm_data[8:4];
			rd_addr_d <= pgm_data[8:4];
			rd_addr_r <= {pgm_data[9], pgm_data[3:0]};
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			alu_in_2 <= rd_data_r;
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		`INSTRUCTION_SUBI_SBCI,
		`INSTRUCTION_ORI_ANDI:
		begin
			rw_addr <= {1'b1, pgm_data[7:4]};
			rd_addr_d <= {1'b1, pgm_data[7:4]};
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			alu_in_2 <= {pgm_data[11:8], pgm_data[3:0]};
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		`INST_COM_NEG_SWAP_INC,
		`INSTRUCTION_ASR,
		`INSTRUCTION_LSR,
		`INSTRUCTION_ROR:
		begin
			rw_addr <= pgm_data[8:4];
			rd_addr_d <= pgm_data[8:4];
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		`INSTRUCTION_LDD_STD:
		begin
			rd_addr_d <= {{3{1'b1}}, pgm_data[3], 1'b0};
			rd_16bit_d <= 1'b1;
			indirect_addr_offset <= {{11{1'b0}}, pgm_data[11:10], pgm_data[2:0]};
		end
		`INSTRUCTION_LD_ST_X:
		begin
			rd_addr_d <= 5'd26;
			rd_16bit_d <= 1'b1;
			indirect_addr_offset <= 16'h0000;
		end
		`INSTRUCTION_LDS_STS:
		begin
			rd_addr_d <= {{3{1'b1}}, pgm_data[3], 1'b0};
			rd_16bit_d <= 1'b1;
		end
		`INSTRUCTION_LD_ST_YZP:
		begin
			rd_addr_d <= {{3{1'b1}}, pgm_data[3], 1'b0};
			rd_16bit_d <= 1'b1;
			rw_addr <= {{3{1'b1}}, pgm_data[3], 1'b0};
			rw_16bit <= 1'b1;
			rw_data <= indirect_addr_offset_res;
			write_to_reg <= 1'b1;
			indirect_addr_offset <= 16'h0001;
		end
		`INSTRUCTION_LD_ST_YZN:
		begin
			rd_addr_d <= {{3{1'b1}}, pgm_data[3], 1'b0};
			rd_16bit_d <= 1'b1;
			rw_addr <= {{3{1'b1}}, pgm_data[3], 1'b0};
			rw_16bit <= 1'b1;
			rw_data <= indirect_addr_offset_res;
			write_to_reg <= 1'b1;
			indirect_addr_offset <= {16{1'b1}};
		end
		`INSTRUCTION_LD_ST_XP:
		begin
			rd_addr_d <= 5'd26;
			rd_16bit_d <= 1'b1;
			rw_addr <= 5'd26;
			rw_16bit <= 1'b1;
			rw_data <= indirect_addr_offset_res;
			write_to_reg <= 1'b1;
			indirect_addr_offset <= 16'h0001;
		end
		`INSTRUCTION_LD_ST_XN:
		begin
			rd_addr_d <= 5'd26;
			rd_16bit_d <= 1'b1;
			rw_addr <= 5'd26;
			rw_16bit <= 1'b1;
			rw_data <= indirect_addr_offset_res;
			write_to_reg <= 1'b1;
			indirect_addr_offset <= {16{1'b1}};
		end
		`INSTRUCTION_XCH,
		`INSTRUCTION_LAS,
		`INSTRUCTION_LAC,
		`INSTRUCTION_LAT:
		begin
			rd_addr_d <= 5'b11110;
		end
		`INSTRUCTION_IN_OUT:
		begin
			rw_addr <= pgm_data[8:4];
			rd_addr_d <= pgm_data[8:4];
			if(!pgm_data[11])
			begin
				io_re <= 1'b1;
				rw_data <= io_in;
				write_to_reg <= 1'b1;
			end
		end
`ifdef USE_MULTIPLYER
		`INSTRUCTION_MUL:
		begin
			rw_16bit <= 1'b1;
			rd_addr_d <= pgm_data[8:4];
			rd_addr_r <= {pgm_data[9], pgm_data[3:0]};
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			alu_in_2 <= rd_data_r;
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
`endif
		`INSTRUCTION_LDI:
		begin
			rw_addr <= {1'b1, pgm_data[7:4]};
			// Connect busses
			rw_data <= {pgm_data[11:8], pgm_data[3:0]};
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		`INSTRUCTION_ADIW_SBIW:
		begin
			rw_addr <= {2'b11, pgm_data[5:4], 1'b0};
			rw_16bit <= 1'b1;
			rd_addr_d <= {2'b11, pgm_data[5:4], 1'b0};
			rd_16bit_d <= 1'b1;
			//rd_addr_r <= {5{1'b00}};
			rd_16bit_r <= 1'b1;
			// Connect busses
			rw_data <= alu_out;
			alu_in_1 <= rd_data_d;
			alu_in_2 <= {10'h000, pgm_data[7:6], pgm_data[3:0]};
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		`INSTRUCTION_IJMP_ICALL:
		begin
			rd_addr_d <= 5'b11110;
			rd_16bit_d <= 1'b1;
		end
		endcase
	end
	`STEP2:
	begin
		casex(tmp_pgm_data)
		`INSTRUCTION_LDD_STD,
		`INSTRUCTION_LDS_STS,
		`INSTRUCTION_LD_ST_YZP,
		`INSTRUCTION_LD_ST_YZN,
		`INSTRUCTION_LD_ST_XP,
		`INSTRUCTION_LD_ST_XN,
		`INSTRUCTION_LD_ST_X,
		`INSTRUCTION_POP_PUSH:
		begin
			if(!tmp_pgm_data[9])
			begin
				rd_addr_d <= tmp_pgm_data[8:4];
				rw_addr <= tmp_pgm_data[8:4];
				// Connect busses
				rw_data <= data_in;
				// Signalize write_to_reg;
				write_to_reg <= 1'b1;
			end
		end
		`INSTRUCTION_XCH,
		`INSTRUCTION_LAS,
		`INSTRUCTION_LAC,
		`INSTRUCTION_LAT:
		begin
			rw_addr <= 5'b11110;
			// Connect busses
			rw_data <= data_in;
			// Signalize write_to_reg;
			write_to_reg <= 1'b1;
		end
		endcase
	end
	endcase
end

reg [15:0]pc_offset = 0;
wire [15:0]pc_inc_value;
//wire [15:0]pc_inc_value = pc_offset + 1;
reg [15:0]sp_inc_value = 0;
reg [7:0]TEMP8 = 0;
reg [15:0]TEMP16 = 0;
/*
 * PC, SP and jumps.
 * Jump and branch instructions.
 */
always @ (posedge clk)
begin
	if(rst)
	begin
		ALU_FLAGS[0] <= 1'b0;	//Carry Flag
		ALU_FLAGS[1] <= 1'b0;	//Zero Flag
		ALU_FLAGS[2] <= 1'b0;	//Negative Flag
		ALU_FLAGS[3] <= 1'b0;	//Two's complement overflow indicator 
		ALU_FLAGS[4] <= 1'b0;	//N?V for signed tests
		ALU_FLAGS[5] <= 1'b0;	//Half Carry Flag
		ALU_FLAGS[6] <= 1'b0;	//Transfer bit used by BLD and BST instructions
		ALU_FLAGS[7] <= 1'b0;	//Global Interrupt Enable/Disable Flag
		sp_inc_value <= {16{1'b0}};
		pc_offset <= {16{1'b0}};
		PC <= {16{1'b0}};
		step_cnt <= `STEP1;
	end
	else
	begin
		ALU_FLAGS[0] <= ALU_FLAG_C_OUT;	//Carry Flag
		ALU_FLAGS[1] <= ALU_FLAG_Z_OUT;	//Zero Flag
		ALU_FLAGS[2] <= ALU_FLAG_N_OUT;	//Negative Flag
		ALU_FLAGS[3] <= ALU_FLAG_V_OUT;	//Two's complement overflow indicator 
		ALU_FLAGS[4] <= ALU_FLAG_S_OUT;	//N?V for signed tests
		ALU_FLAGS[5] <= ALU_FLAG_H_OUT;	//Half Carry Flag
		ALU_FLAGS[6] <= ALU_FLAG_T_OUT;	//Transfer bit used by BLD and BST instructions
		ALU_FLAGS[7] <= ALU_FLAG_I_OUT;	//Global Interrupt Enable/Disable Flag
		step_cnt <= `STEP1;
		data_we <= 1'b0; // Clear "data_we" if not specified otherwise.
		write_to_io <= 1'b0;
		sp_inc_value <= {16{1'b0}};// No inrement nor decrement SP if not specified otherwise.
		SP <= SP + sp_inc_value;
		PC <= PC + pc_inc_value;// Increment PC by 1 if not specified otherwise.
		if(pc_offset)// If last instruction has been a relative jump, reload "pc_offset" with 0 for instruction increment.
			pc_offset <= {16{1'b0}};
		write_to_io_tmp <= write_to_io;// Synchronize "write_to_io" with "write_to_io_tmp" on high edge of "clk".
		case(step_cnt)
		`STEP1:
		begin
			casex(pgm_data)
			`INSTRUCTION_JMP:
			begin
				tmp_pgm_data <= pgm_data;
				step_cnt <= `STEP2;
			end
			`INSTRUCTION_RJMP:
			begin
				pc_offset <= {{5{pgm_data[11]}}, pgm_data[10:0]};
			end
			`INSTRUCTION_IJMP:
			begin
				PC <= rd_data_d;
			end
			`INSTRUCTION_CALL,
			`INSTRUCTION_ICALL,
			`INSTRUCTION_RCALL:
			begin
				step_cnt <= `STEP2;
				tmp_pgm_data <= pgm_data;
				data_out <= PC[7:0];// Put low byte of the PC.
				data_we <= 1'b1; // Put "data_we" to high to store low byte of the PC.
				TEMP8 <= PC[15:8];// Put high byte of the PC.
				sp_inc_value <= {16{1'b1}};// Decrement SP.
				if(pgm_data != `INSTRUCTION_CALL)
					pc_offset <= {16{1'b1}};// If the instruction is different than CALL freeze the PC counter.
				if(pgm_data == `INSTRUCTION_ICALL)
					TEMP16 <= rd_data_d;// Backup the reg Z value to a 16bit temporary register because the reading section of REG's is asynchronous.
				else
					TEMP16 <= {pgm_data[11], 4'h0, pgm_data[10:0]};// If is a relative CALL load the offset to "TEMP16".
			end
			`INSTRUCTION_POP_PUSH:
			begin
				step_cnt <= `STEP2;
				tmp_pgm_data <= pgm_data;
				if(~pgm_data[9])
					sp_inc_value <= 16'h0001;// Increment SP.
			end
			`INSTRUCTION_LDD_STD,
			`INSTRUCTION_LDS_STS,
			`INSTRUCTION_LD_ST_YZP,
			`INSTRUCTION_LD_ST_YZN,
			`INSTRUCTION_LD_ST_X,
			`INSTRUCTION_LD_ST_XP,
			`INSTRUCTION_LD_ST_XN:
			begin
				tmp_pgm_data <= pgm_data;
				step_cnt <= `STEP2;
			end
			`INSTRUCTION_XCH,
			`INSTRUCTION_LAS,
			`INSTRUCTION_LAC,
			`INSTRUCTION_LAT:
			begin
				tmp_pgm_data <= pgm_data;
				step_cnt <= `STEP2;
				data_addr <= rd_data_d;
			end
			`INSTRUCTION_IN_OUT:
			begin
				io_addr <= {pgm_data[10:9], pgm_data[3:0]};
				if(pgm_data[11])
				begin
					io_out <= rd_data_d;
					write_to_io <= 1'b1; // Put "data_we" to high to store the selected register.
				end
			end
			endcase
		end
		`STEP2:
		begin
			casex(tmp_pgm_data)
			`INSTRUCTION_JMP:
			begin
				PC <= pgm_data;
			end
			`INSTRUCTION_CALL,
			`INSTRUCTION_ICALL,
			`INSTRUCTION_RCALL:
			begin
				data_out <= TEMP8;// Put high byte of the PC from "REMP".
				data_we <= 1'b1; // Put "data_we" to high to store low byte of the PC.
				sp_inc_value <= {16{1'b1}};// Decrement SP.
				if(tmp_pgm_data == `INSTRUCTION_ICALL)
					PC <= TEMP16;// If is a indirect CALL instruction load "TMP16"(Z) to PC.
				else if(tmp_pgm_data == `INSTRUCTION_RCALL)
					pc_offset <= TEMP16;// If is a relative CALL load the "TMP16" to PC offset register.
				else
					PC <= pgm_data;// If is a CALL instruction, load the constant address to PC.
			end
			`INSTRUCTION_POP_PUSH:
			begin
				if(tmp_pgm_data[9])
				begin
					data_out <= rd_data_d;
					data_we <= 1'b1; // Put "data_we" to high to store the selected register.
					sp_inc_value <= {16{1'b1}};// Decrement SP.
				end
			end
			`INSTRUCTION_LDD_STD,
			`INSTRUCTION_LD_ST_YZN,
			`INSTRUCTION_LD_ST_X,
			`INSTRUCTION_LD_ST_XN:
			begin
				data_addr <= indirect_addr_offset_res;
				if(tmp_pgm_data[9])
				begin
					data_out <= rd_data_d;
					data_we <= 1'b1; // Put "data_we" to high to store the selected register.
				end
			end
			`INSTRUCTION_LDS_STS:
			begin
				data_addr <= pgm_data;
				if(tmp_pgm_data[9])
				begin
					data_out <= rd_data_d;
					data_we <= 1'b1; // Put "data_we" to high to store low byte of the PC.
				end
			end
			`INSTRUCTION_LD_ST_YZP,
			`INSTRUCTION_LD_ST_XP:
			begin
				data_addr <= rd_data_d;
				if(tmp_pgm_data[9])
				begin
					data_out <= rd_data_d;
					data_we <= 1'b1; // Put "data_we" to high to store the selected register.
				end
			end
			`INSTRUCTION_LD_ST_YZN:
			begin
				data_addr <= indirect_addr_offset_res;
				if(tmp_pgm_data[9])
				begin
					data_out <= rd_data_d;
					data_we <= 1'b1; // Put "data_we" to high to store low byte of the PC.
				end
			end
			`INSTRUCTION_XCH:
			begin
				data_out <= rd_data_d;
				data_we <= 1'b1; // Put "data_we" to high to store the selected register.
			end
			`INSTRUCTION_LAS,
			`INSTRUCTION_LAC,
			`INSTRUCTION_LAT:
			begin
				step_cnt <= `STEP3;
				TEMP8 <= rd_data_d;
			end
		endcase
		end
		`STEP3:
		begin
			casex(tmp_pgm_data)
			`INSTRUCTION_LAS:
			begin
				data_out <= data_in | TEMP8;
				data_we <= 1'b1; // Put "data_we" to high to store the selected register.
			end
			`INSTRUCTION_LAC:
			begin
				data_out <= data_in & ~TEMP8;
				data_we <= 1'b1; // Put "data_we" to high to store the selected register.
			end
			`INSTRUCTION_LAT:
			begin
				data_out <= data_in ^ TEMP8;
				data_we <= 1'b1; // Put "data_we" to high to store the selected register.
			end
			endcase
		end
		endcase
	end
end

mega_regs regs(
	.clk(clk),
	.rw_addr(rw_addr),
	.rw_data(rw_data),
	.rw_16bit(rw_16bit),
	.write(write_to_reg),
	.rd_addr_d(rd_addr_d),
	.rd_data_d(rd_data_d),
	.rd_16bit_d(rd_16bit_d),
	.read_d(1'b1),
	.rd_addr_r(rd_addr_r),
	.rd_data_r(rd_data_r),
	.rd_16bit_r(rd_16bit_r),
	.read_r(1'b1)
);

mega_alu alu(
	.inst(pgm_data),
	.in_addr_1(rd_addr_d),
	.in_addr_2(rd_addr_r),
	.in_1(alu_in_1),
	.in_2(alu_in_2),
	.out(alu_out),
	//.c_out(alu_c_out),
	.ALU_FLAG_C_IN(ALU_FLAGS[0]),		//Carry Flag
	.ALU_FLAG_Z_IN(ALU_FLAGS[1]),		//Zero Flag
	.ALU_FLAG_N_IN(ALU_FLAGS[2]),		//Negative Flag
	.ALU_FLAG_V_IN(ALU_FLAGS[3]),		//Two's complement overflow indicator 
	.ALU_FLAG_S_IN(ALU_FLAGS[4]),		//N?V for signed tests
	.ALU_FLAG_H_IN(ALU_FLAGS[5]),		//Half Carry Flag
	.ALU_FLAG_T_IN(ALU_FLAGS[6]),		//Transfer bit used by BLD and BST instructions
	.ALU_FLAG_I_IN(ALU_FLAGS[7]),		//Global Interrupt Enable/Disable Flag

	.ALU_FLAG_C_OUT(ALU_FLAG_C_OUT),	//Carry Flag
	.ALU_FLAG_Z_OUT(ALU_FLAG_Z_OUT),	//Zero Flag
	.ALU_FLAG_N_OUT(ALU_FLAG_N_OUT),	//Negative Flag
	.ALU_FLAG_V_OUT(ALU_FLAG_V_OUT),	//Two's complement overflow indicator 
	.ALU_FLAG_S_OUT(ALU_FLAG_S_OUT),	//N?V for signed tests
	.ALU_FLAG_H_OUT(ALU_FLAG_H_OUT),	//Half Carry Flag
	.ALU_FLAG_T_OUT(ALU_FLAG_T_OUT),	//Transfer bit used by BLD and BST instructions
	.ALU_FLAG_I_OUT(ALU_FLAG_I_OUT)		//Global Interrupt Enable/Disable Flag
);
    
`ifdef USE_CUSTOM_ADDER
add_w_carry # (.WIDTH(16)) add_PC(
		.c_in(1'b1),
		.in_1(pc_offset),
		.in_2(pc_offset),
		.out(pc_inc_value),
		.c_out()	
	);
`else
wire [16:0]pc_inc_value_tmp;
assign pc_inc_value_tmp = {pc_offset, 1'b1} + {pc_offset, 1'b1};
assign pc_inc_value = pc_inc_value_tmp[16:1];
`endif

assign io_we = write_to_io_tmp & clk;// If "write_to_io_tmp" is high put to "io_we" the "clk" signal.

endmodule
