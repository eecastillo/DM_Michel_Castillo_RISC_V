// Coder:           David Adrian Michel Torres, Eduardo Ethandrake Castillo Pulido
// Date:            16/03/23
// File:			     risc_v_top.v
// Module name:	  risc_v_top
// Project Name:	  risc_v_top
// Description:	  Main module for RISCV project

module risc_v_top (
	//Inputs - Platform
	input clk_50Mhz,
	input rst_n,
	//Input - RX
	input rx,
	//Output - Tx
	output tx
);

//Wires to interconnect modules
wire ALUSrcB, ALUSrcA, MemWrite, pc_src, jalr_o, regWrite, clk, pll_lock, mult_active, mult_done;
wire [1:0] mem2Reg;
wire [31:0] pc_prim, pc_out, adr, rd1_data, rd1_data_reg, rd2_data, rd2_data_reg, pc_next, pc_plus_4, pc_target;
wire [31:0] memory_out, instr2perf, wd3_wire, imm_gen_out; 
wire [31:0] SrcA, SrcB, alu_result, alu_result_final, pc_jal, mult_result;
wire [31:0] aluSrcA_2_fwdA, aluSrcB_2_fwdB, fwd_SW_2_wd, instr_stall;
wire [1:0] ForwardA, ForwardB, ForwardSW;
wire [2:0] alu_control;
wire[3:0] alu_operation;
wire [31:0] data_memory_2_slave, address_memory_2_slave, data_return_rom, data_return_ram, data_return_uart;
wire we_memory_2_rom, re_memory_2_rom, we_memory_2_ram, re_memory_2_ram, we_memory_2_uart, re_memory_2_uart;
wire PCEnable, PCWriteCond, bne, alu_zero, MemRead, alu_zero_bne;
wire [1:0] cs;
wire [31:0] decoded_address,ram_rom_data, gpio_data;
wire mem_select, stall_mux, pc_stall, if_id_stall;
wire [13:0] id_ex_controlpath_in;
wire nop_inject;
wire branch_validated;
wire nop_inject_delayed;
wire nop_inject_desition;
wire branch_flush_clear;
wire nop_inject_delayed_2;
wire branch_prediction;
wire [31:0] branch_prediction_target;
wire [31:0] mult_branch_predict_out;
wire branch_prediction_delayed;
wire prediction_checkout_ex_mem;
wire [31:0] pc_prev_plus_4;
wire [31:0] pc_target_jump;
wire pc_restore;


//Datapath Pipeline registers
wire [63:0] mult_if_pipe_out;
wire [63:0] if_id_datapath_out;
wire [192:0] id_ex_datapath_out;
wire [205:0]ex_mem_datapath_out;
wire [132:0]mem_wb_datapath_out;
//Control Path Pipeline registers
wire [13:0] id_ex_controlpath_out;
wire [7:0]ex_mem_controlpath_out;
wire [2:0]mem_wb_controlpath_out;	

///////////////////////FETCH//////////////////////////////////////////////


//`define SIMULATION
//`ifndef SIMULATION
//	pll_risc_v PLL_RISCV ( .refclk(clk_50Mhz), .rst(~rst_n), .outclk_0(clk), .locked(pll_lock) );
//`else
//	assign clk = clk_50Mhz;
//	assign pll_lock = 1'b1;
//`endif

assign clk = clk_50Mhz;
//assign pll_lock = 1'b1;

//PC multiplexor
multiplexor_param #(.LENGTH(32)) mult_pc (
	.i_a(pc_plus_4),
	.i_b(pc_target_jump),
	.i_selector((PCEnable|branch_flush_clear)&branch_validated),
	.out(pc_next)
);

//PC
ffd_param_pc_risk #(.LENGTH(32), .RST_VAL(32'h400_000)) ff_pc (
	.i_clk(clk), 
	.i_rst_n(rst_n), 
	.i_en(~pc_stall),
//	.pll_lock(pll_lock), //start the program when PLL is lock
	.d(mult_branch_predict_out),
	.q(pc_out)
);

adder #(.LENGTH(32)) adder_pc_4 (
	.i_a(32'h4),
	.i_b(pc_out),
	.q(pc_plus_4)
);

branch_prediction #(.DATA_WIDTH(32), .BRANCH_NO(8)) branch_predictor(
    .i_clk(clk),
    .i_rst_n(rst_n),
    .ex_mem_opcode(ex_mem_datapath_out[172:166]),
    .if_id_opcode(if_id_datapath_out[38:32]),
    .ex_mem_branch_taken(PCEnable),
    .ex_mem_branch_target(ex_mem_datapath_out[63:32]),
    .if_pc(if_id_datapath_out[31:0]),
    .ex_mem_pc(ex_mem_datapath_out[31:0]),
    .prediction(branch_prediction),
    .branch_target(branch_prediction_target),
	.prediction_checkout_ex_mem(prediction_checkout_ex_mem)
);


//branch predictor mux
multiplexor_param #(.LENGTH(32)) mult_branch_predict (
	.i_a(pc_next),
	.i_b(branch_prediction_target),
	.i_selector(branch_prediction),
	.out(mult_branch_predict_out)
);

//Memory ROM
instr_memory #(.DATA_WIDTH(32), .ADDR_WIDTH(10)) memory_rom (
	.address(pc_out),
	.rd(instr2perf),
	.clk(clk),
	.we(1'b0) //RO memory
);

ffd_param_clear_n #(.LENGTH(1)) nop_delayer(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(branch_flush_clear),
	.d(nop_inject),
	//outputs
	.q(nop_inject_delayed)
);

assign nop_inject_desition = (nop_inject | nop_inject_delayed) & (~branch_flush_clear);

multiplexor_param #(.LENGTH(64)) mult_if_pipe (
	.i_a({instr2perf,pc_out}),
	.i_b(64'h0),
	.i_selector(nop_inject_desition),
	.out(mult_if_pipe_out)
);



/////////////////////////FETCH->DECODE/////////////////////////////////////
//if_id_datapath
//	PC : 31:0
// Instruction : 63:32
wire [63:0] if_id_datapath_in = mult_if_pipe_out;

ffd_param_clear_n #(.LENGTH(64)) if_id_datapath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(~if_id_stall),
	.i_clear(1'b0),
	.d(if_id_datapath_in),
	//outputs
	.q(if_id_datapath_out)
);

/////////////////////////DECODE///////////////////////////////////////////

//IMMEDIATE GENERATOR
imm_gen immediate_gen(
	.i_instruction(if_id_datapath_out[63:32]),//instr2perf
	.o_immediate(imm_gen_out)
);

//Register file
register_file reg_file (
	.clk(clk),
	.we3(mem_wb_controlpath_out[2]),//regWrite),******************************
	.a1(if_id_datapath_out[51:47]),//instr2perf[19:15]),
	.a2(if_id_datapath_out[56:52]),//instr2perf[24:20]),
	.a3(mem_wb_datapath_out[132:128]),//instr2perf[11:7]),-***************
	.wd3(wd3_wire),
	.rd1(rd1_data_reg),
	.rd2(rd2_data_reg),
	.rst_n(rst_n)
);

jump_detection_unit jump_detection(
    .opcode(if_id_datapath_out[38:32]),
    .nop_inject(nop_inject)
);

control_unit cu (
	.opcode(if_id_datapath_out[38:32]),//instr2perf[6:0]),
	.func3(if_id_datapath_out[46:44]),//instr2perf[14:12]),
	.ALUSrcA(ALUSrcA),
	.ALUSrcB(ALUSrcB),
	.PCSrc(pc_src),
	.ALUOP(alu_control),
	.MemWrite(MemWrite),
	.MemRead(MemRead),
	.RegWrite(regWrite),
	.MemtoReg(mem2Reg),
	.JALR_o(jalr_o),
	.PCWriteCond(PCWriteCond),
	.BNE(bne)
);

hazard_detection_unit hazard_detection(
    .id_ex_memread(id_ex_controlpath_out[9]),
    .id_ex_rd(id_ex_datapath_out[139:135]),
    .if_id_rs1(if_id_datapath_out[51:47]),
    .if_id_rs2(if_id_datapath_out[56:52]),
	.branch_taken(branch_flush_clear),
    .pc_stall(pc_stall),
    .if_id_stall(if_id_stall),
    .stall_mux(stall_mux)
);

multiplexor_param #(.LENGTH(14)) mult_id_ex_control_path (
	.i_a({regWrite,mem2Reg,MemWrite,MemRead,pc_src,PCWriteCond,bne,jalr_o,alu_control,ALUSrcB,ALUSrcA}),
	.i_b(14'h0),
	.i_selector(stall_mux),
	.out(id_ex_controlpath_in)
);

//Stall control
// 1'b0  -->  if_id_datapath_out[63:32]
// 1'b1  -->  32'h0
// Out   -->  instr_stall [31:0]
multiplexor_param #(.LENGTH(32)) mux_stall_control (
	.i_a(if_id_datapath_out[63:32]),
	.i_b(32'h0),
	.i_selector(stall_mux),
	.out(instr_stall)
);


/////////////////////////DECODE->EXECUTE////////////////////////////////////
//id_ex_datapath
//	PC : 31:0
//	rd1 : 63:32
//	rd2	: 95:64
//	Imm	: 127:96
//	Instruction: 159:128
// branch_prediction : 160
// branch_prediction_target : 192:161


wire [192:0] id_ex_datapath_in = {branch_prediction_target,branch_prediction,instr_stall,imm_gen_out,rd2_data_reg,rd1_data_reg,if_id_datapath_out[31:0]};

ffd_param_clear_n #(.LENGTH(193)) id_ex_datapath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(branch_flush_clear),
	.d(id_ex_datapath_in),
	//outputs
	.q(id_ex_datapath_out)
);

//id_ex_controlpath
//EX:
//	ALUSrcA:	0
//	ALUSrcB:	1
//	ALUOp:	  4:2
//	JALR:		5
//MEM
//	BNE:		6
//	PCWriteCond:7
//	PCSrc:		8
//	MemRead:	9
//	MemWrite:  10
//WB
//	MemToReg: 12:11
//	RegWrite:	13	



ffd_param_clear_n #(.LENGTH(14)) id_ex_controlpath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(branch_flush_clear),
	.d(id_ex_controlpath_in),
	//outputs
	.q(id_ex_controlpath_out)
);


/////////////////////////EXECUTE////////////////////////////////////////////

ALU_control alu_ctrl(
    //inputs
	.i_opcode(id_ex_datapath_out[134:128]),//instr2perf[6:0]),
	.i_funct7(id_ex_datapath_out[159:153]),//instr2perf[31:25]),
	.i_funct3(id_ex_datapath_out[142:140]),//instr2perf[14:12]),
	.i_aluop(id_ex_controlpath_out[4:2]),//alu_control),
	 //outputs
    .o_alu_operation(alu_operation)
);

adder #(.LENGTH(32)) adder_jump (
	.i_a(id_ex_datapath_out[127:96]), //imm_gen_out),
	.i_b(id_ex_datapath_out[31:0]),  //pc_out),
	.q(pc_jal)
);

multiplexor_param #(.LENGTH(32)) mult_jalr (
	.i_a(pc_jal),
	.i_b(alu_result),
	.i_selector(id_ex_controlpath_out[5]),//jalr_o),
	.out(pc_target)
);

multiplexor_param #(.LENGTH(32)) mult_alu_srcB (
	.i_a(id_ex_datapath_out[95:64]),//rd2_data_reg),
	.i_b(id_ex_datapath_out[127:96]),//imm_gen_out),
	.i_selector(id_ex_controlpath_out[1]),//ALUSrcB),
	.out(aluSrcB_2_fwdB)
);

multiplexor_param #(.LENGTH(32)) mult_alu_srcA (
	.i_a(id_ex_datapath_out[31:0]),//pc_out),
	.i_b(id_ex_datapath_out[63:32]),//rd1_data_reg),
	.i_selector(id_ex_controlpath_out[0]),//ALUSrcA),
	.out(aluSrcA_2_fwdA)
);

//ForwardA - Mux
double_multiplexor_param #(.LENGTH(32)) forwardA_mux (
	.i_a(aluSrcA_2_fwdA),
	.i_b(wd3_wire),//mem_wb_datapath_out[95:64]),
	.i_c(ex_mem_datapath_out[96:65]),
	.i_d(32'h0),
	.i_selector(ForwardA),
	.out(rd1_data)
);

//ForwardB - Mux
double_multiplexor_param #(.LENGTH(32)) forwardB_mux (
	.i_a(aluSrcB_2_fwdB),
	.i_b(wd3_wire),//mem_wb_datapath_out[95:64]),
	.i_c(ex_mem_datapath_out[96:65]),
	.i_d(32'h0),
	.i_selector(ForwardB),
	.out(rd2_data)
);

ALU #(.LENGTH(32)) alu_block (
	.i_a(rd1_data),
	.i_b(rd2_data),
	.i_control(alu_operation),
	.o_alu_zero(alu_zero),
	.alu_result(alu_result)
);

//ForwardSW - Mux
double_multiplexor_param #(.LENGTH(32)) mult_fwd_SW (
	.i_a(id_ex_datapath_out[95:64]),
	.i_b(wd3_wire),
	.i_c(ex_mem_datapath_out[96:65]),
	.i_d(32'h0),
	.i_selector(ForwardSW),
	.out(fwd_SW_2_wd)
);

forward_unit fwd_unit (
	.ex_mem_regWrite(ex_mem_controlpath_out[7]),
	.mem_wb_regWrite(mem_wb_controlpath_out[2]),
	.ex_mem_rd(ex_mem_datapath_out[165:161]),
	.mem_wb_rd(mem_wb_datapath_out[132:128]),
	.id_ex_reg_rs1(id_ex_datapath_out[147:143]),
	.id_ex_reg_rs2(id_ex_datapath_out[152:148]),
	.ALUSrcB(id_ex_controlpath_out[1]),
	.forwardA(ForwardA),
	.forwardB(ForwardB),
	.forwardSW(ForwardSW)
);

//Pipelined Multiplier
pipelined_multiplier #(.LENGTH(32)) pipelined_mult (
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_alu_Src_A(rd1_data),
	.i_alu_Src_B(rd2_data),
	.i_control(alu_operation),
	.o_alu_result(mult_result),
	.o_mult_active(mult_active),
	.o_mult_done(mult_done)
);

multiplexor_param #(.LENGTH(32)) mult_alu_result (
	.i_a(alu_result),
	.i_b(mult_result),
	.i_selector(mult_done),
	.out(alu_result_final)
);

///////////////////////EXECUTE -> MEM//////////////////////////////////////

//ex_mem_datapath
//	PC : 31:0
//	PC_JUMP_TARGET : 63:32
//	ALU_zero	: 64
//	Alu_result_f: 96:65
//	Immediate	: 128:97
//	rd2: 	160:129	
//	rd:		165:161
//	opcode:	172:166
// branch_prediction : 173
// branch_prediction_target : 205:174


wire [205:0] ex_mem_datapath_in = {id_ex_datapath_out[192:161],id_ex_datapath_out[160],id_ex_datapath_out[134:128],id_ex_datapath_out[139:135],fwd_SW_2_wd,id_ex_datapath_out[127:96],alu_result_final,alu_zero,pc_target,id_ex_datapath_out[31:0]};

ffd_param_clear_n #(.LENGTH(206)) ex_mem_datapath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(branch_flush_clear),
	.d(ex_mem_datapath_in),
	//outputs
	.q(ex_mem_datapath_out)
);

//ex_mem_controlpath
//MEM
//	BNE:		0
//	PCWriteCond:1
//	PCSrc:		2
//	MemRead:	3
//	MemWrite:   4
//WB
//	MemToReg:  6:5
//	RegWrite:	7

wire [7:0] ex_mem_controlpath_in = {id_ex_controlpath_out[13:6]};

ffd_param_clear_n #(.LENGTH(8)) ex_mem_controlpath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(branch_flush_clear),
	.d(ex_mem_controlpath_in),
	//outputs
	.q(ex_mem_controlpath_out)
);

///////////////////////////MEM/////////////////////////////////////////////
//Memory map
master_memory_map #(.DATA_WIDTH(32), .ADDR_WIDTH(7)) memory_map (
	//CORES <--> Memory map
	.wd(ex_mem_datapath_out[160:129]),//rd2_data_reg),
	.address(ex_mem_datapath_out[96:65]),//alu_result),
	.we(ex_mem_controlpath_out[4]),//MemWrite),
	.re(ex_mem_controlpath_out[3]),//MemRead),
	.clk(clk),
	.rd(memory_out),
	//Memory_map <--> Slaves
	.map_Data(data_memory_2_slave),
	.map_Address(address_memory_2_slave),
	//Memory_map <--> RAM
	.HRData1(data_return_ram),
	.WSel_1(we_memory_2_ram),
	.HSel_1(re_memory_2_ram),
	//Memory_map <--> UART
	.HRData2(data_return_uart),
	.WSel_2(we_memory_2_uart),
	.HSel_2(re_memory_2_uart)
);

//Memory RAM
data_memory #(.DATA_WIDTH(32), .ADDR_WIDTH(9)) memory_ram (
	.wd(data_memory_2_slave),
	.address(address_memory_2_slave),
	.we(we_memory_2_ram),
	.re(re_memory_2_ram),
	.clk(clk),
	.rd(data_return_ram)
);

//UART
uart_IP #(.DATA_WIDTH(32)) uart_IP_module (
	.wd(data_memory_2_slave),
	.address(address_memory_2_slave),
	.we(we_memory_2_uart),
	.rst_n(rst_n),
	.clk(clk),
	.rd(data_return_uart),
	.rx(rx),
	.tx(tx)
);


assign PCEnable = (ex_mem_controlpath_out[2]/*pc_src*/ | (ex_mem_controlpath_out[1]/*PCWriteCond*/ & alu_zero_bne));

//Multiplexor to select between ZERO & NOT ZERO FOR BRANCHES
multiplexor_param #(.LENGTH(1)) mult_branch (
	.i_a(ex_mem_datapath_out[64]),//alu_zero),
	.i_b(~ex_mem_datapath_out[64]),//~alu_zero),
	.i_selector(ex_mem_controlpath_out[0]),//bne),
	.out(alu_zero_bne)
);

branch_control_unit branch_control(
    .take_branch(PCEnable),
    .opcode(ex_mem_datapath_out[172:166]),
	.prediction_checkout_ex_mem(ex_mem_datapath_out[173]),
    .clear(branch_flush_clear),
	.PC_restore(pc_restore),
	.prediction_target_ex_mem(ex_mem_datapath_out[205:174]),
	.real_target(ex_mem_datapath_out[63:32]),
	.branch_validated(branch_validated)
);


adder #(.LENGTH(32)) adder_pc_prev_4 (
	.i_a(32'h4),
	.i_b(ex_mem_datapath_out[31:0]),
	.q(pc_prev_plus_4)
);
multiplexor_param #(.LENGTH(32)) mult_pc_restore (
	.i_a(ex_mem_datapath_out[63:32]),
	.i_b(pc_prev_plus_4),
	.i_selector(pc_restore),
	.out(pc_target_jump)
);

///////////////////////////////////////MEM -> WB////////////////////////////////////////////////

//mem_wb_datapath
//	PC : 31:0
//	mem rd : 63:32
//	Alu_result	: 95:64
//	Immediate	: 127:96
//	rd:		132:128

wire [132:0] mem_wb_datapath_in = {ex_mem_datapath_out[165:161],ex_mem_datapath_out[128:97],ex_mem_datapath_out[96:65],memory_out,ex_mem_datapath_out[31:0]};

ffd_param_clear_n #(.LENGTH(133)) mem_wb_datapath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(1'b0),
	.d(mem_wb_datapath_in),
	//outputs
	.q(mem_wb_datapath_out)
);

//mem_wb_controlpath
//WB
//	MemToReg:  1:0
//	RegWrite:	2

wire [2:0] mem_wb_controlpath_in = {ex_mem_controlpath_out[7:5]};

ffd_param_clear_n #(.LENGTH(3)) mem_wb_controlpath_ffd(
	//inputs
	.i_clk(clk),
	.i_rst_n(rst_n),
	.i_en(1'b1),
	.i_clear(1'b0),
	.d(mem_wb_controlpath_in),
	//outputs
	.q(mem_wb_controlpath_out)
);


////////////////////////////////////////WB/////////////////////////////////////////////////////

double_multiplexor_param #(.LENGTH(32)) mult_alu_param (
	.i_a(mem_wb_datapath_out[95:64]),//alu_result),
	.i_b(mem_wb_datapath_out[63:32]),//memory_out),
	.i_c(mem_wb_datapath_out[31:0] + 32'h4),//pc_plus_4),
	.i_d(mem_wb_datapath_out[127:96]),//imm_gen_out),
	.i_selector(mem_wb_controlpath_out[1:0]),//mem2Reg),
	.out(wd3_wire)
);



endmodule