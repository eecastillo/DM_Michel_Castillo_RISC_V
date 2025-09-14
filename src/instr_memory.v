// Coder:           David Adrian Michel Torres, Eduardo Ethandrake Castillo Pulido
// Date:            16/03/23
// File:			     instr_data_memory.v
// Module name:	  instr_data_memory
// Project Name:	  risc_v_top
// Description:	  Memory that contains instructions to perform

module instr_memory #(parameter DATA_WIDTH = 32, parameter ADDR_WIDTH = 32) (
	//inputs
	input [(DATA_WIDTH-1):0] address,
	input we, clk,
	//outputs
	output [(DATA_WIDTH-1):0] rd
);

// Declare the RAM array
reg [DATA_WIDTH-1:0] rom_memory [0:(ADDR_WIDTH*ADDR_WIDTH)-1];
wire [DATA_WIDTH-1:0] map_Address = (address + (~32'h400_000 + 1'b1)) >> 2'h2;

//Initial data with program to execute
initial begin
	// program
	$readmemh("../asm/multi_simple.txt", rom_memory);
end

always @(posedge clk)
begin
	//Write
	if (we)
		rom_memory[address] <= {DATA_WIDTH{1'b0}};  //RO memory
end

// Reading if memory read enable
assign rd = rom_memory[map_Address];

endmodule