// Coder:           David Adrian Michel Torres
// Date:            14/09/25
// File:			     pipelined_multiplier.v
// Module name:	  pipelined_multiplier
// Project Name:	  risc_v_top
// Description:	  Pipelined multiplier

module pipelined_multiplier #(parameter LENGTH=32) (
    input i_clk,
    input i_rst_n,
    input [LENGTH-1:0] i_alu_Src_A,
    input [LENGTH-1:0] i_alu_Src_B,
    input [3:0] i_control,
    output reg [LENGTH-1:0] o_alu_result,
    output reg o_mult_active, o_mult_done
);  

//Combinational logic
wire do_mult;
wire [LENGTH-1:0] rst_mult;
assign do_mult = (i_control == 4'h2) ? 1'b1 : 1'b0;
assign rst_mult = i_alu_Src_A * i_alu_Src_B;

//Sequential logic
reg [LENGTH-1:0] p1, p2;
reg v1, v2;
always @(posedge i_clk, negedge i_rst_n) 
begin
    if(!i_rst_n) begin
        v1 <= 1'b0; v2 <= 1'b0; o_mult_done <= 1'b0;
        p1 <= {LENGTH{1'b0}}; p2 <= {LENGTH{1'b0}};
    end
    else begin
        o_alu_result <= p2; o_mult_done <= v2;
        p2 <= p1;           v2 <= v1;

        if (do_mult) begin
            p1 <= rst_mult;
            v1 <= 1'b1;
        end
        else begin
            p1 <= {LENGTH{1'b0}};
            v1 <= 1'b0;
        end
    end
end

always @(posedge i_clk, negedge i_rst_n) 
begin
    if(!i_rst_n) 
        o_mult_active <= 1'b0;
    else if (do_mult)
        o_mult_active <= 1'b1;
    else if (o_mult_done)
        o_mult_active <= 1'b0;
end

endmodule