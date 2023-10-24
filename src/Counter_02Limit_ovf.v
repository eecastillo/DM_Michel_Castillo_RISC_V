// Coder:           Cuauhtemoc Aguilera
// Date:            09/09/22
// File:			     Counter_02Limit_ovf.v
// Module name:	  Counter_02Limit_ovf
// Project Name:	  risc_v_top
// Description:	  Counter overflow

module Counter_02Limit_ovf #(parameter N=3, parameter Limit = 4)(
    input rst_n,
    input enable,
    input clk,
    input clear,
    //output reg [N-1:0] Q,
    output overflow,
    output pre_overflow
 );

reg [N-1:0] Q;
assign overflow = (Q==Limit)? 1'b1 : 1'b0;
assign pre_overflow = (Q==(Limit-1'b1))? 1'b1 : 1'b0;

 always @(negedge rst_n, posedge clk)
  begin
    if(!rst_n)
        Q <= {N{1'b0}};
    else
        if(clear)
            Q <= {N{1'b0}};
        else if (enable)
            if (Q==Limit)
                Q <= {N{1'b0}};
            else
                Q <= Q + 1'b1;
        else
            Q <= Q;
  end 
endmodule
