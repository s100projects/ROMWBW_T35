/************************************************************************
*   FILE:   n_bitLatchClr.v     TFOX    Feb. 20, 2023   Ver. 0.1        *
*   This is a simple n-bit Transparent latch, no clock with clear       *
************************************************************************/

module n_bitLatchClr3
#(
    parameter N = 8,
    parameter V = 0)
    (input  clk,
     input  load,               // out = in if THIS is high
                                // out = LATCHED in if load LOW
     input clr,
     input  [N-1:0] inData,
     output reg [N-1:0] regOut
     );
     
always @(posedge clk or negedge clr)
    begin 
        if(!clr)
        regOut <= V;
     else if (load)
        regOut <= inData;
     else
        regOut <= regOut;
    end
        
endmodule

