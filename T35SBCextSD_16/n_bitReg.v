/************************************************************************
*   FILE:   n_bitReg.v     TFOX    Oct. 12, 2022   Ver. 0.1        		*
************************************************************************/

module n_bitReg
#(
    parameter N = 8)
    (input  load,
     input  clock,
     input  clr,
     input  [N-1:0] inData,
     //
     output reg [N-1:0] regOut
     );
     
always @(posedge clock or posedge clr) begin
    if(clr == 1)
        regOut <= 0;
    else if(load == 1)
        regOut <= inData;
	end
        
endmodule

