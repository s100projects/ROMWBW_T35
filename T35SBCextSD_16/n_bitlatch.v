/************************************************************************
*   FILE:   n_bitLatch.v     TFOX    Oct. 20, 2022   Ver. 0.1        	*
*   This is a simple n-bit Transparent latch, no clock or clear         *
************************************************************************/

module n_bitLatch
#(
    parameter N = 8)
    (input  load,               // out = in if THIS is high
                                // out = LATCHED in if load LOW
     input  clock,
     input  [N-1:0] inData,
     //
     output reg [N-1:0] regOut
     );
     
always @(negedge clock)
//    begin
        if(load)
            regOut <= inData;
//    else
//        regOut <= inData;
//	end
        
endmodule

