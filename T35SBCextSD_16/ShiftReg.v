/************************************************************************
*	File:  ShiftReg.v		TFOX		Ver. 0.2		11/1/22		    *
*																		*
*	A simple shift register.  Used with the USB interface to			*
*		set the end of USB transmission and possibly cue up next byte	*
*   TFOX, N4TLF January 20, 2023   You are free to use it               *
*       however you like.  No warranty expressed or implied             *
************************************************************************/

module ShiftReg
    (
    input           clk,
    input           clr,
    input           SerIn,
    output  [7:0]   qout);
   
reg [7:0] temp;

always @(posedge(clk) or negedge(clr))
    if(clr == 1'b0)
        temp <= 8'b0;
    else begin
        temp = {temp[6:0], SerIn};
    end
    
assign qout = temp;

endmodule


