/********************************************************************************
*       SDClockMux.v.     TFOX        Updated version 0.5     Jan 20, 2023        *
*           TFOX, N4TLF January 31, 2023   You are free to use it               *
*           however you like.  No warranty expressed or implied                 *
*   SDClock speed INPUT Multiplexer.  Uses a much higher clock to drive posedge   *
*       selection.         *
********************************************************************************/
module  SDClockMux
    (
    input           MHz10,
    input           kHz400,
    input           pll0_250MHz,
    input           SDClkSelect, 
    output  reg    SDLocalClk
    );

always @(posedge pll0_250MHz) 
    begin
        if(SDClkSelect)
            SDLocalClk = MHz10;
        else
            SDLocalClk = kHz400;
        end
    
endmodule
    
