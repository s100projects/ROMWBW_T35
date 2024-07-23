/********************************************************************
*   FILE:   cpuHAdrMux.v      Ver 0.2         Oct. 27, 2022         *
*   TFOX, N4TLF January 20, 2023   You are free to use it           *
*       however you like.  No warranty expressed or implied         *
*   Feb.17, 2023: Changed posedge and removed using wire for output *
*                                                                   *
********************************************************************/

module cpuHAdrMux
    (
    input [7:0] cpuHighAdr,
    input   sOUT,
    input   sINP,   
    input   pll0_250MHz,
    output reg [7:0] HighAdr
    );

always @(posedge pll0_250MHz) begin
    if (sOUT | sINP)
        HighAdr <= 8'b0;
    else
        HighAdr <= cpuHighAdr;  
    end
    
endmodule
