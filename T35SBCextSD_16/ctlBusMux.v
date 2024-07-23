/********************************************************************
*   FILE:   cltBusMux.v      Ver 0.1    TFOX     Nov. 6, 2022       *
*   TFOX, N4TLF January 20, 2023   You are free to use it           *
*       however you like.  No warranty expressed or implied         *
*       2/17/23:  Changed posedge to add 250MHz clocking,           *
*                       and direct output to controlout             *
*                                                                   *
********************************************************************/

module ctlBusMux
    (
    input [4:0] controlin,
    input   select,
    input   pll0_250MHz,
    output reg [4:0] controlout
    );
    
always @(posedge pll0_250MHz) begin
    if (select)
        controlout <= 8'b0;
    else
        controlout <= controlin;  
    end

endmodule

