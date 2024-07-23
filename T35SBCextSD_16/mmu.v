/************************************************************************
*   File:  mmu.v        TFOX        Ver 0.1     May 22, 2024            *
*       Memory Management Unit based on the Zeta V2 SBC MMU             *
*       Uses four eight-bit registers to hold memory address A14-A21    *
*       for four memory banks: 0000-3fff,4000-7fff,8000-bfff,c000-ffff  *
*   TFOX, N4TLF May 16, 2024   You are free to use it however you like  *
*      No warranty expressed or implied                                 *
************************************************************************/

module mmu
    (
    input           clk,
    input           n_reset,        // mmu enable/disable FF reset
    input  [7:0]    mmuDataIn,      // eight bit CPU Data OUT to mmu
    output [7:0]    mmuDataOut,
    input  [15:0]   address,
    input           MMUPageEnWrEn,        // CPU IN/OUT WRITE
    input           MMURegFileWrEn,
    input           MMURegFileRdEn,
    output [7:0]    mmuAdrOut      // eight bank addreses MA14-MA21
    );



wire [7:0]      RegFileAdrOut;
wire [1:0]      mmuRdAdr;

reg     pageEnable;

assign  mmuAdrOut   = (pageEnable) ? RegFileAdrOut : 8'h0;
assign  mmuRdAdr    = (MMURegFileRdEn) ? ({address[1], address[0]}) : ({address[15], address[14]});
assign  mmuDataOut  =  mmuAdrOut;

/*****************************************************************************
*   Paging enable/disable flip flop     Output Port 7Ch, bit 0 = 1 enables  **
*****************************************************************************/
always @(posedge clk)        
begin
    if (!n_reset)
        pageEnable <= 1'b0;
    else
    begin
        if (MMUPageEnWrEn)
        begin
            if (mmuDataIn[0])
                pageEnable <= 1'b1;
            else
                pageEnable <= 1'b0;
        end
    end
end

/*****************************************************************************
*   Actual 4 8-bit register file.
*****************************************************************************/

RegisterFile    #(.DataWidth(8), 
                .NumRegs(4))
  mmuRegs
  (
        .clk        (clk),
        .writeEn    (MMURegFileWrEn),
        .writeAddr  (address[1:0]),
        .writeData  (mmuDataIn),
        .readAddr   (mmuRdAdr),
        .readData   (RegFileAdrOut));
        
endmodule
   