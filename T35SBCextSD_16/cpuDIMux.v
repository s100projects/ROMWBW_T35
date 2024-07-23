/********************************************************************
*   FILE:   cpuDIMux.v      Ver 0.7        April 12, 2023           *
*                                                                   *
*   This function selects which device's DATA OUT goes into the Z80 *
*   CPU DATA INPUT bus at any given time. It uses the device's      *
*   select line to enable that device's DATA OUT signals            *
*   The Efinix FPGAs do NOT have internal tri-state buffering       *
*   capabilities, so this is especially important to prevent        *
*   clashing of signals of the Z80 DATA INPUT bus.                  *
*   TFOX, N4TLF January 20, 2023   You are free to use it           *
*       however you like.  No warranty expressed or implied         *
*       Ver 0.3 1/30/23:  Added pll0_250MHz clocking                *
*       Ver 0.4 2/14/23:  Added/fixed IDE selection (project 10)    *
*                           and fixed = vs <= in else if            *
*       Ver 0.5 2/21/23 TFOX, added PS/2 Data and Status in         *
*       Ver 0.5 2/23/23 TFOX, added video ports                     *
*       Ver 0.6 3/1/23  TFOX, added Printer and Buzzer              *
*       Ver 0.65 3/11/23 TFOX, fixed nop on boot, and added generic *
*           S100 I/O board support                                  *
*		Ver 0.7 4/2023 TFOX		added vector interrupt and SD Card	*
********************************************************************/

module cpuDIMux
    (
    input [7:0] romData,
    input [15:0] rstAdr,
    input [7:0] ramaData,
    input [7:0] s100DataIn,
    input [7:0] ledread,
    input [7:0] iobyte,
    input [7:0] usbRxD,
    input [7:0] usbStatus,
    input [7:0] ps2kybdData,
    input [7:0] ps2StatInp,
    input [7:0] ramVGAData,
    input [7:0] inPtrStat,
    input [7:0] RTCDataToCPU,
    input [7:0] RTCSpiBusyFlag,
    input [7:0] intsToCpu,
    input [7:0] SDdataToCPU,
    input [7:0] SD_statusToCPU,
    input [7:0] mmu2cpuDataIn,
    
    input   reset_cs,
    input   rom_cs,
    input   c3En_cs,
    input   ladrEn_cs,
    input   hadrEn_cs,

    input   ram_cs,
//    input   inPortcon_cs,
    input   inLED_cs,
    input   iobyteIn_cs,
    input   usbStat_cs,
    input   usbRxD_cs,
    input   ide_cs,
    input   ps2DIn_cs,
    input   ps2StIn_cs,
    input   vgaRAM_cs,
    input   printerStat_cs,
    input   DataFmRTC_cs,
    input   RTCSpiBusy_cs,
    input   z80Read,
    input   intVectToCPU_cs,
    input   DataFmSD_cs,
    input   SD_status_cs,
    input   MMURegFileRdEn,
    input   pll0_250MHz,
    output reg [7:0] outData
    );

always @(posedge pll0_250MHz) begin
    if (rom_cs)
        outData <= romData;
        
    else if (c3En_cs)
        outData <= 8'hC3;
     else if (ladrEn_cs)
        outData <= rstAdr[7:0];
    else if (hadrEn_cs)
        outData <= rstAdr[15:8];
       
 //   else if (reset_cs)              // execute a NOP upon reset
 //       outData <= 8'h00;           // until at ROM @ F000
   else if (ide_cs)
        outData <= s100DataIn;
//    else if(inPortcon_cs)
//        outData <= s100DataIn;
    else if (ram_cs)
        outData <= ramaData;
    else if (inLED_cs)
        outData <= ledread;
    else if (iobyteIn_cs)
        outData <= iobyte;
     else if (usbRxD_cs)
        outData <= usbRxD;
     else if(usbStat_cs)
        outData <= usbStatus;
     else if(ps2DIn_cs)
        outData <= ps2kybdData;
     else if(ps2StIn_cs)
        outData <= ps2StatInp;
     else if(vgaRAM_cs)
        outData <= ramVGAData;
     else if(printerStat_cs)
        outData <= inPtrStat;
     else if(!DataFmRTC_cs)
        outData <= RTCDataToCPU;
     else if(RTCSpiBusy_cs)
        outData <= RTCSpiBusyFlag;
     else if(intVectToCPU_cs)
        outData <= intsToCpu;
     else if(DataFmSD_cs)
        outData <= SDdataToCPU;
     else if(SD_status_cs)
        outData <= SD_statusToCPU;
     else if (MMURegFileRdEn)
        outData <= mmu2cpuDataIn;
     else if(z80Read)
        outData <= s100DataIn;
     else  
        outData <= 8'h0;
    end
    
endmodule
