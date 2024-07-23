/************************************************************************
*   File:  portDecoder.v    TFOX    Ver 0.6     Oct.12, 2022            *
*       This module creates I/O Port _cs (chip selects) (active HIGH)   *
*       based on addresses A7-A0 only.                                  *
*   TFOX, N4TLF January 20, 2023   You are free to use it               *
*       however you like.  No warranty expressed or implied             *
*   Feb 12, 2023: Ver 0.5, added and fixed IDE Ports for 8255           *
*   March 12, 2023 Ver 0.6, added RTC, removed extra VGA cursor inputs  *
************************************************************************/

module portDecoder
    (
    input [7:0]     address,
    input           iowrite,            //sOUT signal
    input           ioread,             // sINP signal
    //
    output          outPortFF_cs,           // Port FF, OUT
//    output          inPortCon_cs,
    output          outFbarLEDs_cs,         // Port 06, OUT
    output          inFbarLEDs_cs,          // Port 06, IN
    output          outMiscCtl_cs,          // Port 07, OUT
    output          inIOBYTE_cs,
    output          outRAMA16_cs, 
    output          inUSBst_cs,
    output          inusbRxD_cs,
    output          outusbTxD_cs,
    output          idePorts8255_cs,
    output          ps2Status_cs,
    output          ps2Data_cs,
    output          vgaCX_out_cs,
    output          vgaCursorY_out_cs,
    output          vgaCursorCtl_out_cs,
    output          printer_cs,
    output          printerStat_cs,
    output          printerStrobe_cs,
    output          buzzerOut_cs,
    output          DataToRTC7_0_cs,
    output          DataToRTC15_8_cs,
    output          DataFmRTC_cs,
    output          RTCSpiBusy_cs,
    output          RTCSpi_cs,
    output          RTCSpiReadFF_cs,
    output          RTCSpiWrite1_cs,
    output          DataToSD_cs,
    output          DataFmSD_cs,
    output          SD_Clk_cs,
    output          SD_Card_select_cs,
    output          SD_status_cs,
    output          SDWrite_cs,
    output          SDRead_cs,
    output          MMUPageEnWrEn,
    output          MMURegFileWrEn,
    output          MMURegFileRdEn

    );
    
    assign outPortFF_cs         = (address[7:0] == 8'hff) && iowrite;
//    assign inPortCon_cs       = (address[7:1] == 7'h00) && ioread;
    assign ps2Status_cs         = (address[7:0] == 8'h02) & ioread;     // PS2 kybd status IN
    assign ps2Data_cs           = (address[7:0] == 8'h03) & ioread;     // PS2 kybd Data IN

    assign outFbarLEDs_cs       = (address[7:0] == 8'h06) & iowrite;   // Port 06 write
    assign inFbarLEDs_cs        = (address[7:0] == 8'h06) & ioread;    // port 06 read
    assign outMiscCtl_cs        = (address[7:0] == 8'h07) & iowrite;   // Port 07 out
    assign inIOBYTE_cs          = (address[7:0] == 8'h36) & ioread;    // IN = IOBYTE(switches)
    assign outRAMA16_cs         = (address[7:0] == 8'h36) & iowrite;   // Out 36 D0 = RAM A16
    assign inUSBst_cs           = (address[7:0] == 8'h34) & ioread;    // input USB status port
    assign inusbRxD_cs          = (address[7:0] == 8'h35) & ioread;    // USB UART Rx Data input
    assign outusbTxD_cs         = (address[7:0] == 8'h35) & iowrite;   // USB UART Tx Data output
    assign idePorts8255_cs      = (address[7:2] == 6'b001100 & (ioread | iowrite)); //
    assign vgaCX_out_cs         = (address[7:0] == 8'hC0) & iowrite;
    assign vgaCursorY_out_cs    = (address[7:0] == 8'hC1) & iowrite;
    assign vgaCursorCtl_out_cs  = (address[7:0] == 8'hC2) & iowrite;
    assign printer_cs           = (address[7:0] == 8'hC7) & iowrite;
    assign printerStat_cs       = (address[7:0] == 8'hC7) & ioread;
    assign printerStrobe_cs     = (address[7:0] == 8'hC6) & iowrite;
    assign buzzerOut_cs         = (address[7:0] == 8'h00) && iowrite;
    assign DataToRTC7_0_cs      = (address[7:0] == 8'h68) && iowrite;   // Address to RTC (D0-D7)
    assign DataFmRTC_cs         = (address[7:0] == 8'h68) && ioread;    // Data From RTC (D0-D7)
    assign RTCSpiBusy_cs        = (address[7:0] == 8'h6A) && ioread;    // SPI Busy In
    assign RTCSpi_cs            = (address[7:0] == 8'h6A) && iowrite;   // RTC_cs LATCH (bit D0)
    assign RTCSpiReadFF_cs      = (address[7:0] == 8'h6B) && ioread;    // SPI RTC READ TRIGGER
    assign RTCSpiWrite1_cs      = (address[7:0] == 8'h6B) && iowrite;   // SPI RTC WRITE TRIGGER
    assign DataToSD_cs          = (address[7:0] == 8'h6C) & iowrite;   // Data to SD via 8-bit SPI
    assign DataFmSD_cs          = (address[7:0] == 8'h6C) & ioread;    // Data Ftom SD via 8-bit SPI
    assign SD_Clk_cs            = (address[7:0] == 8'h6D) & iowrite;   // SD/SPI Clock speed select
    assign SD_Card_select_cs    = (address[7:0] == 8'h6E) & iowrite;   // SD card A/B select
    assign SD_status_cs         = (address[7:0] == 8'h6E) & ioread;    // SD card STATUS Input
    assign SDWrite_cs           = (address[7:0] == 8'h6F) & iowrite;   // SD SPI Write trigger
    assign SDRead_cs            = (address[7:0] == 8'h6F) & ioread;    // SD SPI Write trigger
    assign MMUPageEnWrEn        = ((address == 16'h7c) & iowrite) ? (1'b1) : (1'b0);
    assign MMURegFileWrEn       = (((address >= 16'h78) && (address <= 16'h7b)) & iowrite )? (1'b1) : (1'b0);
    assign MMURegFileRdEn       = (((address >= 16'h78) && (address <= 16'h7b)) & ioread )? (1'b1) : (1'b0);
    endmodule
