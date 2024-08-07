/************************************************************************
*   FILE:  T35SBCextSD_16_top.v    Ver .3    3/23/23                  	*
*                                                                     	*
*	This project adds SD Card access via an eight-bit SPI Interface		*
*		Work on several "fixes" were also created here first, before	*
*		being applied to several earlier projects						*
*   TFOX, N4TLF March 23, 2023   You are free to use it             	*
*       however you like.  No warranty expressed or implied           	*
*   TFOX, N4TLF, April 15, 2023     SD Card Interface working           *
*   TFOX, N4TLF, April 28, 2023     Interrupt code Seems fine           *
*   TFOX, N4TLF, April 29, 2023     RTC Seems to work, timing looks OK  *
*   TFOX, N4TLF, October 5, 023     Removed need for microcomputer.vhd  *
*                                   to expose hidden CPU clock divider  *
*   TFOX, N4TLF, October 8, 2023    Replaced nop reset to Monitor with  *
*                                   Jump On Reset, ala Altair & IMSAI   *
************************************************************************/

//**************    UART.V INFO FOR TESTING 2/16/24 ************************
//NOTE:  This has been modified by TFOX to comment out one of the tx drivers
//          at lines 90 and 94, which caused errors
//      9/10/23:  Also modified to fix intermitent RX data garbled by
//          adding a register to the rx input that now feeds rx_in

// VERSION TEST Feb.16, 2024 @2345.
//      9,600 baud= no SOH errors, ususally works
//      19,200 baud = NOT SOH retry every second to fourth packet
//            does eventually successfully transfer u.asm, 58 sectors
//      38,400 baud = NOT SOH retry every 5-6 packet, sometimes completes
//              file transfer 58 sectors.
//      57,600 baud = NOT SOH retry every 2-3 packet, sometimes completes
//              file transfer 58 sectors
// test uses u.com, Rx status in port 34, ani 01, jnz status loop
//                  Tx status in port 34, ani 02, jnz status loop
//***********************************************************************

module  T35SBCextSD_16_top(
                                                // FPGA SIGNAL PINS INPUTS
    clockIn,            // 50MHz input from onboard oscillator
    pll0_LOCKED,
    pll0_2MHz,
    pll0_50MHz,         // NOTE:  This also drives SBC_LEDs mux to 
                        // eliminate a build error
    pll0_250MHz,
                        // Next comes all the S100 bus signals
    s100_n_RESET,       // on SBC board reset button (GPIOT_RXP12)
    s100_DI,            // S100 Data In bus
    s100_xrdy,          // xrdy is S100 pin 3, on Mini Front Panel
                        // and Monahan Bus DIsplay Board (BDB)
    s100_rdy,           // second Ready signal, S100 pin
    s100_HOLD,
    ps2Clk,
    ps2Data,
    inPrnAck,
    inPrnBusy,
    //
    in_n_INTA,
    in_n_INTB,
    in_n_INTC,
    in_n_INTD,
    inEnableINTA,       // onboard jumper to enable/disable INTA
    s100_n_INT,
                                                // FPGA SIGNAL PINS OUTPUTS
    S100adr0_15,
    S100adr16_19,
    s100_DO,            // S100 SBC Data Out bus
   
    s100_pDBIN,
    s100_pSYNC,   
    s100_pSTVAL,
    s100_n_pWR,
    s100_sMWRT,
    s100_pHLDA,
    s100_PHI,
    s100_CLOCK,         // 2MHz Clock signal to S100 bus    
    s100_sHLTA,
    s100_sINTA,
    s100_n_sWO,
    s100_sMEMR,
    s100_sINP,
    s100_sOUT,
    s100_sM1,
    s100_PHANTOM,       // turn OFF Phantom LED on Front panels
    s100_ADSB,          // turn OFF these (ADSB & SDSB) LEDs on BDB
    s100_CDSB,          // turn OFF these LEDs on BDB
    
                    // Some of the SBC non-S100 output signals
    SBC_LEDs,           // The SBC LEDs for testing
    sw_IOBYTE,          // I/O Byte Switches  NOT USED AS Z80 IOBYTE HERE!
    outPrn,             // Printer Port output
    outPrnStrobe,
    out8255_n_cs,
    idePorts_n_rd,
    idePorts_n_wr,
    outIDE_n_rd,
    outIDE_n_wr,
    
    cpuClkOut_P19,
    spare_P1,
    spare_P17,
    spare_P32,
    spare_P33,

    ram_A16,
    ram_A17,
    ram_A18,
    ram_n_cs,
    ram_n_oe,
    ram_n_wr,
    biData_IN,
    biData_OUT,
    biData_OE,
    
    seg7,
    seg7_dp,
    boardActive,
    buzzer,
    diagLED,
    highRomLED,
    lowRomLED,
    highRamLED,

    s100PhantomLED,
    prnAckLED,
    usbTXbusyLED,
    usbRXbusyLED,
    usbRXData,
    usbTXData,
    usbCTS,
//    usbDTR,   //////////////////////////////////////////////////////////////////////////////////////
    vgaRed,
    vgaGreen,
    vgaBlue,
    vgaHSync,
    vgaVSync,
    
    rtcSpiClk,
    rtcSpiSI,
    rtcSpiSO,
    rtcSpiCS,
//    rtcPwrFail,//////////////////////////////////////////////////////////////////////////////////////////
    rtcIntLED,
    rtc_n_INT,
    
    sdCardClk,
    sdMOSI,
    sdMISO,
    sd_n_Acs,
    sd_n_Bcs,

    F_in_cdsb,
    F_in_sdsb,    
    F_add_oe,
    F_bus_stat_oe,
    F_out_DO_oe,
    F_out_DI_oe,
    F_bus_ctl_oe);
        
    input   clockIn;
    input   pll0_LOCKED;
    input   pll0_2MHz;
    input   pll0_50MHz;
    input   pll0_250MHz;
    input   usbRXData;
 //   input   usbDTR;                 //////////////////////////////////////////////////////////////////////////////////////
    input   [7:0] sw_IOBYTE;
    input   s100_n_RESET;
    input   s100_xrdy;
    input   s100_rdy;
    input   s100_HOLD;
    input   ps2Clk;
    input   ps2Data;
    input   inPrnAck;
    input   inPrnBusy;
    input   [7:0] s100_DI;
    input   rtcSpiSO;
    input   rtc_n_INT;
//    input   rtcPwrFail;             ////////////////////////////////////////////////////////////////////////////////////
    input   F_in_cdsb;
    input   F_in_sdsb;
    input   in_n_INTA;
    input   in_n_INTB;
    input   in_n_INTC;
    input   in_n_INTD;
    input   s100_n_INT;
    input   inEnableINTA;       // onboard jumper to enable(low, shorted)/or disable(open) INTA
    input   sdMISO;


    
    output  [15:0]S100adr0_15;
    output  [3:0] S100adr16_19;
    output  s100_pDBIN;
    output  s100_pSYNC; 
    output  s100_pSTVAL;
    output  s100_n_pWR;
    output  s100_sMWRT;
    output  s100_pHLDA;
    output  [7:0] s100_DO;
    output  s100_PHI;
    output  s100_CLOCK;
    output  s100_sHLTA;
    output  s100_sINTA;
    output  s100_n_sWO;
    output  s100_sMEMR;
    output  s100_sINP;
    output  s100_sOUT;
    output  s100_sM1;
    //
    output  [7:0] SBC_LEDs;
    output  [7:0] outPrn;
    output  outPrnStrobe;
    output  out8255_n_cs;       // output to 8255 chip select (pin 6)
    output  idePorts_n_rd;      // U27 pins 1&19 buffer 8255->DI bus
    output  idePorts_n_wr;      // U24 pins 1&19 buffer DO bus->8255
    output  outIDE_n_rd;        // to 8255 pin 5 (rd*)
    output  outIDE_n_wr;        // to 8255 pin 36 (wr*)
    output  cpuClkOut_P19;
    output  spare_P1;
    output  spare_P17;
    output  spare_P32;
    output  spare_P33;
    
    output  [6:0] seg7;
    output  seg7_dp;
    output  s100_PHANTOM;       // turn OFF phantom light
    output  s100_ADSB;          // turn OFF these LEDs (ADSB & SDSB) on BDB
    output  s100_CDSB;          // turn OFF these LEDs on BDB
    output  ram_A16;
    output  ram_A17;
    output  ram_A18;
    output  ram_n_cs;
    output  ram_n_oe;
    output  ram_n_wr;
    
    input   [7:0] biData_IN;
    output  [7:0] biData_OUT;
    output  [7:0] biData_OE;
    
    output  boardActive;
    output  buzzer;
    output  diagLED;
    output  highRomLED;
    output  lowRomLED;
    output  highRamLED;
    output  s100PhantomLED;
    output  prnAckLED;
    output  rtcSpiCS;
    output  rtcSpiClk;
    output  rtcSpiSI;
    output  rtcIntLED;
    output  usbTXbusyLED;
    output  usbRXbusyLED;
    output  usbTXData;
    output  usbCTS;
    output  vgaRed;
    output  vgaGreen;
    output  vgaBlue;
    output  vgaHSync;
    output  vgaVSync;
    
    output  sdCardClk;
    output  sdMOSI;
    output  sd_n_Acs;
    output  sd_n_Bcs;
   
    
    output  F_add_oe;
    output  F_bus_stat_oe;
    output  F_out_DO_oe;
    output  F_out_DI_oe;
    output  F_bus_ctl_oe;
    
///////////////////////////////////////////////////////////////////
//    parameter   romWaitvalue = 8'hFF;
//    parameter   ioWaitvalue = 8'hFF;
    parameter   jumpResetAddr = 16'hF000;

    wire    cpuClock;
    reg     T80Clock;
    
    wire    z80_n_m1;
    wire    z80_n_mreq;
    wire    z80_n_iorq;
    wire    z80_n_rd;
    wire    z80_n_wr;
    wire    z80_n_rfsh;
    wire    z80_n_halt;
    wire    z80_n_busak;
    wire    z80_n_wait;
    
    wire    [15:0]  cpuAddress;
    wire    [7:0]   cpuDataOut;
    wire    [7:0]   cpuDataIn;
    wire    [7:0]   mmu2cpuDataIn;
    wire    [7:0]   romOut;
    wire    [7:0]   out255;
    wire    [7:0]   sw_IOBYTE;
    wire    [7:0]   debugReg;
    wire    [7:0]   fbarSbcLeds;    // out/in port 06, BAR LEDs
    wire    [7:0]   miscControl;    // Out port 07, Misc control
    wire    [7:0]   outramA16;       // IN=IOBYTE switch, Out (d0)=RAM A16
    wire    [7:0]   usbStat;        // USB UART Input Status port
    wire    [7:0]   usbRxData;      // USB UART received data
    wire    [7:0]   usbTxData;      // USB UART Transmit Data
    wire    [7:0]   usbTxDelay;
    wire    [6:0]   ps2DataChar;
    wire    [7:0]   ps2kybdData;
    wire    [7:0]   ps2StatIn;
    wire    [7:0]   inPtrStat;
    wire    [7:0]   ma14_21;
    wire    n_reset;
    wire    n_resetLatch;
    wire    biOutEN;            // Bidirectional Bus OUTPUT ENABLE
    wire    romWait;
    wire    ioWait;
    wire    boardWait;
    wire    z80mreq;
    wire    memRD;        // Memory READ signal
    wire    memWR;
    wire    outFF;
    wire    inPortCON_cs;
    wire    c3En;
    wire    ladrEn;
    wire    hadrEn;
    wire    n_jorphant;

    wire    rom_cs;
    wire    ram_cs;
    wire    nop_n_cs;
    wire    miscCtl_cs;
    wire    inBarLED_cs;
    wire    outBarLED_cs;
    wire    iniobyte_cs;
    wire    inusbStat_cs;       // USB UART Status input select
    wire    inusbRxData_cs;     // USB UART Rx Data input select
    wire    outusbTxData_cs;    // USB UART Tx Data output select
    wire    printer_cs;
    wire    printerStat_cs;
    wire    printerStrobe_cs;
    wire    prnAck;
    wire    startBuzzer;
    wire    buzzerOut_cs;
    wire    runBuzzer;
    wire    stopBuzzer;
    wire    resetBuzzer;
    
            /////////////////////   RTC and 16-bit SPI wires    ///////////////
    wire    RTCWriteClr;
    wire    RTCWrite;          // Write dual FF output
    wire    RTCRead;           // Read dual FF output
    wire    RTCReadOut1;       // Read dual FF intermediate output
    wire    RTCWriteOut1;      // Write dual FF intermediate output
    wire    DataToRTC7_0_cs;    // Select line, RTC data input bits 7-0, PORT 68 OUT
    wire    DataToRTC15_8_cs;   // Select line, RTC data input bits 15-8, PORT 69 OUT
    wire    DataFmRTC_cs;       // Select line, RTC data back to CPU, PORT 69 IN
    wire    RTCSpiBusy_cs;        // Select line, SPI Busy FF clock, PORT 6A IN
    wire    RTCSpi_cs;          // Select line, RTC chip select latch, PORT 6A OUT
    wire    RTCSpiReadFF1_cs;   // First RTC SPI read FF clock, Port 6B IN
    wire    RTCSpiWrite1_cs;    // First RTC SPI write FF clock, Port 6B OUT
    wire    spiBusyFlag;        // SPI busy flag from 16-bit SPI interface 
    wire    LatchedspiRTCBusyFlag;  // RTC spi latched busy flag
    wire    [15:0]  DataOutToRTC;   // Data output register TO RTC SPI
    wire    [15:0]  DataInFromRTC;  // Data INPUT register from RTC SPI
    wire    [7:0]   RTCSpiBusyFlag;
    wire    [7:0]   RTCDataToCPU;
    wire    [1:0]   RTCslaveSelect;

    /////////////// Vector Interrupts wires /////////////////////////////////
    wire    [7:0]   intsIn;         // Interrupts in to latch 
    wire    [7:0]   intsLatched;    // Latched interrupts to encoder
    wire    [7:0]   intsToCpu;      // encoded interrupt vector to CPU
    wire            intVectToCPU_cs; // Select signal to cpuDImux
    wire            intToCPU_n;     //
    wire            z80_n_boardInt; // interrupt signal out to Z80 CPU
//    wire            intsGs;         // an interrupt vector has been created
    reg             z80_n_Enable;   // enable the Z80 int pin or not

//////////////////  SD via SPI wires        /////////////////////////////////    
    wire    [7:0]   DataToSD;           // Write Data Out to SD from CPU
    wire    [7:0]   DataFmSD;           // Read Data in from SD TO CPU
    wire    [7:0]   SD_status;          // SD SPI Status Info
    wire    [7:0]   SDdataToCPU;        // SD Data To CPU
    wire    [7:0]   SD_statusToCPU;     // SD status byte to CPU
    wire    [1:0]   SDSlaveSelect;
    wire    [1:0]   SDCardSelect;
    wire            DataToSD_cs;        // Data from CPU TO SD/SPI select flag
    wire            DataFmSD_cs;        // Data TO CPU from SD/SPI select flag
    wire            SD_Clk_cs;          // SD SPI Interface clock speed select
    wire            SD_Card_select_cs;
    wire            SD_status_cs;       // SD status to CPU select/enable flag
    wire            SDWrite_cs;         // SD Write trigger flag
    wire            SDRead_cs;          // SD Read trigger flag
//    wire            SD_n_Acs;           // SD Card A select
//    wire            SD_n_Bcs;           // SD Card B select
    wire            SDClkSpeed;         // SD Clock speed
    wire            SDWriteOut1;        // SD Write Out intermediate signal
    wire            SDReadOut1;         // SD Read intermediate signal
    wire            SDWrite;            // SD Write signal from FF to SPI interface
    wire            SDRead;             // SD Read signal from FF to SPI interface
    wire            SDWait;             // SD Wait signal
    wire            SDWriteClr;
    wire            SDspiBusy_cs;
    wire            SDSpi_cs;
    wire            SDSpiBusyFlag;
    wire            SDSpiClr;
    wire            SDLocalClk;
    wire            IntEncGs;
    reg             MHz25;
    
////////////////////////////////////////////////////////////////////////////////////
    
    wire    outrama16_cs;
    wire    n_inta;             // internal INTA signal
    wire    inta;               // TEMPorary POSITIVE INTA 
    wire    mrq_norfsh;         // memory request, NOT refesh
    wire    sWO;                // combined write out
    wire    sOUT;
    wire    sINP;
    wire    n_mwr;
    wire    iorq;               // inverted z80_iorq_n for wait state generation
    wire    liorq;              // latched iorq
    wire    psyncstrt;          // start of psync signal
    wire    psyncend;           // end of psync
    wire    endsync;
    wire    busin;              // active high bus in for S100
    wire    pstval;             // ps trobe value
    wire    psync;              // S100 pSync
    wire    io_output;          // IO OUTPUT signal
    wire    n_pWR;
    wire    z80_n_HoldIn;
    wire    pHLDA;
    wire    pDBIN;
    wire    romDisable;
    wire    romHigh;
    wire    usbDataReady;
    wire    usbUARTbusy;
    wire    usbByteRcvd;
    wire    usbBusyRcvg;
    wire    usbUARTOvrun;
    wire    usbUARTerror;
    wire    usbUARTFFull;
    wire    usbOverrun;
    wire    ide8255_cs;
    wire    ide8255_RD;
    wire    ps2CharAvail;
    wire    ps2Stat;                // high when character is waiting
    wire    ps2Status_cs;       // PS2 status port select
    wire    ps2Data_cs;         // PS2 data port select
    wire    ps2DataIn_cs;       // PS2 Data IN logic select
    wire    statDisable;
    wire    ctlDisable;
    
    reg [20:0]  counter;            // 21-bit counter for CPU clock
    wire [6:0]  z80_stat;           // z80 CPU status register
    wire [6:0]  statusout;          // z80 S100 status outputs
    wire [4:0]  controlBus;         // S100 Control signals mux in
    wire [4:0]  controlOut;         // S100 Control Signals Output to bus
    wire [15:0] buildAddress;         // S100 address build location
    wire [13:0] romAddress;
    wire [19:0] ramAddress;
    
//////////////////////////      VGA STUFF   /////////////////////////////////

    wire        vgaCurX_cs;
    wire        vgaCurY_cs;
    wire        vgaCurCtl_cs;
    wire        vgaCXout_cs;
    wire        vgaCurYout_cs;
    wire        vgaCurCtlout_cs;

    wire [11:0] textAdr;            // Text Address OUT to dual-port RAM, address 2
    wire [7:0]  textData;           // Text Data IN from dual-port RAM, port 2
    wire [11:0] fontAdr;            // font Address Out to ROM Address in
    wire [7:0]  fontData;           // Font Data IN from FPGA ROM
    wire [7:0]  ramVGAData;
    wire [7:0]  ocrxIn;
    wire [7:0]  ocryIn;
    wire [7:0]  octlIn;
    wire [7:0]  outCtl;
    wire        vgaRam_cs;
    wire        vgaRamWriteData;
    wire        vgaRamReadData;

    
    
//////////////////////////////////////  MISC TESTING/DEBUG STUFF    ///////////////////////////////////////////////////////////////////////////////
assign boardActive = !usbOverrun;     //!pll0_LOCKED;   // LED is LOW to turn ON

assign n_reset = s100_n_RESET;
localparam FPGA_VER = 7'b0001110;
//assign seg7 = 7'b0001110;               // The letter "F", Top segment is LSB
assign seg7_dp = !(n_resetLatch & counter[20]); // Tick to show activity
assign cpuClkOut_P19 = cpuClock;

assign  diagLED = !usbUARTerror;      //s100_n_INT;       //1'b1;

assign spare_P1  = s100_pSTVAL;
assign spare_P17 = pstval; 
assign spare_P32 = 1'b0;                         
assign spare_P33 = 1'b0;
assign usbRXbusyLED = !usbByteRcvd;
assign usbTXbusyLED = !usbUARTbusy;
assign usbCTS       = 1'b0;
//assign boardActive = inusbRxData_cs;   // LED is LOW to turn ON

////////////////////////////    TURN ON SBC BUFFERS FOR NOW
assign F_add_oe = !F_in_sdsb;            // Address Bus enable  GPIOB_TXN17
assign F_bus_stat_oe = !F_in_sdsb;       // Status bus enable   GPIOB_TXN19
assign F_out_DO_oe = !F_in_sdsb;         // S100 Data OUT bus   GPIOL_114
assign F_out_DI_oe = !F_in_sdsb;         // S100 Data IN bus    GPIOL_116
assign F_bus_ctl_oe = !F_in_cdsb;        // Control bus enable  GPIO_R120

///////////////////////////     Create various Z80 and S100 signals

assign  n_inta = !(!z80_n_m1 & !z80_n_iorq);        // create Interrupt ACK signal 
assign  io_output = (!z80_n_wr & !z80_n_iorq);      // create OUTPUT signal
assign  mrq_norfsh = ((!z80_n_mreq) & z80_n_rfsh);  // memory rqst, NOT during refresh
assign  n_mwr = !(mrq_norfsh & z80_n_rd);           // memory WRITE, NOT during refresh
assign  sWO = (n_mwr & z80_n_wr);                    // combined write out
assign  psyncstrt = !(inta | liorq | mrq_norfsh);   // START_SYNC NOR gate on Waveshare
assign  endsync = !(psyncend);

assign  z80mreq = !z80_n_mreq;
assign  psync = !(psyncstrt | endsync | !z80_n_rfsh);   // PSYNCRAW NOR gate on Waveshare
assign  busin = (n_inta & z80_n_rd);       // create the BUS IN signal
assign  pstval = !(psync & !cpuClock);   // create the pSTVAL signal//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//assign  pstval = !(psync & !T80Clock);   // create the pSTVAL signal//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
assign  n_pWR = !(endsync & (!z80_n_wr));
assign  sOUT = (!z80_n_wr & !z80_n_iorq);    // create the basic OUT status bit
assign  sINP = (!z80_n_rd & !z80_n_iorq);    // crfeate the basic INP status bit
assign  memRD = (!z80_n_rd & !z80_n_mreq);   // create the basic memory READ status bit
assign  memWR = (!z80_n_wr & !z80_n_mreq);   // create the basic memory WRITE status bit
assign  romDisable = !miscControl[1];
assign  romHigh = (miscControl[0] | miscControl[2]);
assign  iorq = !z80_n_iorq;                 // inverted iorq for wait state generation

assign boardWait = (romWait & ioWait);
assign buzzer = !startBuzzer;
//assign resetBuzzer = !stopBuzzer;     //***********************************************************************************************************
assign ps2DataIn_cs = ps2Data_cs & s100_pDBIN;
assign ps2kybdData[7] = 1'b0;           // keyboard data Port 3, test bit 7
assign ps2StatIn[7:1] = 7'b0;           // keyboard status Port 2

assign vgaRamReadData = vgaRam_cs & memRD & pDBIN;
assign vgaRamWriteData = vgaRam_cs & memWR & !n_pWR;

/************************************************************************************
*       HM628512 512k STATIC RAM CHIP INTERACE                                      *
************************************************************************************/
assign ram_A16 = ramAddress[16];        //outramA16[0];      // A16 to RAM chip
assign ram_A17 = ramAddress[17];          //S100adr16_19[1];           // A17 to RAM chip
assign ram_A18 = ramAddress[18];          //S100adr16_19[2];           // A18 to RAM chip
assign ram_n_cs = !ram_cs;      // Static RAM chip select signal (active low)
assign ram_n_oe = !memRD;       // Static RAM memory READ signal (active low)
assign ram_n_wr = !memWR;       // Static RAM memory WRITE signal (active low)

assign biData_OUT[7:0] = cpuDataOut[7:0];   // set bidirectional data to CPU data

assign biOutEN =  (memWR & ram_cs); // Create the biDir. Out Enable for RAM Writes
assign biData_OE[0] = biOutEN;      // LOW enables Bidirectional Data OUT 0
assign biData_OE[1] = biOutEN;      // LOW enables Bidirectional Data OUT 1
assign biData_OE[2] = biOutEN;      // LOW enables Bidirectional Data OUT 2
assign biData_OE[3] = biOutEN;      // LOW enables Bidirectional Data OUT 3
assign biData_OE[4] = biOutEN;      // LOW enables Bidirectional Data OUT 4
assign biData_OE[5] = biOutEN;      // LOW enables Bidirectional Data OUT 5
assign biData_OE[6] = biOutEN;      // LOW enables Bidirectional Data OUT 6
assign biData_OE[7] = biOutEN;      // LOW enables Bidirectional Data OUT 7

/*************************************************************************************
*           IDE Interface       For Compact Flash Card                               *
*************************************************************************************/
assign out8255_n_cs = !(ide8255_cs);                 // to 8255 chip select (pin 6)
assign idePorts_n_rd = (outIDE_n_rd | out8255_n_cs); // from 8255 to CPU (IN) bfr U27
assign idePorts_n_wr = (outIDE_n_wr | out8255_n_cs); // to 8255 from CPU (OUT) bfr U24
assign outIDE_n_rd = !(s100_sINP & pDBIN);          // to 8255 RD* (pin 5)
assign outIDE_n_wr = !(sOUT & !n_pWR);               // to 8255 WR* (pin 36)

/************************************************************************************
*   Z80 Status register.  Controls the seven Z80 Status out (s) bits.               *
************************************************************************************/
assign z80_stat[0] = !z80_n_halt;   // inverted z80 !HLTA
assign z80_stat[1] = sOUT;          // sOUT signal
assign z80_stat[2] = sINP;          // sIN signal
assign z80_stat[3] = memRD;         // sMEMR signal
assign z80_stat[4] = sWO;           // create sWO- signal
assign z80_stat[5] = !z80_n_m1;     // create the sM1 signal
assign z80_stat[6] = !n_inta;       // create sINTA signal

/************************************************************************************
*   Z80 Control Register.  Control bits out (p bits).                               *
************************************************************************************/
assign  controlBus[0] = psync;      // Active High, start of new bus cycle
assign  controlBus[1] = pstval;     // Active LOW, Indicates stable address & status
assign  controlBus[2] = pDBIN;      // Active HIGH, read strobe, slave can input data
assign  controlBus[3] = n_pWR;      // Active LOW, generalized write strobe to slaves
assign  controlBus[4] = pHLDA;      // Active HIGH, Perm Master relinquishing control

/************************************************************************************
*   Various Address bus creation or modification                                                          *
************************************************************************************/
assign buildAddress[7:0] = cpuAddress [7:0];
assign ramAddress[13:0] = buildAddress[13:0];
//assign ramAddress[16] = (!buildAddress[15] & outramA16[0]);
assign ramAddress[14] = ma14_21[0];
assign ramAddress[15] = ma14_21[1];
assign ramAddress[16] = ma14_21[2];
assign ramAddress[17] = ma14_21[3];
assign ramAddress[18] = ma14_21[4];
assign romAddress[11:0] = buildAddress[11:0];
assign romAddress[12] = miscControl[0];
assign romAddress[13] = miscControl[2];

assign  highRomLED = (~romHigh);
assign  lowRomLED = (romHigh);
assign  highRamLED = ~outramA16[0];
assign  prnAckLED = prnAck;

/********************************************************************************
*       USB Status Register                                                     *
********************************************************************************/
assign usbStat[0] = usbDataReady;   // USB UART Data Ready      JOHN CONIN 
assign usbStat[1] = usbUARTbusy;    // USB UART Tx busy         JOHN CONOUT
assign usbStat[2] = usbByteRcvd;    // USB UART byte received
assign usbStat[3] = usbBusyRcvg;    // USB UART busy receiving
assign usbStat[4] = 1'b0;
assign usbStat[5] = usbUARTFFull;   // USB UART FIFO Full
assign usbStat[6] = usbUARTOvrun;   // USB UART Overrun...cleared by read
assign usbStat[7] = usbUARTerror;   // USB UART receive error
//assign usbRXbusyLED = !usbByteRcvd;
//assign usbTXbusyLED = !usbUARTbusy;
//assign usbCTS       = 1'b1;

/********************************************************************************
*       Printer Port Status Register (IN C7)                                    *
********************************************************************************/
assign inPtrStat[0] = prnAck;
assign inPtrStat[1] = inPrnBusy; 
assign inPtrStat[2] = 1'b1;
assign inPtrStat[3] = 1'b1; 
assign inPtrStat[4] = 1'b1;
assign inPtrStat[5] = 1'b1;
assign inPtrStat[6] = 1'b1;
assign inPtrStat[7] = 1'b1;


//////////////////////////////////////  FIXED S100 SIGNALS HERE /////////////////////
// s100_pDBIN created by read strobe FF below
assign s100_pDBIN = pDBIN;
assign z80_n_wait = s100_xrdy & s100_rdy & boardWait;      // Z80 Wait = low to wait
assign s100_pSYNC = psync;
assign s100_pSTVAL = pstval;
assign s100_n_pWR = n_pWR;
assign s100_PHI = T80Clock;
assign s100_CLOCK = !pll0_2MHz;             
assign s100_sMWRT = !(n_pWR | io_output);
assign s100_pHLDA = !z80_n_busak;
assign s100_DO = cpuDataOut;                  // S100 Data OUT bus signals
 
//////////////////////////////////////////////////////////////////////////////////////
//      Status Output signals, per IEEE-696.  These signals are latched by pSTVAL   //
//          based on John Monahan's Waveshare design.                               //
//          Status signals are prefixed with an "s"                                 //
//////////////////////////////////////////////////////////////////////////////////////
assign  s100_sHLTA = statusout[0];      //!z80_n_halt;           //statusout[0];
assign  s100_sOUT =  statusout[1];      //sOUT;              //statusout[1];
assign  s100_sINP =  statusout[2];      //sINP;              //statusout[2];
assign  s100_sMEMR = statusout[3];      //memRD;           //statusout[3];
assign  s100_n_sWO = statusout[4];      //sWO;               //statusout[4];
assign  s100_sM1 =   statusout[5];      //!z80_n_m1;             //statusout[5];
assign  s100_sINTA = statusout[6];      //!n_inta;           //statusout[6];

//////////////////////////////////////////////////////////////////////////////////
//  Control Output signals, per IEEE-696.  These signals can be tristated based //
//  on John Monahan's Waveshare design.  Efinix does not have tri-state         //
//  outputs internally, so these outputs are set high instead.... for now       //
//  These signals are prefixed with a "p"                                       //
//////////////////////////////////////////////////////////////////////////////////
//assign  s100_pSYNC = controlOut[0];
//assign  s100_pSTVAL = cntrolOut[1];
//assign  s100_pDBIN = controlOut[2];
//assign  s100_n_pWR = controlOut[3];
//assign  s100_pHLDA = controlOut[4];

assign s100_PHANTOM = 0;
assign s100PhantomLED = ~s100_PHANTOM; 
assign s100_ADSB = !statDisable;                // Address and Status Disable
assign s100_CDSB = !ctlDisable;                 // Control Signals Disabe



//////////////////////////////////////////////////////////////////////////////
//      Debug Register.  These are displayed when IOBYTE switches 5 & 4     //
//          are set to 10 (OFF ON)                                          //
//////////////////////////////////////////////////////////////////////////////
assign debugReg[0] = !ram_cs;
assign debugReg[1] = !n_reset;
assign debugReg[2] = !nop_n_cs;
assign debugReg[3] = !rom_cs;
assign debugReg[4] = s100_pDBIN;
assign debugReg[5] = s100_sMEMR;
assign debugReg[6] = s100_sINP;
assign debugReg[7] = s100_sOUT;



/****************************************************************************
*       Make 25MHz clock from 50MHz PLL Output                              *
****************************************************************************/
always @(posedge pll0_50MHz)
    begin
        MHz25 <= ~MHz25;
    end

/****************************************************************************
*   Basic counter/divider based on 2MHz PLL Clock                           *
*       The counter creates some slower speeds used below for debugging     *
****************************************************************************/
always @(posedge pll0_2MHz)                                                 
    begin
        if(n_reset == 0) begin      // if reset set low...
            counter <= 21'b0;       // reset counter to 0
        end                         // end of reset counter
        else
            counter <= counter + 1; // increment counter
    end

 /*******************************************************************************
*   CPU Clock input Mux.  Selects one of four clock frequencies                 *
*       Consider this as a sort of CPU clock frequency preselector.  It has four*
*       clock inputs, the output is fed into the actual CPU clock speed         *
*       This allows one to select a typical fast CPU speed, or very slow speeds *
*       for front panel LED testing.                                            *
********************************************************************************/
ClockMux    ClkMux(
    .MHz2        (pll0_2MHz),
    .MHz50       (pll0_50MHz),                       //(MHz25),
    .KHz31       (counter[5]),
    .Hz250       (counter[12]),
    .pll0_250MHz (pll0_250MHz),
    .sw          (sw_IOBYTE[7:6]), 
    .cpuclk      (cpuClock));
    
/****************************************************************************
*   CPU Clock generator, removed from microcomputer.vhd                     *
*       10/14/23, changed this from a max 25MHz input to a max of 50MHz     *                                                                        *
****************************************************************************/
// with divider set to 24 & 12, T80 CPU clock is 2.0MHz, 50% duty cycle
// with divider set to 11 & 6, T80 CPU clock is 4.167 MHz, 50% duty cycle
// with divider set to 9 & 5, T80 CPU clock is 5.0 MHz, 50% duty cycle
// with divider set to 7 & 3, T80 CPU clock is 6.25 MHz, 5 high, 3 low
// with divider set to 6 & 3, T80 CPU clock is 7.14 MHz, 4 high, 3 low **
// with divider set to 4 & 2, T80 CPU clock is 10.0 MHz, 3 clock high, 2 low
// with divider set to 3 & 2, T80 CPU clock is 12.5MHz, 50% duty cycle
// with divider set to 2 & 2, T80 CPU clock is 16.7MHz, 1 clock high, 2 low
// with divider set to 1 & 1, T80 CPU fails to run
// note:  above 7MHz, the voice module on the serial board 4.2 may not work

    reg [5:0]   cpuClkCount;
    
    always @(posedge cpuClock)      // Input is from cpuClock mux above
    begin
        if(cpuClkCount < 6)
                cpuClkCount <= cpuClkCount + 1;
        else
                cpuClkCount <= 0;
        if(cpuClkCount < 3)
               T80Clock <= 1'b0;
        else
               T80Clock <= 1'b1;
        end

/****************************************************************************
*   t80s    Z80 CPU emulator
****************************************************************************/
t80s    t80s1(
        .reset_n    (n_reset),              // INPUT  LOW to reset
		.clk_n      (T80Clock),
		
		.wr_n       (z80_n_wr),
		.rd_n       (z80_n_rd),
		.mreq_n     (z80_n_mreq),
		.iorq_n     (z80_n_iorq),
		.wait_n		(z80_n_wait),
        .int_n      (z80_n_boardInt | inEnableINTA),
        ////         L=INT, H=noInt   L=EN_INT, H=Disable  
		.nmi_n      (1'b1),
        .busrq_n    (z80_n_HoldIn),
        .m1_n       (z80_n_m1),
        .rfsh_n     (z80_n_rfsh),
        .halt_n     (z80_n_halt),
		.busak_n    (z80_n_busak),
		.a          (cpuAddress),
		.do         (cpuDataOut),
		.di          (cpuDataIn)	
		);
        
/************************************************************************************
*   S100 High Address MUX.  Z80 sends I/O Data OUT on A8-A15.  This disables that   *
************************************************************************************/
cpuHAdrMux  HighAdrMux(
    .cpuHighAdr     (cpuAddress[15:8]),     // Address for all but INPUT or OUTPUT
    .sOUT           (sOUT),                 // Zero out A15-08 if an OUTPUT
    .sINP           (sINP),                 // or an INPUT  
    .pll0_250MHz    (pll0_250MHz),
    .HighAdr        (buildAddress[15:8]));  // Send modified A15-08 to BUILD Address
        
 /************************************************************************************
*   CPU Data INPUT Multiplexer      Note: Efinix FPGAs do NOT have tristate ability  *
*************************************************************************************/
cpuDIMux    cpuInMux (                      // These are the actual eight-bit data
                                            // busses muxed to the CPU
    .romData        (romOut[7:0]),
    .rstAdr         (jumpResetAddr[15:0]),
    .s100DataIn     (s100_DI[7:0]),
    .ramaData       (biData_IN[7:0]),
    .ledread        (fbarSbcLeds[7:0]),
    .iobyte         (sw_IOBYTE[7:0]),    
    .usbStatus      (usbStat[7:0]),
    .usbRxD         (usbRxData[7:0]),
    .ps2kybdData    (ps2kybdData[7:0]),
    .ps2StatInp     (ps2StatIn[7:0]),
    .ramVGAData     (ramVGAData[7:0]),
    .inPtrStat      (inPtrStat[7:0]),
    .RTCDataToCPU   (RTCDataToCPU[7:0]),
    .RTCSpiBusyFlag (RTCSpiBusyFlag[7:0]),
    .intsToCpu      (intsToCpu[7:0]),
    .SDdataToCPU    (SDdataToCPU[7:0]),
    .SD_statusToCPU (SD_statusToCPU[7:0]),
    .mmu2cpuDataIn   (mmu2cpuDataIn),
                                            // and these are the one-bit enables to
                                            // select what data from above goes to CPU
    .reset_cs       (!nop_n_cs),
    .rom_cs         (rom_cs),
    .c3En_cs        (c3En),
    .ladrEn_cs      (ladrEn),
    .hadrEn_cs      (hadrEn),    
    .ram_cs         (ram_cs),
    .inLED_cs       (inBarLED_cs),
    .iobyteIn_cs    (iniobyte_cs),  
    .usbStat_cs     (inusbStat_cs),
    .usbRxD_cs      (inusbRxData_cs),
    .ide_cs         (ide8255_cs),
    .ps2DIn_cs      (ps2DataIn_cs),
    .ps2StIn_cs     (ps2Status_cs),
    .vgaRAM_cs      (vgaRamReadData),
    .printerStat_cs (printerStat_cs),
    .DataFmRTC_cs   (!DataFmRTC_cs),
    .RTCSpiBusy_cs  (RTCSpiBusy_cs),
    .z80Read        (!z80_n_rd),
    .intVectToCPU_cs (intVectToCPU_cs),
    .DataFmSD_cs    (DataFmSD_cs),
    .SD_status_cs   (SD_status_cs),
    .MMURegFileRdEn (MMURegFileRdEn),
    .pll0_250MHz    (pll0_250MHz),
    .outData        (cpuDataIn[7:0])
    );
 
/************************************************************************************
*   Memory decoder                                                                  *
************************************************************************************/     
memAdrDecoder  mem_cs(
    .address        (buildAddress[15:12]),  // change to [15:9] for small ROM (prj 6)
    .memwrite       (memWR),
    .memread        (memRD),
//    .reset_cs       (nop_n_cs),
    .n_jorphant     (n_jorphant),
    .romDisable     (romDisable),
    .rom_cs         (rom_cs),
    .ram_cs         (ram_cs),
    .vgaRam_cs      (vgaRam_cs)
     );

/************************************************************************************
*   Jump to address on reset                                                        *
*       This jumps directly to an address in parameter jumpResetAddr (usually F000) *
*       by forcing a jump instruction *C3h) followed by the low then high address   *
*       bytes directly into the CPU Data In multiplexer                             *
*       This is more like Altair, IMSAI, and other older S100 computers             *
************************************************************************************/

wire    c3ff;
wire    ladrff;
wire    hadrff;
wire    jorphant;

dff3     jorc3(                         // jump on reset, C3 (jump) enable
        .clk        (!pDBIN),
        .pst_n      (1'b1),
        .clr_n      (n_reset),     
        .din        (1'b1),
        .q          (c3ff)
        );
assign c3En = !(c3ff | !pDBIN);

dff3     jorLadr(                       // jump on reset, LOW address enable
        .clk        (!pDBIN),
        .pst_n      (n_reset),
        .clr_n      (1'b1),     
        .din        (c3ff),
        .q          (ladrff)
        );
assign ladrEn = !(ladrff | !pDBIN);

dff3     jorHadr(                       // jump on reset, HIGH address enable
        .clk        (!pDBIN),
        .pst_n      (n_reset),
        .clr_n      (1'b1),     
        .din        (ladrff),
        .q          (hadrff)
        );
assign hadrEn = !(hadrff | !pDBIN);

dff3     jorphan(                       // jump on reset, HIGH address enable
        .clk        (hadrff),
        .pst_n      (n_reset),
        .clr_n      (1'b1),     
        .din        (1'b0),
        .q          (jorphant)
        );
assign n_jorphant = !jorphant;
 
     
/************************************************************************************
*   Boot ROM for Z80 CPU                                                            *
************************************************************************************/     
rom   #(.ADDR_WIDTH(14),                    // set address width for larger ROMs
	.RAM_INIT_FILE("fpgamon.inithex"))
    sbc_rom (
    .address    (romAddress[13:0]),     //(cpuAddress[10:0]),
	.clock      (T80Clock),
	.data       (romOut[7:0])
);

/************************************************************************************
    IO Ports Decoder.                                                               *
************************************************************************************/
portDecoder ports_cs(
    .address                (cpuAddress[7:0]),
    .iowrite                (sOUT),    
    .ioread                 (sINP),
    .outPortFF_cs           (outFF),
    .outFbarLEDs_cs         (outBarLED_cs),
    .inFbarLEDs_cs          (inBarLED_cs),
    .outMiscCtl_cs          (miscCtl_cs),
    .inIOBYTE_cs            (iniobyte_cs),
    .inUSBst_cs             (inusbStat_cs),
    .inusbRxD_cs            (inusbRxData_cs),
    .outusbTxD_cs           (outusbTxData_cs),
    .outRAMA16_cs           (outrama16_cs),
    .idePorts8255_cs        (ide8255_cs),
    .ps2Status_cs           (ps2Status_cs),
    .ps2Data_cs             (ps2Data_cs),
    .vgaCX_out_cs           (vgaCXout_cs),
    .vgaCursorY_out_cs      (vgaCurYout_cs),
    .vgaCursorCtl_out_cs    (vgaCurCtlout_cs),
    .printer_cs             (printer_cs),
    .printerStat_cs         (printerStat_cs),
    .printerStrobe_cs       (printerStrobe_cs),
    .buzzerOut_cs           (buzzerOut_cs),
    .DataToRTC7_0_cs        (DataToRTC7_0_cs),
    .DataToRTC15_8_cs       (DataToRTC15_8_cs),
    .DataFmRTC_cs           (DataFmRTC_cs),
    .RTCSpiBusy_cs          (RTCSpiBusy_cs),
    .RTCSpi_cs              (RTCSpi_cs),
    .RTCSpiReadFF_cs        (RTCSpiReadFF1_cs),
    .RTCSpiWrite1_cs        (RTCSpiWrite1_cs),
    .DataToSD_cs            (DataToSD_cs),
    .DataFmSD_cs            (DataFmSD_cs),
    .SD_Clk_cs              (SD_Clk_cs),
    .SD_Card_select_cs      (SD_Card_select_cs),
    .SD_status_cs           (SD_status_cs),
    .SDWrite_cs             (SDWrite_cs),
    .SDRead_cs              (SDRead_cs),
    .MMUPageEnWrEn          (MMUPageEnWrEn),
    .MMURegFileWrEn         (MMURegFileWrEn),
    .MMURegFileRdEn         (MMURegFileRdEn)
     );
  
/************************************************************************************
*   Z80 CPU status bits latch.  Output feeds the S100 status bit (sXXXX)            *
************************************************************************************/
n_bitLatch      #(7)
      s100stat(
     .load      (F_in_sdsb),       //   (!pstval),
     .clock     (pstval),           //(!T80Clock),
//     .clr       (1'b0),
     .inData    (z80_stat),
     .regOut    (statusout)
     );

 /************************************************************************************
*   S100 Address 0-15 Latch.     Latches address bus for S100 timing                *
************************************************************************************/
n_bitReg        #(16)
      s100adr(
     .load      (pstval),
     .clock     (!T80Clock),
     .clr       (1'b0),
     .inData    (ramAddress),       // Was buildAddress
     .regOut    (S100adr0_15)
     );

/************************************************************************************
*   S100 Address 16-19 Latch.     Latches address bus A16-A19 for S100 timing       *
************************************************************************************/
n_bitReg        #(4)
      s100adr16_19(
     .load      (pstval),
     .clock     (!T80Clock),
     .clr       (1'b0),
     .inData    (4'b0),
     .regOut    (S100adr16_19)
     );

/************************************************************************************
*   S100 Control Bus Signals MUX.  Sets Control bus to Z80 signals or all high      *
************************************************************************************/
ctlBusMux  CtlBusMux(
    .controlin      (controlBus),
    .select         (!n_resetLatch),
    .pll0_250MHz    (pll0_250MHz),
    .controlout     (controlOut)        
    );

/************************************************************************************
*   FBAR Diagnostic LEDs on Port 06 (also readback capability)                      *
************************************************************************************/
n_bitReg    outPort06(
 //    #(parameter N = 8)(
     .load      (outBarLED_cs),
     .clock     (T80Clock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (fbarSbcLeds));


/************************************************************************************
*   Output Port 07      Misc Control bits                                           *
************************************************************************************/
n_bitReg    outPort07(
 //    #(parameter N = 8)(
     .load      (miscCtl_cs),
     .clock     (T80Clock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (miscControl));

/************************************************************************************
*   Port 35  Out = USB Tx Data Out                                                  *
************************************************************************************/
n_bitReg    outUSBport35(
 //    #(parameter N = 8)(
     .load      (outusbTxData_cs),
     .clock     (T80Clock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (usbTxData));
     
     
/************************************************************************************
*   Port 36  IN = IOBYTE switches,     Out = RAM A16 Page Low/High (using D0)       *
************************************************************************************/
n_bitReg    outPort36(
 //    #(parameter N = 8)(
     .load      (outrama16_cs),
     .clock     (T80Clock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (outramA16));
   
/************************************************************************************
*   S100 output Port 255 (0xFF) to Front Panel LEDs                                 *
************************************************************************************/
n_bitReg    outPortFF(
 //    #(parameter N = 8)(
     .load      (outFF),
     .clock     (T80Clock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (out255)
    );
    
/*************************************************************************************
*   onboard LEDs INPUT Multiplexer.  This allows quick troubleshooting               *
*       NOTE NOTE   This module INVERTS the output for driving the LEDs              *
*************************************************************************************/
LedBarMux       lmux(
    .cpuDO          (cpuDataOut [7:0]),     // if both switches UP (00)
    .cpuDI          (cpuDataIn [7:0]),      // if Switches UP DN (01)
    .fbarSbcLeds    (fbarSbcLeds),          // if both switches DOWN (11)
    .portFFDO       (out255[7:0]),              // if switches DN UP (10)   
//    .debugreg   (debugReg[7:0]),        
    .sw         	(sw_IOBYTE[5:4]),
    .pll0_50MHz    (pll0_50MHz),
     //    .ram_cs,   
     .LEDoutData   (SBC_LEDs)   // INVERTED DATA OUT TO DRIVE LEDS!!!!!!
    );

/*********************************************************************************
*   IORQ Latch FF                                                                *
*********************************************************************************/
dff3     iorqlatch(
        .clk        (T80Clock),
        .pst_n      (1'b1),
        .clr_n      (!z80_n_iorq),     
        .din        (!z80_n_iorq),
        .q          (liorq)
        );
        
/********************************************************************************
*   pSYNC End latch FF                                                          *
********************************************************************************/        
         dff3     endpsync(
         .clk       (cpuClock),             //T80Clock here breaks the system//////////////////////(cpuClock),/////////////
//         .clk       (T80Clock),             //T80Clock here breaks the system//////////////////////(cpuClock),/////////////
        .pst_n      (!psyncstrt),
        .clr_n      (1'b1),     
        .din        (psyncstrt),
        .q          (psyncend)
        );

/********************************************************************************
*   Read Strobe latch FF    Output creates/latches pDBIN signal                 *
********************************************************************************/
dff3     readstrobe(
        .clk        (!pstval),
        .pst_n      (1'b1),
        .clr_n      (!busin),     
        .din        (!busin),
        .q          (pDBIN)
        );

/********************************************************************************
*   RESET Latch FF      Output drives Active LED and Control Out mux            *
********************************************************************************/        
dff3     resetLatch(
        .clk        (T80Clock),
        .pst_n      (n_reset),
        .clr_n      (1'b1),     
        .din        (1'b1),
        .q          (n_resetLatch)
        );

/********************************************************************************
*   S100 HOLD IN (busreq) Latch FF  Driven from S100 HOLD pin                   *
********************************************************************************/        
dff3     holdInLatch(
        .clk        (T80Clock),
        .pst_n      (1'b1),
        .clr_n      (1'b1),     
        .din        (s100_HOLD),
        .q          (z80_n_HoldIn)
        );

/********************************************************************************
*   S100 HLDAout (busak) Latch FF  Outputs HLDA to disable Address and Status   *
********************************************************************************/        
dff3     HLDAoutLatch(
        .clk        (!T80Clock),
        .pst_n      (1'b1),
        .clr_n      (!z80_n_busak),     
        .din        (!z80_n_busak),
        .q          (statDisable)
        );

/********************************************************************************
*        Control Disable Latch           										*
********************************************************************************/        
dff3     ctlDisableLatch(
        .clk        (T80Clock),
        .pst_n      (1'b1),
        .clr_n      (1'b1),     
        .din        (!(s100_HOLD | !statDisable)),
        .q          (ctlDisable)
        );

/********************************************************************************
*   Jump To ROM F0.  output (nop_n_cs) goes LOW upon reset, enabling NOPs to    *
*       CPU Data IN until rom address (F000) is reached, then it goes high      *
*       Supplanted by Jump On Reset logic 10/8/23.  TFOX
********************************************************************************        
dff3    jumptoRom(
        .clk        (n_reset),      // active low reset clocks din (low) to Q out
        .pst_n      (!rom_cs),      // preset sets nop_cs high when rom selected
        .clr_n      (1'b1),         // clear not used here    
        .din        (1'b0),         // always low
        .q          (nop_n_cs));    // low to enable NOPs until ROM reached
*/

/********************************************************************************
*   IO wait generation shift register (similar to hardware 74LS165)             *
*       Each zero in SerIn or ParIn causes one wait state output                *
*       ioWaitValue: FF = no wait states, 00 = 312ns (max wait)                 *
*       To add wait states, start with MOST significant bit first(ie: 7F=1 wait)*
*       3F = 2 wait states, etc.  DO NOT START WITH LSB first,as wait state     *
*           timing will be off.  Leave SerIn as a high for after shift          *
********************************************************************************/
parameter   romWaitvalue = 8'hfF;
parameter   ioWaitvalue = 8'hff;

// These timing are bassed on a 5MHz CPU clock (T80Clock)
// ffh = no wait states (iowait = 0ns, IO pDBIN = 344ns, other = 188ns)
// 7fh = one wait state = iowait = 156ns, IO pDBIN = 344 mostly or 375ns
// 3fh = two wait states = iowait = 344 or 375ns, IO_pDBIN = 531 or 563ns
// 1fh = three wait states = iowait = 563ns, IO pDBIN =n750ns
// 0fh = four wait stattes = iowait = 750ns or 781ns, IO pDBIN = 937 or 969ns
// 07h = five wait statess = iowait = 937 or 969ns, IO pDBIN = 1.125 or 1.156ns
// 03h = six wait states = iowait = 1.156 or 1.188ns, IO_pDBIN = 1.344 or 1.375ns
// 01h = seven wait states = iowait = 1.344 or 1.375ns, IO_pDBIN = 1.531 or1.563ns
// 00h = eight wait states (max) = iowait = 1.531 or 1.563ns, IO_pDBIN = 1.75ns

//  Changed clock divider to 3:2, resulting in T80 clock of 6.25MHz

// changed clock divider to 2:2, resulting in T80 clock of 
  
assign outPrn[0] = T80Clock;     //!s100_PHI;
assign outPrn[1] = z80_n_rd;
assign outPrn[2] = z80_n_mreq;
assign outPrn[3] = z80_n_m1 | z80_n_mreq;
assign outPrn[4] = busin;
assign outPrn[5] = psyncend;
assign outPrn[6] = s100_pSTVAL;
assign outPrn[7] = s100_sOUT;

ParShiftReg     iowait(
    .clk        (cpuClock),         //(T80Clock),                  // shift on Positive clock edge////////////////////////////////////////////////////////////////////////
    .SerIn      (1'b1),                      // shift register serial input
    .ParIn      (ioWaitvalue),               // Shift register parallel input
    .load       (!(iorq && s100_pSYNC)),     // Shift (high)/Load (low) input
    .qout       (ioWait)                     // single bit WAIT output
    );

/********************************************************************************
*   ROM wait generation shift register (similar to hardware 74LS165)            *
*       Each zero in SerIn or ParIn causes one wait state output                *
*       Timing is the same as IO wait states described above                    *
********************************************************************************/
ParShiftReg     romwait(
    .clk        (T80Clock),                     // shift on Positive clock edge
    .SerIn      (1'b1),                         // shift register serial input
    .ParIn      (romWaitvalue),                 // Shift register parallel input
    .load       (!(z80mreq & rom_cs && s100_pSYNC)),  // Shift (high)/Load (low) input
    .qout       (romWait)                       // single bit output
    );

 
/****************************************************************************
*   Eight bit shift register
****************************************************************************/
ShiftReg    usbTXdelay(
    .clk    (T80Clock),
    .clr    (!usbUARTbusy),
    .SerIn  (outusbTxData_cs),
    .qout   (usbTxDelay));

/****************************************************************************
*       USB UART serial data transfer                                       *
*           From opencores                                                  *
****************************************************************************/

/*
uart  usbuart(
    .clk                (pll0_50MHz),		// The master clock for this module 50MHz
    .rst                (!n_reset),     // Synchronous reset.
    .rx 				(usbRXData),		// UART Input - Incoming serial line
    .txd_out 			(usbTXData),	    // UART output - Outgoing serial line
    .transmit 			(usbTxDelay[2]), // Input to UART - Signal to transmit a byte = 1
    .tx_byte 		    (usbTxData),    // UART input - Byte to transmit
    .received 			(usbByteRcvd),   // UART output - Indicates a byte has been rcvd
    .rcvd_byte 		    (usbRxData),     // UART OUTPUT - Byte received
//    .is_receiving 		(usbBusyRcvg),   // UART output Low when receive line is idle.
    .is_transmitting 	(usbUARTbusy),   // UART output - Low when transmit line is idle.
    .recv_error         (usbUARTerror),  // output - Indicates error in receiving data.
	.data_ready         (usbDataReady),  // UART Output - has Rx data
	.data_read          (inusbRxData_cs), // UART input - read the received data
    .rcvr_overrun       (usbOverrun));     // Indicates when a Rx Data Overun condition occurs (cleared by data_read)
*/

// ****************************************************************************
// Added by Jeff Wilson to delay the transmit signal by two T80 Clocks
// ****************************************************************************
reg [1:0] tx_cs_delay;
always @(posedge T80Clock) 
begin
    if (!n_reset)
        tx_cs_delay <= 2'b00;
    else
    begin
        tx_cs_delay [0] <= outusbTxData_cs;
        tx_cs_delay [1] <= tx_cs_delay [0];
    end
end

assign seg7_dp = !(n_resetLatch & counter[20]); // Tick to show activity

reg [1:0]   uart_err;
reg [6:0]   seg7_reg;
localparam  UART_OK  = 2'b00;
localparam  UART_OVF = 2'b01;
localparam  UART_FF  = 2'b10;
localparam  UART_ERR = 2'b11;

assign  seg7 = seg7_reg;

always @(posedge pll0_50MHz)
begin
    if (!n_reset)
        uart_err        <= 2'b00;
    else
    begin
        // Errors persist until reset
        if (usbUARTOvrun | usbUARTerror)
            uart_err [0] <= 1'b1;
        if (usbUARTFFull | usbUARTerror)
            uart_err [1] <= 1'b1;
    end
end
 
always @(n_reset or seg7_dp or uart_err)
begin
    if (seg7_dp)
        seg7_reg = FPGA_VER;
    else
    begin
        if (uart_err == UART_OK)
            seg7_reg = 7'b1111111;     // No Error (blank)
        else if (uart_err == UART_OVF)
            seg7_reg = 7'b0100011;     // display is 'o' for overflow
        else if (uart_err == UART_FF)
            seg7_reg = 7'b0001110;     // display is 'F' for FIFO Full
        else if (uart_err == UART_ERR)
            seg7_reg = 7'b0110000;     // display is 'E' for Rx Error
    end
end

/****************************************************************************
*       USB UART serial data transfer                                       *
*           From opencores                                                  *
****************************************************************************/
uart  usbuart(
    .clk                (pll0_50MHz),		// The master clock for this module 50MHz
    .rst                (!n_reset),         // Synchronous reset.
    .rx 				(usbRXData),		// UART Input - Incoming serial line
    .txd_out			(usbTXData),	    // UART output - Outgoing serial line
    .transmit 			(tx_cs_delay[1]),    // Was (usbTxDelay[2]),    // Input to UART - Signal to transmit a byte = 1  JLW *****************************************
    .tx_byte 		    (usbTxData),        // UART input - Byte to transmit
    .received 			(usbByteRcvd),      // UART output - Indicates a byte has been rcvd
    .rcvd_byte 		    (usbRxData),        // UART OUTPUT - Byte received
    .is_receiving 		(usbBusyRcvg),      // UART output Low when receive line is idle.
    .is_transmitting 	(usbUARTbusy),      // UART output - Low when transmit line is idle.
    .recv_error         (usbUARTerror),     // output - Indicates error in receiving data.
	.data_ready         (usbDataReady),     // UART Output - has Rx data
	.data_read          (inusbRxData_cs),   // UART input - read the received data
    .rcvr_overrun       (usbUARTOvrun),     // UART Character Overrun
    .rcvr_fifo_full     (usbUARTFFull));    // UART FIFO Full

/************************************************************************************
*   PS/2 Keyboard support.  Added 2/22/23                                           *
************************************************************************************/    
ps2_keyboard_to_ascii   ps2_keyboard_to_ascii(
		.clk        (clockIn),          // 50MHz clock input to FPGA board                      /////////////////////////////////////////////
        .ps2_clk    (ps2Clk),           // PS/2 clock signal input from keyboard
        .ps2_data   (ps2Data),          // PS/2 data signal input from keyboard
        .ascii_new  (ps2CharAvail),     // output flag indicating new ASCII value
        .ascii_code  (ps2DataChar));    // PS2 code input from ps2_keyboard module

dff3     kybdLatch(                             // if dff, sends char 2-3 times
//        .clk        (pll0_250MHz),
//        .en         (ps2CharAvail), 
//        .rst_n      (n_reset & !ps2Data_cs),     
        .clk        (ps2CharAvail),
        .pst_n      (1'b1),
        .clr_n      (n_reset & !ps2Data_cs),     
        .din        (1'b1),
        .q          (ps2Stat));             // high when character is waiting
        

n_bitLatchClr2  #(.N(7))
    kybdDataLatch(
        .clk    (pll0_250MHz),
        .load   (ps2DataIn_cs),     // out = in if THIS is high
                                    // out = LATCHED in if load LOW
        .clr    (n_reset),          // reset latched data if board reset
        .inData (ps2DataChar[6:0]),
        .regOut (ps2kybdData[6:0]));
        
dff     kybdstatus(                 // was dff3
        .clk        (pll0_250MHz),
        .en         (ps2Status_cs), 
        .rst_n      (n_reset),     
        .din        (ps2Stat),
        .q          (ps2StatIn[0]));  // PS2 Status input to CPU Data In (Port 2), bit 0
        
/****************************************************************************************
*       VGA VIDEO SECTION FOLLOWS  ADDED Starting 2/22/23                               *
****************************************************************************************/

/****************************************************************************************
*       Divide 50MHz clock by 2 to create VGA 25MHz Clock                               *
****************************************************************************************/
/*
always @(posedge pll0_50MHz) begin
  if (!n_reset) begin
    MHz25 <= 1'b0;
  end
  else 
    MHz25 <= ~MHz25;
end
*/        
/****************************************************************************************
*   VGA font ROM        added 2/22/23                                                   *
****************************************************************************************/
rom  #(.ADDR_WIDTH(12),                    // set address width for chargen ROM
	.RAM_INIT_FILE("rom.fonthex"))      //("VGA_FONT.inithex"))
    font_rom (
    .address    (fontAdr),
	.clock      (MHz25),
	.data       (fontData));
    
/****************************************************************************************
*   Dual-Port RAM for character RAM     added 2/22/23                                   *
*       Port 1 connected to CPU     Port 2 connected to VGA module                      *
****************************************************************************************/

true_dual_port_ram
    #(.DATA_WIDTH(8),
	 .ADDR_WIDTH(12),
	 .RAM_INIT_FILE(""))      //("RAM_20H.inithex"))
    vgaRam(                         // Port 1 connected to CPU
    .addr1      (cpuAddress[11:0]),
	.din1       (cpuDataOut),
    .dout1      (ramVGAData[7:0]),      // RAM data OUT to CPU
	.we1        (vgaRamWriteData),     // write data enable
    .clka       (MHz25),
                                    // Port 2 connected to VGA module
    .addr2      (textAdr),
    .din2       (8'b0),                 // VGA module does Not WRITE to RAM
    .dout2      (textData),            // RAM data out to VGA module (read-only)
    .we2        (1'b0),                 // VGA Module, no write necesssary (read-only) 
    .clkb       (MHz25));

/****************************************************************************************
*       VGA 80x40 module.  There are at least three variants of this module, originally *
*       created by Javier Valcarce.  This version has been modified by                  *
*       to fix an issue.  This vesion does NOT include INPUT latches for the CPU to     *
*       read back Cursor registers/settings.                                            *
*       3/1/23, TFOX                                                                    *
****************************************************************************************/

vga80x40        vga(
    .reset      (!n_reset),     // NOTE:  this is a HIGH to reset
    .clk25MHz   (MHz25),   // pixel clock, and other signal timing
    .TEXT_A     (textAdr),      // Text Address OUT to dual-port RAM, address 2, 12 bits
    .TEXT_D     (textData),     // Text Data IN from dual-port RAM, port 2, 8 bits
	.FONT_A     (fontAdr),      // font Address Out to FONT ROM Address in, 12 bits
	.FONT_D     (fontData),     // Font Data IN from FPGA ROM, 8 bits
	.ocrx       (ocrxIn),       // HW cursor X IN from CPU       
    .ocry       (ocryIn),       // HW cursor Y IN from CPU
    .octl       (octlIn),       // HW cursor CONTROL IN from CPU
    .R          (vgaRed),       // VGA Red output to VGA connector
    .G          (vgaGreen),     // VGA Green output to VGA connector
    .B          (vgaBlue),      // VGA Blue output to VGA connector
    .hsync      (vgaHSync),     // VGA Horizontal Sync output
    .vsync      (vgaVSync));     // VGA Vertical Sync output
    
/********************************************************************************
*       THREE CURSOR (X, Y, CTL) OUTPUT LATCHES     (CPU-->vga80x40)            *
********************************************************************************/

n_bitLatchClr2      #(8)
    curXOut(
    .clk        (pll0_250MHz),
    .load       (vgaCXout_cs),           // out = in if high, latched if LOW
    .clr        (n_reset),
    .inData     (cpuDataOut[7:0]),
    .regOut     (ocrxIn[7:0]));

n_bitLatchClr2      #(8)
   curYOut(
    .clk        (pll0_250MHz),
    .load       (vgaCurYout_cs),           // out = in if high, latched if LOW
    .clr        (n_reset),
    .inData     (cpuDataOut[7:0]),
    .regOut     (ocryIn[7:0]));

n_bitLatchClr2      #(8)
   curCTLOut(
    .clk        (pll0_250MHz),
    .load       (vgaCurCtlout_cs),       // out = in if high, latched if LOW
    .clr        (n_reset),
    .inData     (cpuDataOut[7:0]),
    .regOut     (octlIn[7:0]));

/************************************************************************************
*   Parallel Printer Port Support Follows                                           *
************************************************************************************/
 
/********************************************************************************
*   Output Port C7   Parallel Printer DATA OUT Port                             *
********************************************************************************/
/*
n_bitReg    #(8)
    PrinterData(                        // Printer DATA OUTPUT LATCH
     .load      (!printer_cs),               // printer data latch select//////////////////
     .clock     (!printer_cs),     ///////////////////////////////////////////////////////
     .clr       (n_reset & !inPrnAck),
     .inData    (cpuDataOut),               // CPU data output to register
     .regOut    (outPrn));                  // register output to Printer FPGA output
*/        
dff     PtrStrobe(                     // Printer strobe output latch
        .clk    (pll0_250MHz),
        .en        (printerStrobe_cs), 
        .rst_n      (n_reset),     
        .din        (cpuDataOut[0]),        // CPU Data Out bit 0
        .q          (outPrnStrobe));        // Printer strobe FPGA output
        
dff     PtrAck(                        // Printer acknowledgement latch
        .clk        (pll0_250MHz),
        .en         (!printer_cs), 
        .rst_n      (inPrnAck),     
        .din        (1'b1),                 // set D input to high if n_reset
        .q          (prnAck));              // printer ack'ed

/************************************************************************************
*   BUZZER output   Port 00 OUT (any bits)                                          *
************************************************************************************/
dff     buzzerStart(                     // 
        .clk    (pll0_250MHz),
        .en        (buzzerOut_cs), 
        .rst_n      (!stopBuzzer),     
        .din        (1'b1),
        .q          (startBuzzer));
        
dff     buzzerRun(
        .clk    (pll0_250MHz),
        .en        (!counter[17]),
        .rst_n      (n_reset),          //(!stopBuzzer),     
        .din        (startBuzzer),
        .q          (runBuzzer));

dff   buzzerEnd(
        .clk    (pll0_250MHz),
        .en        (counter[17]),
        .rst_n      (n_reset),       //(!stopBuzzer),     
        .din        (runBuzzer),
        .q          (stopBuzzer));

        `define RTC
        
        `ifdef RTC
/************************************************************************************
*   Real Time Clock via SPI                                                         *
************************************************************************************/
wire    rtcSpiClkIn;
wire    RdWrFFClk;

assign  rtcSpiClkIn = !counter[4];
assign  RdWrFFClk = !counter[8];

spi_master    
    #(.slaves(4),
	 .d_width(8))
        rtcSPI(
        .clock      (rtcSpiClkIn),              // counter / 16 (2 MHz / 16 = 125 KHz)
        .reset_n    (n_reset),                  // asynchronous reset
        .enable     (RTCWrite | RTCRead),       // initiate transaction
        .cpol       (1'b1),                     // spi clock polarity
        .cpha       (1'b1),                     // spi clock phase
        .cont       (1'b0),                     // continuous mode command
        .clk_div    (32'b0),                    // system clock cycles per 1/2 period of sclk
        .addr       (32'b0),                    // address of slave
        .tx_data    (DataOutToRTC[7:0]),        // data to transmit
        .miso       (rtcSpiSO),                 // master in, slave out
        .sclk       (rtcSpiClk),                // spi clock
        .ss_n       (RTCslaveSelect),           // slave select
        .mosi       (rtcSpiSI),                 // master out, slave in
        .busy       (spiBusyFlag),              // busy / data ready signal OUT
        .rx_data    (DataInFromRTC[7:0])       // rtc data received
        );
      
n_bitLatchClr2      #(8)
   dataToRTC_7_0(                     // Port 68 OUT
    .clk        (pll0_250MHz),
    .load       (DataToRTC7_0_cs),
    .clr        (n_reset),
    .inData     (cpuDataOut[7:0]),
    .regOut     (DataOutToRTC[7:0])
    );


// n_bitLatchClr2      #(8)
//    dataToRTC_15_8(                    // Port 69 OUT
//     .clk        (pll0_250MHz),
//     .load       (DataToRTC15_8_cs),
//     .clr        (n_reset),
//     .inData     (cpuDataOut[7:0]),
//     .regOut     (DataOutToRTC[15:8])
//     );


n_bitLatchClr2      #(8)
   dataFromRTC_7_0(                   // Port 68 IN
    .clk        (pll0_250MHz),
    .load       (DataFmRTC_cs),        // inverting fixed no real data from RTC
    .clr        (n_reset),               // low to clear latch upon reset
    .inData     (DataInFromRTC[7:0]),    // Data byte back from RTC
    .regOut     (RTCDataToCPU[7:0])      // latch data to send to CPU INPUT
    );

     
n_bitLatchClr2      #(1)
   rtcSPI_select(                   // Port 6A OUT
    .clk        (pll0_250MHz),
    .load       (RTCSpi_cs),
     .clr       (n_reset),
     .inData   (cpuDataOut[0]),
     .regOut   (rtcSpiCS)
     );


/*     
dff    rtcSPI_select(                     // RTC SPI Chip Select latch
        .clk    (pll0_250MHz),
        .en     (RTCSpi_cs),             // Port 6A, bit D0 is strobe out
        .rst_n  (n_reset),              // clear the latch upon reset
        .din    (cpuDataOut[0]),        // CPU Data Out bit 0
        .q      (rtcSpiCS)              // RTC SPI CS FPGA output
        ); 
*/

dff    rtcspiBUSY(                   // RTC SPI BUSY IN latch
        .clk    (pll0_250MHz),
        .en     (RTCSpiBusy_cs),
        .rst_n  (n_reset),
        .din    (spiBusyFlag),           // SPI Busy in from SPI master
        .q      (LatchedspiRTCBusyFlag) // SPI Latched Busy Flag
        );

assign  RTCSpiBusyFlag[0] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[1] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[2] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[3] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[4] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[5] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[6] = LatchedspiRTCBusyFlag;
assign  RTCSpiBusyFlag[7] = LatchedspiRTCBusyFlag;

//assign RTCWriteClr = (n_reset  & !spiBusyFlag);

dff    RTCSpiWrite1(
        .clk    (pll0_250MHz),
        .en     (RTCSpiWrite1_cs),
        .rst_n  (n_reset  & !spiBusyFlag),
        .din    (1'b1),
        .q      (RTCWriteOut1)
        );

dff   RTCSpiWrite2(
        .clk    (pll0_250MHz),
        .en     (RdWrFFClk),
        .rst_n  (n_reset  & !spiBusyFlag),
        .din    (RTCWriteOut1),
        .q      (RTCWrite)
        );

dff   RTCSpiRead1(
        .clk    (pll0_250MHz),
        .en     (RTCSpiReadFF1_cs),
        .rst_n  (n_reset  & !spiBusyFlag),
        .din     (1'b1),
        .q       (RTCReadOut1)
        );

dff   RTCSpiRead2(
        .clk     (pll0_250MHz),
        .en      (RdWrFFClk),
        .rst_n   (n_reset  & !spiBusyFlag),
        .din     (RTCReadOut1),
        .q       (RTCRead)
        );
`endif

`define  VECTORINTERRUPTS


`ifdef VECTORINTERRUPTS
/************************************************************************
*   Vector Interrupts                                                   *
************************************************************************/
wire    [7:0]   intsInLatch;    // Latched INPUT Interrupt Vectors
wire    [7:0]   encoderIn;      // rewired for Interrupt encoder input
wire    [7:0]   intsOutLatch;   // Latched priority encoder OUTPUT
reg             intVec_cs;
//    wire    [7:0]   intsIn;         // Interrupts in to latch 
//    wire    [7:0]   intsLatched;    // Latched interrupts to encoder
//    wire    [7:0]   intsToCpu;      // encoded interrupt vector to CPU
//    wire            intVectToCPU_cs; // Select signal to cpuDImux
//    wire            intToCPU_n;     //
//    wire            z80_n_boardInt; // interrupt signal out to Z80 CPU
//    wire            intsGs;         // an interrupt vector has been created
//    reg             z80_n_Enable;  // enable the Z80 int pin or not

//assign  intVectToCPU_cs = (s100_sINTA & !inEnableINTA);    // Enables interrupt vector write to CPU Data IN
////    INT Level  =  sOUT_INTA  AND NOT enable jumper (low->high) NOTE: s100_INTA HAD DELAYS IN JOHNS BDF  //////////////   BOTTOM OR GATE  ACTIVE LOW
////     =H for vector  ___|--|___  L=Enable, H=disable 
assign intVectToCPU_cs = intVec_cs;

always @(posedge T80Clock) begin                      //cpuClock) begin//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    intVec_cs <= (s100_sINTA & !inEnableINTA);
    end

assign  z80_n_boardInt = (IntEncGs | inEnableINTA);     // & s100_n_INT;        ////////////////////////////////////////////////////////////////////
////  Actual INT->Z80(low) = (Gs(low) or enable jmpr (low)) & bus INTs 



////////////    First, lay out the various interrupt vectors
assign  intsIn[0] = 1'b1;           // not implemented on Z80 FPGA
assign  intsIn[1] = 1'b1;           // not implemented on Z80 FPGA
assign  intsIn[2] = rtc_n_INT;      // RTC Interrupt
assign  intsIn[3] = in_n_INTD;      // INT_D-
assign  intsIn[4] = in_n_INTC;      // INT_C-
assign  intsIn[5] = in_n_INTB;      // INT_B-
assign  intsIn[6] = in_n_INTA;      // INT_A-
assign  intsIn[7] = 1'b1;           // not implemented on Z80 FPGA

//////////  Interrupt Encoder Gs goes LOW if any valid interrupt goes low
//assign IntEncGs = (rtc_n_INT & in_n_INTD & in_n_INTC & in_n_INTB & in_n_INTA);

n_bitLatchClr2      #(8)        // Interrupt signals input latch
      intrptlatch(
     .load      (n_inta),           // high-to-low transition latches
     .clk       (pll0_250MHz),      // current interrupt input status
     .clr       (n_reset),
     .inData    (intsIn),           // interrupt signals inputs
     .regOut    (intsInLatch)       // latched INPUT interrupt signals
     );

////////////    Now rewire and invert interrupt vectors
assign  encoderIn[0] = ~intsInLatch[7];
assign  encoderIn[7:1] = ~intsInLatch[6:0];
     
priEncoder priorityEncoder(
     .pll0_250MHz   (pll0_250MHz),
     .aIn           (encoderIn),            // rewired interrupts to encoder
    .encOut         (intsOutLatch[7:0]),    // rewired & prioritized interrupts
    .intsGs         (IntEncGs)
    );

n_bitLatchClr2     #(8)         // encoded/prioritized Interrupt output latch
      intsFlagToCPUIn(          // to send to Z80 CPU Data Input Mux
     .load      (intVectToCPU_cs),       // high-to-low transition
     .clk       (pll0_250MHz),          //(n_inta),
     .clr       (n_reset),
     .inData    (intsOutLatch[7:0]),    // interrupt outputs to latch
     .regOut    (intsToCpu[7:0])        // Latched Interrupt (RSTxx) to CPU INPUT
     );
    
`endif

`define     SDCARD

`ifdef  SDCARD     
/************************************************************************
*   SD Card via Eight-Bit SPI Interface                                 *        
************************************************************************/        

assign sd_n_Acs = SDCardSelect[0];
assign sd_n_Bcs = SDCardSelect[1];

assign SD_status[7] = SDSpiBusyFlag;
assign SD_status[6] = 1'b0;
assign SD_status[5] = 1'b0;
assign SD_status[4] = 1'b0;
assign SD_status[3] = 1'b0;
assign SD_status[2] = 1'b0;
assign SD_status[1] = sd_n_Acs;
assign SD_status[0] = sd_n_Bcs;

assign SDSpiClr = (n_reset  & !SDSpiBusyFlag);

spi_master    
    #(.slaves(4),
      .d_width(8))
        sdSPI(
        .clock      (pll0_50MHz),        // system clock
        .reset_n    (n_reset),           // asynchronous reset
        .enable     (SDWrite | SDRead),  // initiate transaction
        .cpol       (1'b0),              // spi clock polarity
        .cpha       (1'b0),              // spi clock phase
        .cont       (1'b0),              // continuous mode command
        .clk_div    (32'd5),             // system clock cycles per 1/2 period of sclk
        .addr       (32'b0),             // address of slave
        .tx_data    (DataToSD[7:0]),     // data to transmit from Latch and CPU
        .miso       (sdMISO),            // master in, slave out
        .sclk       (sdCardClk),         // spi clock
        .ss_n       (SDSlaveSelect),     // slave select
        .mosi       (sdMOSI),            // master out, slave in
        .busy       (SDSpiBusyFlag),     // busy / data ready signal OUT
        .rx_data    (DataFmSD[7:0])      // rtc data received
        );

n_bitLatchClr2      #(8)
  LatchDataToSDcard(                     // Data latch from CPU to SD (Port 6C OUT)
    .clk        (pll0_250MHz),
    .load       (DataToSD_cs),
    .clr        (n_reset),
    .inData     (cpuDataOut[7:0]),
    .regOut     (DataToSD[7:0])
    );

n_bitLatchClr2      #(8)
  dataFromSDcard(                       // Data latch from SD SPI to CPU (Port 6C IN)
    .clk        (pll0_250MHz),
    .load       (DataFmSD_cs),          // inverting fixed no real data from RTC
    .clr        (n_reset),              // low to clear latch upon reset
    .inData     (DataFmSD[7:0]),        // Data byte back from RTC
    .regOut     (SDdataToCPU[7:0])      // latch data to send to CPU INPUT
    );
    
n_bitLatchClr2      #(8)
  SDStatus(                             // SD Status Data latch, status to CPU (Port 6E IN)
    .clk        (pll0_250MHz),
    .load       (SD_status_cs),         // inverting fixed no real data from RTC
    .clr        (n_reset),              // low to clear latch upon reset
    .inData     (SD_status[7:0]),       // Data byte back from RTC
    .regOut     (SD_statusToCPU[7:0])   // latch data to send to CPU INPUT
    );
     
n_bitLatchClr3      #(2,'b11)
  SDSPI_select(                         // SD Card A/B select latch, Port 6E OUT,bits 0 & 1
    .clk        (pll0_250MHz),
    .load       (SD_Card_select_cs),
    .clr        (n_reset),
    .inData     (cpuDataOut[1:0]),
    .regOut     (SDCardSelect[1:0])
    );

dff
  SDSpiWrite(
    .clk        (pll0_250MHz),
    .en         (SDWrite_cs & !SDWait),
    .rst_n      (SDSpiClr),
    .din        (1'b1),
    .q          (SDWrite)
    );

dff
  SDSpiRead(
    .clk        (pll0_250MHz),
    .en         (SDRead_cs & !SDWait),
    .rst_n      (SDSpiClr),
    .din        (1'b1),
    .q          (SDRead)
    );

dff
  SDSpiWait(
    .clk        (pll0_250MHz),
    .en         (SDSpiBusyFlag),
    .rst_n      (SDWrite_cs | SDRead_cs),
    .din        (1'b1),
    .q          (SDWait)
    );

`endif

/****************************************************************************
*   Memory Management Unit (MMU)   Basic design from Zeta V2 board          *
*       Uses four 8-bit registers to select four 16k blocks in 64k CP/M     *
*           Address A1 & A0 selects which register is currently used        *
*       Port 78h holds Axx-Ayy addresses for Z80 CP/M memory 0000-3FFF      *
*       Port 79h register holds virtual addresses for Z80 memory 4000-7FFF  *
*       Port 7Ah register holds virtual addresses for Z80 memory 8000-BFFF  *
*       Port 7Bh register holds virtual addresses for Z80 memory C000-FFFF  *
****************************************************************************/

mmu     mmu0(
    .clk            (T80Clock),
    .n_reset        (n_reset),          // mmu enable/disable FF reset
    .mmuDataIn      (cpuDataOut[7:0]),  // eight bit CPU Data OUT to mmu
    .mmuDataOut     (mmu2cpuDataIn[7:0]),
    .address        (cpuAddress[15:0]), // Z80 16-bit address bits for I/O ports and memory paging
    .MMUPageEnWrEn  (MMUPageEnWrEn),    // CPU IN/OUT READ
    .MMURegFileWrEn (MMURegFileWrEn),   // CPU IN/OUT WRITE
    .MMURegFileRdEn (MMURegFileRdEn),
    .mmuAdrOut      (ma14_21[7:0])      // eight bank addreses MA14-MA21
    );
        
endmodule   
    
