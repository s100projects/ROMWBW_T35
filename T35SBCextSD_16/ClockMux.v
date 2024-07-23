/********************************************************************************
*       ClockMux.v.     TFOX        Updated version 0.6     Jan 20, 2023        *
*           TFOX, N4TLF January 31, 2023   You are free to use it               *
*           however you like.  No warranty expressed or implied                 *
*   Clock speed INPUT Multiplexer.  Uses a much higher clock to drive posedge   *
*       selection.  Switches 7 & 6 select cpu clock speed (which is then        *
*       divided by six in microcontroller.vhdl).                                *
*       7 & 6 = 11 (UP UP)(OFF OFF) = 25MHz, actual CPU clock is 5.0MHz         *
*       7 & 6 = 10 (UP DN)(OFF ON)  = 31kHz, actual CPU clock is 6.25kHz        *
*       7 & 6 = 01 (DN UP)(ON OFF)  = 2MHz, actual CPU clock is 400kHz          *
*       7 & 6 = 00 (DN DN)(ON ON)   = 244Hz, actual CPU clock is 48.8Hz         * 
*                                                                               *
*   April 7, 2023, Version 0.6 Changed case statement to match revised switches *
********************************************************************************/

module  ClockMux(
    input           MHz2,
    input           MHz50,
    input           KHz31,
    input           Hz250,
    input           pll0_250MHz,
    input   [1:0]   sw, 
    output  reg    cpuclk
    );

always @(posedge pll0_250MHz) begin
    case(sw)
        2'b11:      cpuclk = MHz50;
        2'b10:      cpuclk = KHz31;
        2'b01:      cpuclk = MHz2;
        default:    cpuclk = Hz250;   
     endcase   

     end
    
endmodule
    
