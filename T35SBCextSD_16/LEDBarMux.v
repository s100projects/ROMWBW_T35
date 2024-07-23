/********************************************************************
*   FILE:   LEDBarMux.v      Ver 0.5         Nov. 28, 2022          *
*       NOTE! NOTE!  OUTPUT IS INVERTED FOR DRIVING THE LEDs        *
                        since the LEDs common are connected to +V   *
*       SWITCHES 5 & 4:     SW = 00 (UP UP) = CPU Data OUTPUT       *
*                           SW = 01 (UP DN) = CPU Data INPUT        * 
*                           SW = 10 (DN UP) = PORT FF OUT           *
*                           SW = 11 (DN DN) = fbarSbcLeds           *
*                                                                   *
*   TFOX, N4TLF January 20, 2023   You are free to use it           *
*       however you like.  No warranty expressed or implied         *
********************************************************************/

module LedBarMux
    (
    input [7:0] cpuDO,
    input [7:0] cpuDI,
    input [7:0] portFFDO,
//    input [7:0] debugreg,
    input [7:0] fbarSbcLeds,
    input [1:0]	sw,
//    input   ram_cs,
    input       pll0_50MHz,
    output reg [7:0] LEDoutData
    );

always @(posedge pll0_50MHz) begin
    case(sw)
        2'b00:      LEDoutData = ~cpuDO;
        2'b01:      LEDoutData = ~cpuDI;
        2'b10:      LEDoutData = ~portFFDO;
        default:    LEDoutData = ~fbarSbcLeds;
    endcase
    end
     
endmodule
