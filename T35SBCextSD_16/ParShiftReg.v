/************************************************************************
*   FILE:   ParShiftReg.v      Ver 0.3      Oct. 14, 2023               *
*                                                                       *
*   This function is used to create wait delays for I/O or ROM access   *
*       It is similar to a 74165 TTL chip or BDF emulation.             *
*       load is used to load the parallel input into the temp register  *
*           load is active LOW to load, high to shift                   *
*       SerIn is the LAST bit sent out.  It also sets the post-delay    *
*           logic level, so for now, keep SerIn active HIGH             *
*       ParIn is the first eight bits used to create a wait delay output*
*           Input: FF = no wait states, 00 = 312ns (max wait)           *
*           To add wait states, start with MOST significant bit first   *
*           (ie: 7F=1 wait) 3F = 2 wait states, etc.                    *
*           DO NOT START WITH LSB first,wait state timing will be wrong *
*   TFOX, N4TLF January 20, 2023   You are free to use it               *
*       however you like.  No warranty expressed or implied             *
*       Oct 14, 2023: modified to fix timing problem with load
************************************************************************/

module ParShiftReg
    (
    input           clk,    // shift on Positive clock edge of cpuClock
    input           SerIn,  // shift register serial input 
    input   [7:0]   ParIn,  // Shift register parallel input
    input           load,   // Shift (low)/Load (high) input
    output          qout);  // single bit ready/wait output
   
reg [8:0]   temp;

always @(posedge clk or negedge load)  begin
        if(load == 1'b0) begin              // if loading the shift register
            temp[8:1] <= ~ParIn[7:0];       // set the parallel data
            temp[0] <= !SerIn;              // including the serial input
            end                             //   which is the non-shift output
        else begin
            temp = temp << 1;               // otherwise shift the register               
        end
    end
    
assign qout = !temp[8];

endmodule

