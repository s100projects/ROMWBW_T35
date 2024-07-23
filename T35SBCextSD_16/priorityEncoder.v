/****************************************************************************
*   FILE:   priorityEncoder.v     TFOX    March 20, 2023   Ver. 0.1         *
*       encOut outputs are set to active HIGH to match interrupt value      *
*       April 10, 2023, Ver.2  Changed logic from case statement to else if *    
*       intsGs should be LOW if any interupts created, otherwise HIGH       *
****************************************************************************/

module priEncoder(
    input               pll0_250MHz,
    input       [7:0]   aIn,
    output reg  [7:0]   encOut,
    output reg          intsGs
    );
    
    always @(posedge pll0_250MHz) begin
    if(aIn[7])      begin   encOut <= 8'h38; intsGs <= 1'b0; end // 7, INT_A
    else if(aIn[6]) begin   encOut <= 8'h30; intsGs <= 1'b0; end // 6, INT_B
    else if(aIn[5]) begin   encOut <= 8'h28; intsGs <= 1'b0; end // 5, INT_C
    else if(aIn[4]) begin   encOut <= 8'h20; intsGs <= 1'b0; end // 4, INT_D
    else if(aIn[3]) begin   encOut <= 8'h18; intsGs <= 1'b0; end // 3, INT_RTC
    else if(aIn[2]) begin   encOut <= 8'h10; intsGs <= 1'b1; end // 2, not used
    else if(aIn[1]) begin   encOut <= 8'h08; intsGs <= 1'b1; end // 1, not used
    else if(aIn[0]) begin   encOut <= 8'h00; intsGs <= 1'b1; end // 0, not used
    else begin encOut <= 8'h00; intsGs <= 1'b1; end // 0, not used
    end
endmodule
