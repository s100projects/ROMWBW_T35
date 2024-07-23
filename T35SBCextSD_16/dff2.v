/********************************************************************************
*   D style flip flop.  Only includes an active-low async CLR input             *
********************************************************************************/
module  dff2 (
    input   clk,
    input   clr_n,
    input   din,
    output  q);

    reg     qout;
    
always @(posedge clk or posedge clr_n)
    begin
        if(!clr_n) begin
                qout <= 1'b0;
            end
        else begin
                qout <= din;
            end
    end
assign q = qout;

endmodule 

