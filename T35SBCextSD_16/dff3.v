/********************************************************************************
*   D style flip flop.
*       inlcludes SET and RESET async inputs                                    *
********************************************************************************/
module  dff3 (
    input   clk,
    input   pst_n,
    input   clr_n,
    input   din,
    output  q);
//    ,output n_q);
    
    reg     qout;
    
always @(posedge(clk) or negedge(pst_n) or negedge(clr_n))
    if(!pst_n) begin
        qout <= 1'b1;
        end
    else if(!clr_n) begin
        qout <= 1'b0;
        end
    else begin //if(clk == 1'b1)
        qout <= din;
        end
 /*   
     if(pst_n == 1'b0)
        qout <= 1'b1;
    else if(clr_n == 1'b0)
        qout <= 1'b0;
    else //if(clk == 1'b1)
        qout <= din;
*/   
assign q = qout;
//assign n_q = !qout;

endmodule 

