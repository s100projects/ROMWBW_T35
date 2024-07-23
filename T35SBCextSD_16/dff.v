
module dff (
    input   clk,        // 250MHz clock
    input   en,         // 
    input   rst_n,
    input   din,
    output reg  q);

always @(posedge clk or negedge rst_n) 
begin
 if(!rst_n)
  q <= 1'b0; 
 else if(en)
  q <= din; 
end 
endmodule 
