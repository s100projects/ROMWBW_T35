module RegisterFile #(
  parameter DataWidth  = 8,
  parameter NumRegs    = 16,
  parameter IndexWidth = $clog2(NumRegs)
) (
  input                       clk,
  input                       writeEn,
  input      [IndexWidth-1:0] writeAddr,
  input      [ DataWidth-1:0] writeData,
  input      [IndexWidth-1:0] readAddr,
  output     [ DataWidth-1:0] readData
  );

  reg [7:0] regs [(NumRegs-1):0];

  always @(posedge clk) begin
    if (writeEn) begin
      regs[writeAddr] <= writeData;
    end
  end

  assign readData = regs[readAddr];

endmodule