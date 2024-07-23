/////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2013-2018 Efinix Inc. All rights reserved.
//
// Single Port ROM
//
// *******************************
// Revisions:
// 0.0 Initial rev
// 1.0 Finalized RTL macro
// *******************************
/****************************************************************************
*   1/31/23:    This version used for TFOX S100 boards                      *
****************************************************************************/

module rom 
#(
	parameter DATA_WIDTH = 8,
	parameter ADDR_WIDTH = 14,
	parameter OUTPUT_REG = "FALSE",
	parameter RAM_INIT_FILE = "fpgamon.inithex")
(
	input [ADDR_WIDTH-1:0] address,
	input clock,
	output [DATA_WIDTH-1:0] data
);
localparam MEMORY_DEPTH = 2**ADDR_WIDTH;

reg	[DATA_WIDTH-1:0]r_rdata_1P;
reg	[DATA_WIDTH-1:0]r_rdata_2P;

reg [DATA_WIDTH-1:0] rom[MEMORY_DEPTH-1:0];

initial	begin
	$readmemh(RAM_INIT_FILE, rom);
	end

always@(posedge clock)
begin
	r_rdata_1P <= rom[address];
	r_rdata_2P <= r_rdata_1P;
end

generate
	if (OUTPUT_REG == "TRUE")
		assign	data = r_rdata_2P;
	else
		assign	data = r_rdata_1P;
endgenerate

endmodule