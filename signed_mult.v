module signed_mult (clk, a, b, out);
	input      signed  clk;
	output 	signed  [15:0]	out; 
	input 	signed	[15:0] 	a; 
	input 	signed	[15:0] 	b; 
	wire 	signed	[31:0]	mult_out;
	// assign mult_out = a * b;
	// select bits for 2.14 fixed point 
	// 3.13
	// assign out = {mult_out[31], mult_out[28:14]};
	// assign out = {mult_out[31], mult_out[27:13]};
	assign out = a * b;
endmodule

