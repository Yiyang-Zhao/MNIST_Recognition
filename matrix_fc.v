// fully connected layer
module matrix_fc(
	input clk,
	input reset,
	input start,
	output reg done,
	input wire [11:0] src1_start_address, //vector
	input wire [11:0] src2_start_address, //matrix
	input wire [11:0] src3_start_address,
	output reg [11:0] sram_address_1,
	input wire [15:0] sram_data_1,

	output reg [11:0] sram_address_2,
	input wire [15:0] sram_data_2,
	
	output reg [11:0] sram_address_3,
	input wire [15:0] sram_data_3,
	
	output reg [3:0] final_result
);
	 reg [3:0] state = 4'd0;
	 reg [31:0] sum_out, weight;
	 wire [15:0] multiplex_out;
    reg [3:0] count;


	 signed_mult_cnm mult1(.clk(clk), .out(multiplex_out), .a(sram_data_1), .b(sram_data_2));

	 
	 always @(posedge clk) begin
	 
		if(reset) begin
        state <= 4'd0;

        done <= 1'd0;
		end
		
		if(state == 4'd0 && start == 1'd1) begin
			sram_address_1 <= src1_start_address;
			sram_address_2 <= src2_start_address;
			sram_address_3 <= src3_start_address;
			//dest_address <= dest_start_address;
			count <= 4'd0;
			final_result <=4'd0;
			state <= 4'd1;
			weight <= 32'd0;
		end

		if(state == 4'd1) begin
			sum_out <= 32'd0;
			state <= 4'd2;
		end

		if(state == 4'd2 && count < 5)begin
			sram_address_1 <= sram_address_1 + 12'd1;
			sram_address_2 <= sram_address_2 + 12'd1;
			//signed_mult mult2(.out(multiplex_out), .a(sram_data_1), .b(sram_data_2));
			sum_out <= multiplex_out + sum_out;
			
			state <= (sram_address_1 == 12'd506)? 4'd3:4'd2;
		end
		
		if(state == 4'd2 && count >= 5)begin
			sram_address_1 <= sram_address_1 + 12'd1;
			sram_address_3 <= sram_address_3 + 12'd1;
			//signed_mult mult2(.out(multiplex_out), .a(sram_data_1), .b(sram_data_2));
			sum_out <= multiplex_out + sum_out;
			
			state <= (sram_address_1 == 12'd506)? 4'd3:4'd2;
		end
		
		if (state == 4'd3)begin
			weight <= (sum_out >= weight) ? sum_out : weight;
			final_result <= (sum_out >= weight) ? count : final_result;
			if (count == 4'd9) begin
				state <= 4'd4;
			end
			else begin
				count <= count + 4'd1;
				state <= 4'd1;
			end
			
			//dest_write_en <= 1'd0;
			sram_address_1 <= src1_start_address;
         //state <= (sram_address_2 == 12'd5096) ? 4'd4:4'd1;
		end
		if (state == 4'd4)begin
			 done <= 1'd1;
		end	 
	end
endmodule

module signed_mult_cnm (clk, a, b, out);
	input      signed  clk;
	output 	signed  [15:0]	out; 
	input 	signed	[15:0] 	a; 
	input 	signed	[15:0] 	b; 
	wire 	signed	[31:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 2.14 fixed point 
	// 3.13
	// assign out = {mult_out[31], mult_out[28:14]};
	assign out = mult_out;
	//assign out = a * b;
endmodule
