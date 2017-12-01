module clock_domain_1b 
						  #(	parameter width = 2	)
							(	input logic fast_clk,
								input logic slow_clk,
								input logic [width-1:0] async_data_in,
								output reg q1, q2, q3, q4, q5, 
								output logic en_5, reset_1,
								output logic [width-1:0] sync_data_out);

// note: you can simulate a module that depends on parameters, just change 
// 		variable width in the .vwf file

assign reset_1 = ~slow_clk & q3;
assign en_5 = q3 & ~q4;

always_ff @(posedge slow_clk, posedge reset_1)
	if(reset_1) q1 <= 1'b0;
	else q1 <= 1'b1;	

always_ff @(posedge fast_clk)
begin
	q2 <= q1;
	q3 <= q2;
	q4 <= q3;
end

always_ff @(posedge fast_clk)
begin
	if(en_5) sync_data_out <= async_data_in;
end



endmodule