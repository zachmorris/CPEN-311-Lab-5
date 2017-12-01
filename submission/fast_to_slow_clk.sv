module fast_to_slow_clk 
						  #(	parameter width = 12	)
							(	input logic fast_clk,
								input logic slow_clk,
								input logic [width-1:0] async_data_in,
								output logic [width-1:0] sync_data_out);
								
reg [width-1:0] q1, q2;
reg q3, flop_en;

always_ff@(posedge fast_clk)
	q1 <= async_data_in;

always_ff@(posedge fast_clk)
	if(flop_en)
		q2 <= q1;
		
always_ff@(posedge slow_clk)
	sync_data_out <= q2;
	
always_ff@(posedge ~fast_clk)
	q3 <= slow_clk;
	
always_ff@(posedge ~fast_clk)
	flop_en <= q3;
	
endmodule 