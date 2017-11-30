module lfsr_5b(input logic clk,
					input logic reset,
					output logic [4:0] q);

logic feedback;					
					
always_ff @(posedge clk, posedge reset)
	if(reset)	q <= 5'h1F;  // load with ones
	else 			q <= {feedback, q[4:1]};

assign feedback = q[0]^q[2];

endmodule
					