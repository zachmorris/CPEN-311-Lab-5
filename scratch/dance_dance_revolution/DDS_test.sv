module DDS_test(	input logic clk,
						input logic lfsr_clk, 
						output logic [4:0] lfsr_data,
						input logic lfsr_reset,
						output logic async_lfsr_0,
						output logic sync_lfsr_0,

						//DDS
						output wire [11:0] dds_sin_out,
						output wire [11:0] dds_cos_out,
						output wire [11:0] dds_sq_out,
						output wire [11:0] dds_saw_out,

						// modulation DDS
						output wire [11:0] dds_sin_ASK,
						output wire [11:0] dds_sin_BPSK);

wire [11:0] dds_sin_comp;

/// LFSR code
// don't need a clk
// in lfsr_5b, can we do <output logic [4:0] q = 5'h1F>
lfsr_5b lfsr_inst(	.clk(lfsr_clk),
							.reset(lfsr_reset),
							.q(lfsr_data));
   
/// DDS code
waveform_gen waveform_gen_inst(	.clk(clk),
											.reset(1'b1),
											.en(1'b1),
											.phase_inc(32'h28F5C29),
											.sin_out(dds_sin_out),
											.cos_out(dds_cos_out),
											.squ_out(dds_sq_out),
											.saw_out(dds_saw_out));
											
/// Modulate DDS with LFSR 
// Synchronize the LFSR bitstream
clock_domain_sync #(1) LFSR_bit_sync(
									.fast_clk(clk),
									.slow_clk(lfsr_clk),
									.async_data_in(async_lfsr_0),
									.sync_data_out(sync_lfsr_0));

// I think this is synced to the 1Hz clock, need to cross clock domains									
assign async_lfsr_0 = lfsr_data[0];

// ASK modulation 
assign dds_sin_ASK = sync_lfsr_0 ? dds_sin_out : 12'b0;

// BPSK modulation
assign dds_sin_BPSK = sync_lfsr_0 ? dds_sin_out : dds_sin_comp;
assign dds_sin_comp = ~dds_sin_out + 12'b1;



endmodule