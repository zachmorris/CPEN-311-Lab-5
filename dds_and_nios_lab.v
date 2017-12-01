`default_nettype none
`include "./scan_events.h"
//`define ENABLE_AUDIO_DEMO
module dds_and_nios_lab(
    //////////// CLOCK //////////
    CLOCK_50,
    CLOCK_27,
	
    //////////// LED //////////
    LEDG,
    LEDR,

    //////////// KEY //////////
    KEY,

    //////////// SW //////////
    SW,

    //////////// SEG7 //////////
    HEX0,
    HEX1,
    HEX2,
    HEX3,
    HEX4,
    HEX5,
    HEX6,
    HEX7,

    //////////// RS232 //////////
    UART_RXD,
    UART_TXD,

    //////////// Audio //////////
    AUD_ADCDAT,
    AUD_ADCLRCK,
    AUD_BCLK,
    AUD_DACDAT,
    AUD_DACLRCK,
    AUD_XCK,

    //////////// I2C for Audio  //////////
    FPGA_I2C_SCLK,
    FPGA_I2C_SDAT,

    //////////// Flash //////////
    FL_ADDR,
    FL_CE_N,
    FL_DQ,
    FL_OE_N,
    FL_RST_N,
    FL_WE_N,
    
    
    //////// PS2 //////////
    PS2_CLK,
    PS2_DAT,
    
    //////// SDRAM //////////
    DRAM_ADDR,
    DRAM_BA_0,
    DRAM_BA_1,
    DRAM_CAS_N,
    DRAM_CKE,
    DRAM_CLK,
    DRAM_CS_N,
    DRAM_DQ,
    DRAM_LDQM,
    DRAM_UDQM,
    DRAM_RAS_N,
    DRAM_WE_N,
    
    
    //////// SDCARD //////////
    SD_CLK,
    SD_CMD,
    SD_DAT,
    SD_DAT3,
    
    
    //////// GPIO //////////
    GPIO_0,
    GPIO_1,
    
                //////////// SRAM //////////
    SRAM_ADDR,
    SRAM_CE_N,
    SRAM_DQ,
    SRAM_LB_N,
    SRAM_OE_N,
    SRAM_UB_N,
    SRAM_WE_N,

//////////// LCD //////////
    LCD_DATA,
    LCD_EN,
    LCD_ON,
    LCD_RS,
    LCD_RW,
	 
	 ////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B  						//	VGA Blue[9:0]

);

//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input                       CLOCK_50;
input                       CLOCK_27;

//////////// LED //////////
output           [8:0]      LEDG;
output          [17:0]      LEDR;

//////////// KEY //////////
input            [3:0]      KEY;

//////////// SW //////////
input           [17:0]      SW;

//////////// SEG7 //////////
output           [6:0]      HEX0;
output           [6:0]      HEX1;
output           [6:0]      HEX2;
output           [6:0]      HEX3;
output           [6:0]      HEX4;
output           [6:0]      HEX5;
output           [6:0]      HEX6;
output           [6:0]      HEX7;

//////////// LCD //////////
inout            [7:0]      LCD_DATA;
output                      LCD_EN;
output                      LCD_ON;
output                      LCD_RS;
output                      LCD_RW;

//////////// RS232 //////////
input                       UART_RXD;
output                      UART_TXD;

//////////// Audio //////////
input                       AUD_ADCDAT;
inout                       AUD_ADCLRCK;
inout                       AUD_BCLK;
output                      AUD_DACDAT;
inout                       AUD_DACLRCK;
output                      AUD_XCK;

//////////// I2C for Audio  //////////
output                      FPGA_I2C_SCLK;
inout                       FPGA_I2C_SDAT;

//////////// Flash //////////
output          [21:0]      FL_ADDR;
output                      FL_CE_N;
inout            [7:0]      FL_DQ;
output                      FL_OE_N;
output                      FL_RST_N;
output                      FL_WE_N;



//////////// PS2 //////////
inout                       PS2_CLK;
inout                       PS2_DAT;

//////////// SDCARD //////////
output                      SD_CLK;
inout                       SD_CMD;
inout                       SD_DAT;
input                       SD_DAT3;

//////////// GPIO //////////
inout           [35:0]      GPIO_0;
inout           [35:0]      GPIO_1;


//////////// SRAM //////////
output          [17:0]      SRAM_ADDR;
output                      SRAM_CE_N;
inout           [15:0]      SRAM_DQ;
output                      SRAM_LB_N;
output                      SRAM_OE_N;
output                      SRAM_UB_N;
output                      SRAM_WE_N;
                
                
//////////// SDRAM //////////
output          [11:0]      DRAM_ADDR;
output                      DRAM_BA_0;
output                      DRAM_BA_1;
output                      DRAM_CAS_N;
output                      DRAM_CKE;
output                      DRAM_CLK;
output                      DRAM_CS_N;
inout           [15:0]      DRAM_DQ;
output                      DRAM_LDQM;
output                      DRAM_UDQM;
output                      DRAM_RAS_N;
output                      DRAM_WE_N;

////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]

//=======================================================
//  REG/WIRE declarations
//=======================================================
parameter COMPILE_HISTOGRAM_SUPPORT = 0;

wire video_clk_40Mhz;
wire vga_de;

//LFSR
logic [4:0] lfsr_data;
logic lfsr_reset;
logic Clock_1Hz;
reg async_lfsr_0;
logic sync_lfsr_0;

//DDS
wire [11:0] dds_sin_out;
wire [11:0] dds_cos_out;
wire [11:0] dds_sq_out;
wire [11:0] dds_saw_out;

// modulation DDS
wire [11:0] dds_sin_ASK;
wire [11:0] dds_sin_comp;
wire [11:0] dds_sin_BPSK;

//VGA
wire [10:0]CounterX;
wire [10:0]CounterY;
wire      [7:0]  vga_R;
wire      [7:0]  vga_G;
wire      [7:0]  vga_B;
wire cursor_on;
wire [11:0]cursor_RGB;    
wire graph_on;
wire [7:0]R_graph;
wire [7:0]G_graph;
wire [7:0]B_graph;
wire graph_on2;
wire [7:0]R_graph2;
wire [7:0]G_graph2;
wire [7:0]B_graph2;
wire graph_on3;
wire [7:0]R_graph3;
wire [7:0]G_graph3;
wire [7:0]B_graph3;
wire [9:0]data_plot;
(* keep = 1, preserve = 1 *) wire sampler;
wire[9:0]mouse_x;
wire[9:0]mouse_y;

//KEYBOARD
wire        wKeyboardEventReady;
wire [7:0]  wKeyboardEventType;

//HISTOGRAM
logic bar_on;
logic bar_on_raw;
wire [2047:0]histoGrama;
logic [15:0]to_histogram;
logic synced_bar_on;
logic [15:0] flash_audio_data_left_to_histogram, actual_audio_to_histogram;


//modulator
wire [7:0]signal_selector;
wire [3:0]modulation_selector;
wire [31:0]div_freq_count;
wire [31:0]keyboard_keys;

//Audio Generation Signal
//Note that the audio needs signed data - so convert 1 bit to 8 bits signed
wire [31:0]nios_audio_syn_data;
wire audio_selector;
wire [15:0] flash_audio_data;
wire [15:0] flash_audio_data_left, flash_audio_data_right;
(* keep = 1, preserve = 1 *) wire [15:0] actual_audio_data_left;
(* keep = 1, preserve = 1 *) wire [15:0] actual_audio_data_right;
wire audio_left_clock, audio_right_clock;

logic [31:0]DATA_AUDIO; 
logic [31:0]DATA_DIV_FREG; 
logic L;
logic R;
logic [15:0]Left_channel;
logic [15:0]Right_channel;
logic WRREQ;
logic WRCLK;
logic STOP; 
logic PAUSE;
logic FIFO_FULL;
logic EMPTY;
logic audio_sampling;
logic [11:0]used_fifo;

/*Keyboard Keys*/
reg HOLDING_1;
reg HOLDING_2;
reg HOLDING_3;
reg HOLDING_4;
reg HOLDING_5;
reg HOLDING_6;
reg HOLDING_7;
reg HOLDING_8;

reg HOLDING_F1;
reg HOLDING_F2;
reg HOLDING_N;
reg HOLDING_M;
reg HOLDING_SPACE;
reg HOLDING_LEFT_ARROW;
reg HOLDING_RIGHT_ARROW;
reg HOLDING_UP_ARROW;
reg HOLDING_DOWN_ARROW;
 
(*keep = 1, preserve = 1 *)logic CLK_25MHZ;



//=======================================================
//  Structural coding 
//=======================================================
assign VGA_BLANK=vga_de;
assign VGA_CLK=video_clk_40Mhz;
assign VGA_SYNC=1'b1 & SW[1];

logic reset_trigger;
logic system_reset;
logic fast_reset;

generate_one_shot_pulse 
#(
.num_clks_to_wait(2)
)
generate_reset_trigger
( 
.clk(CLOCK_50), 
.oneshot_pulse(reset_trigger)
);

generate_controlled_length_pulse
#(
.default_count           (10000),
.num_bits_counter        (24 ),
.pulse_out_initial_value (1  )
)
generate_system_reset
(
.async_reset(reset_trigger),
.pulse_out(system_reset),
.clk(CLOCK_50)
);

generate_controlled_length_pulse
#(
.default_count           (1000),
.num_bits_counter        (24 ),
.pulse_out_initial_value (1  )
)
generate_fast_system_reset
(
.async_reset(reset_trigger),
.pulse_out(fast_reset),
.clk(CLOCK_50)
);

logic CPU_CLK;

SDRAM_PLL 	PLL1	(.inclk0(CLOCK_50),.c0(CPU_CLK),.c1(DRAM_CLK),.c2(video_clk_40Mhz));

always @(negedge CLOCK_50)
begin
      CLK_25MHZ<= !CLK_25MHZ;
end

/// NIOS II Qsys

DE2_QSYS U0( 
		.clk_clk(CLOCK_50),                        //                     clk.clk
		.reset_reset_n(!system_reset),                  //                   reset.reset_n
		.key_external_connection_export(KEY), // key_external_connection.export
		 .clk_25_in_clk                                 (CLK_25MHZ),                                 //                       clk_25_in.clk
        .clk_40_in_clk                                 (video_clk_40Mhz),                                 //                       clk_40_in.clk
        .cpu_clk_for_sdram_clk                         (CPU_CLK),                          //               cpu_clk_for_sdram.clk

	   .sdram_wire_addr(DRAM_ADDR),                //              sdram_wire.addr
		.sdram_wire_ba({DRAM_BA_1,DRAM_BA_0}),                  //                        .ba
		.sdram_wire_cas_n(DRAM_CAS_N),               //                        .cas_n 
		.sdram_wire_cke(DRAM_CKE),                 //                        .cke
		.sdram_wire_cs_n(DRAM_CS_N),                //                        .cs_n
		.sdram_wire_dq(DRAM_DQ),                  //                        .dq
		.sdram_wire_dqm({DRAM_UDQM,DRAM_LDQM}),                 //                        .dqm
		.sdram_wire_ras_n(DRAM_RAS_N),               //                        .ras_n
		.sdram_wire_we_n(DRAM_WE_N) ,                //                        .we_n
		.vga_alt_vip_itc_0_clocked_video_vid_clk       (video_clk_40Mhz),       // vga_alt_vip_itc_0_clocked_video.vid_clk
	   .vga_alt_vip_itc_0_clocked_video_vid_data      ({vga_R[7:0],vga_G[7:0],vga_B[7:0]}),      //                                .vid_data
	   .vga_alt_vip_itc_0_clocked_video_vid_datavalid (vga_de), //                                .vid_datavalid
	   .vga_alt_vip_itc_0_clocked_video_vid_v_sync    (VGA_VS),    //                                .vid_v_sync
	   .vga_alt_vip_itc_0_clocked_video_vid_h_sync    (VGA_HS),    //                                .vid_h_sync
		
		//AUDIO 
		.audio2fifo_0_data_divfrec_export              (DATA_DIV_FREG),              //       audio2fifo_0_data_divfrec.export
	   .audio2fifo_0_empty_export                     (EMPTY),                     //              audio2fifo_0_empty.export
	   .audio2fifo_0_fifo_full_export                 (FIFO_FULL),                 //          audio2fifo_0_fifo_full.export
	   .audio2fifo_0_fifo_used_export                 (used_fifo),                 //          audio2fifo_0_fifo_used.export
	   .audio2fifo_0_out_data_audio_export            (DATA_AUDIO),            //     audio2fifo_0_out_data_audio.export
	   .audio2fifo_0_out_pause_export                 (PAUSE),                 //          audio2fifo_0_out_pause.export
	   .audio2fifo_0_out_stop_export                  (STOP),                  //           audio2fifo_0_out_stop.export
	   .audio2fifo_0_wrclk_export                     (WRCLK),                     //              audio2fifo_0_wrclk.export
	   .audio2fifo_0_wrreq_export                     (WRREQ),                   //              audio2fifo_0_wrreq.export
		
		
		//interfaces
	   .signal_selector_export                        (signal_selector[7:0]),                        //                 signal_selector.export
	   .modulation_selector_export                    (modulation_selector[3:0]),                    //             modulation_selector.export
	   .keyboard_keys_export                          (keyboard_keys[31:0]),                          //                   keyboard_keys.export
	   .mouse_pos_export                              ({12'd0,mouse_x[9:0],mouse_y[9:0]}),                              //                       mouse_pos.export
	   .div_freq_export                               (nios_audio_syn_data[31:0]),                               //                        div_freq.export
	   .audio_sel_export                              (audio_selector),                               //                       audio_sel.export
	   
       .vga_vga_clk_clk                               ()
       
	);
	

////////////////////////////////////////////////////////////////////
// 
//                       Put DDS + LFSR Code Here
//
////////////////////////////////////////////////////////////////////		

/// LFSR code
Generate_Arbitrary_Divided_Clk32 
Gen_1Hz_clk
(.inclk(CLOCK_50),
.outclk(Clock_1Hz),
.outclk_Not(),
.div_clk_count(32'h17D7840),
.Reset(1'h1)
); 

lfsr_5b lfsr_inst(	.clk(Clock_1Hz),
							.reset(!KEY[1]), // should set this to 0. How do we actually reset this D:
							.q(lfsr_data));
   
/// DDS code
waveform_gen waveform_gen_inst(	.clk(CLOCK_50),
											.reset(1'b1),
											.en(1'b1),
											.phase_inc(32'h81),
											.sin_out(dds_sin_out),
											.cos_out(dds_cos_out),
											.squ_out(dds_sq_out),
											.saw_out(dds_saw_out));
											
/// Modulate DDS with LFSR 
// Synchronize the LFSR bitstream
clock_domain_sync #(1) LFSR_bit_sync(
									.fast_clk(CLOCK_50),
									.slow_clk(Clock_1Hz),
									.async_data_in(lfsr_data[0]),
									.sync_data_out(sync_lfsr_0));

// I think this is synced to the 1Hz clock, need to cross clock domains									


	


// ASK modulation 
assign dds_sin_ASK = sync_lfsr_0 ? dds_sin_out : 12'b0;

// BPSK modulation
assign dds_sin_BPSK = sync_lfsr_0 ? dds_sin_out : dds_sin_comp;
assign dds_sin_comp = ~dds_sin_out + 12'b1;

parameter [2:0] ASK_C = 3'b000;
parameter [2:0] FSK_C = 3'b001;
parameter [2:0] BPSK_C = 3'b010;
parameter [2:0] LFSR_C = 3'b011;

parameter [2:0] SIN_C = 3'b000;
parameter [2:0] COS_C = 3'b001;
parameter [2:0] SAW_C = 3'b010;
parameter [2:0] SQUARE_C = 3'b011;



wire dds_sin_FSK = 12'b0;
wire [11:0] mod_data_to_slow;
wire [11:0] sig_data_to_slow;


fast_to_slow_clk #(12) mod_sync(
.fast_clk(CLOCK_50),
.slow_clk(sampler),
.async_data_in(mod_data_to_slow),
.sync_data_out(actual_selected_modulation));

fast_to_slow_clk #(12) signal_sync(
.fast_clk(CLOCK_50),
.slow_clk(sampler),
.async_data_in(sig_data_to_slow),
.sync_data_out(actual_selected_signal));

always_comb
begin

	case(modulation_selector)
	ASK_C: 	mod_data_to_slow = dds_sin_ASK;
	FSK_C: 	mod_data_to_slow = dds_sin_FSK;
	BPSK_C: mod_data_to_slow = dds_sin_BPSK;
	LFSR_C: mod_data_to_slow = {lfsr_data[0], 11'b0};
	default: mod_data_to_slow = dds_sin_ASK;
	
	endcase
	
	case(signal_selector)
	
	SIN_C: sig_data_to_slow = dds_sin_out;
	COS_C:	sig_data_to_slow = dds_cos_out;
	SAW_C:	sig_data_to_slow = dds_saw_out;
	SQUARE_C:	sig_data_to_slow = dds_sq_out;
	default: sig_data_to_slow = dds_sin_out;
	
	endcase
end

(* keep = 1, preserve = 1 *) logic [11:0] actual_selected_modulation;
(* keep = 1, preserve = 1 *) logic [11:0] actual_selected_signal;


////////////////////////////////////////////////////////////////////
// 
//                       End of Student Code Section
//
////////////////////////////////////////////////////////////////////	
assign keyboard_keys[31:0]={15'd0,HOLDING_1,HOLDING_2,HOLDING_3,HOLDING_4,HOLDING_5,HOLDING_6,HOLDING_7,HOLDING_8,HOLDING_F1,HOLDING_F2,HOLDING_N,HOLDING_M,HOLDING_SPACE,HOLDING_LEFT_ARROW,HOLDING_RIGHT_ARROW,HOLDING_UP_ARROW,HOLDING_DOWN_ARROW};


///KEYBOARD --------------------

Keyboard_Controller kc0 (
	  .iPS2_CLK    (PS2_CLK),
	  .iPS2_DAT    (PS2_DAT),
	  .iCLK        (CLK_25MHZ),
	  .iRESET      (1'b0),
	  .oEventReady (wKeyboardEventReady),
	  .oEventType  (wKeyboardEventType)
 );

 logic reset_from_key, reset_from_key_clk_40, graph_enable_scroll;
 
 doublesync_no_reset sync_reset_from_key (.indata (!KEY[0]),    .outdata(reset_from_key),    .clk    (CLK_25MHZ));
 doublesync_no_reset sync_reset_from_key_clk_40 (.indata (!KEY[0]),    .outdata(reset_from_key_clk_40),    .clk    (video_clk_40Mhz));
 doublesync_no_reset sync_graph_enable_scroll (.indata (SW[0]),    .outdata(graph_enable_scroll),    .clk    (video_clk_40Mhz));


// Detectar letras.
 always @(posedge CLK_25MHZ) begin
	  if(reset_from_key) begin
			HOLDING_SPACE         <= 1'b0;
			HOLDING_LEFT_ARROW   <= 1'b0;
			HOLDING_RIGHT_ARROW  <= 1'b0;
			HOLDING_UP_ARROW     <= 1'b0;
			HOLDING_DOWN_ARROW   <= 1'b0;
			HOLDING_M<=1'b0;
			HOLDING_N<=1'b0;
			HOLDING_F1<=1'b0;
			HOLDING_F2<=1'b0;
			
			HOLDING_1<=1'b0;
			HOLDING_2<=1'b0;
			HOLDING_3<=1'b0;
			HOLDING_4<=1'b0;
			HOLDING_5<=1'b0;
			HOLDING_6<=1'b0;
			HOLDING_7<=1'b0;
			HOLDING_8<=1'b0;
	  end
	  else begin
			if(wKeyboardEventType == `SC_MAKE_SPACE)             HOLDING_SPACE        <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_SPACE)       HOLDING_SPACE        <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_LEFT_ARROW)        HOLDING_LEFT_ARROW  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_LEFT_ARROW)  HOLDING_LEFT_ARROW  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_RIGHT_ARROW)       HOLDING_RIGHT_ARROW <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_RIGHT_ARROW)   HOLDING_RIGHT_ARROW <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_UP_ARROW)          HOLDING_UP_ARROW    <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_UP_ARROW)    HOLDING_UP_ARROW    <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_DOWN_ARROW)        HOLDING_DOWN_ARROW  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_DOWN_ARROW)  HOLDING_DOWN_ARROW  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_N)        HOLDING_N  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_N)  HOLDING_N  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_M)        HOLDING_M  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_M)  HOLDING_M  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_F1)        HOLDING_F1  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_F1)  HOLDING_F1  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_F2)        HOLDING_F2  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_F2)  HOLDING_F2  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_1)        HOLDING_1  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_1)  HOLDING_1  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_2)        HOLDING_2  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_2)  HOLDING_2  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_3)        HOLDING_3  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_3)  HOLDING_3  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_4)        HOLDING_4  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_4)  HOLDING_4  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_5)        HOLDING_5  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_5)  HOLDING_5  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_6)        HOLDING_6  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_6)  HOLDING_6  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_7)        HOLDING_7  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_7)  HOLDING_7  <= 1'b0;
			if(wKeyboardEventType == `SC_MAKE_8)        HOLDING_8  <= 1'b1;
			else if(wKeyboardEventType == `SC_BREAK_8)  HOLDING_8  <= 1'b0;
	  end
 end
 
 
 
/* video X Y counter generation */
hack_ltm_sincronization hack_ltm_sincronization_inst
(
	.clk_lcd(video_clk_40Mhz) ,	// input  clk_lcd_sig
	.den_lcd(vga_de) ,	// input  den_lcd_sig
	.count_x(CounterX) ,	// output [10:0] count_x_sig
	.count_y(CounterY) , 	// output [10:0] count_y_sig
	.reset(1'b1)
);

 logic HOLDING_LEFT_ARROW_sync_to_video ;
 logic HOLDING_UP_ARROW_sync_to_video   ;
 logic HOLDING_DOWN_ARROW_sync_to_video ;
 logic HOLDING_RIGHT_ARROW_sync_to_video;
 logic HOLDING_M_sync_to_video;
 logic HOLDING_SPACE_sync_to_video;
 
 
 doublesync_no_reset sync_HOLDING_LEFT_ARROW  (.indata (HOLDING_LEFT_ARROW),    .outdata(HOLDING_LEFT_ARROW_sync_to_video),    .clk    (video_clk_40Mhz));
 doublesync_no_reset sync_HOLDING_UP_ARROW    (.indata (HOLDING_UP_ARROW),      .outdata(HOLDING_UP_ARROW_sync_to_video),      .clk    (video_clk_40Mhz));
 doublesync_no_reset sync_HOLDING_DOWN_ARROW  (.indata (HOLDING_DOWN_ARROW),    .outdata(HOLDING_DOWN_ARROW_sync_to_video),    .clk    (video_clk_40Mhz));
 doublesync_no_reset sync_HOLDING_RIGHT_ARROW (.indata (HOLDING_RIGHT_ARROW),   .outdata(HOLDING_RIGHT_ARROW_sync_to_video),   .clk    (video_clk_40Mhz));
 doublesync_no_reset sync_HOLDING_M (.indata (HOLDING_M),   .outdata(HOLDING_M_sync_to_video),   .clk    (video_clk_40Mhz));
 doublesync_no_reset sync_HOLDING_SPACE (.indata (HOLDING_SPACE),   .outdata(HOLDING_SPACE_sync_to_video),   .clk    (video_clk_40Mhz));

//Cursor
/*Cursor Control*/ 
Cursor Cursor_inst
(
	.iLeft (HOLDING_LEFT_ARROW_sync_to_video  ) ,	// input  iLeft_sig
	.iUp   (HOLDING_UP_ARROW_sync_to_video    ) ,	// input  iUp_sig
	.iDown (HOLDING_DOWN_ARROW_sync_to_video  ) ,	// input  iDown_sig
	.iRight(HOLDING_RIGHT_ARROW_sync_to_video ) ,	// input  iRight_sig
	.iVGA_X(CounterX) ,	// input [10:0] iVGA_X_sig
	.iVGA_Y(CounterY) ,	// input [10:0] iVGA_Y_sig
	.oRGB(cursor_RGB[11:0]) ,	// output [11:0] oRGB_sig
	.iRST(reset_from_key_clk_40) ,	// input  iRST_sig
	.iCLK(video_clk_40Mhz) ,	// input  iCLK_sig
	.slow(HOLDING_M_sync_to_video) ,	// input  slow_sig
	.ichangeC(HOLDING_SPACE_sync_to_video) ,	// input  ichangeC_sig
	.arrow_on(cursor_on) ,	// output  arrow_on_sig
	.pointer_x(mouse_x[9:0]),
	.pointer_y(mouse_y[9:0])
	
);	 


Generate_Arbitrary_Divided_Clk32 
Generate_Sampler_Clock(
.inclk(!video_clk_40Mhz),
.outclk(sampler),
.outclk_Not(),
.div_clk_count(32'd112000),
.Reset(1'h1));

//VGA Oscilloscope Modules
plot_graph plot_graph1
(
	.CountX_in(CounterX) ,	// input [Xcount-1:0] CountX_in_sig
	.CountY_in(CounterY) ,	// input [Ycount-1:0] CountY_in_sig
	.data_graph({~actual_selected_modulation[11],actual_selected_modulation[10:4]}) ,	// input [size_data-1:0] data_graph_sig
	.data_graph_rdy(sampler) ,	// input  data_graph_rdy_sig
	.display_clk(video_clk_40Mhz) ,	// input  display_clk_sig
	.color_graph(24'h00ff30) ,	// input [numberRGB-1:0] color_graph_sig
	.scroll_en(graph_enable_scroll) ,	// input  scroll_en_sig
	.Pos_X(11'd0) ,	// input [Xcount-1:0] Pos_X_sig
	.Pos_Y(11'd266) ,	// input [Ycount-1:0] Pos_Y_sig
	.Witdh(11'd640) ,	// input [width_grp-1:0] Witdh_sig
	.Height(3'd2) ,	// input [height_grp-1:0] Height_sig
	.line_width(2'd0) ,	// input [2:0] line_width_sig
	.Graph_en(1'b1) ,	// input  Graph_en_sig
	.Graph_on(graph_on) ,	// output  Graph_on_sig 
	.R(R_graph[7:0]) ,	// output [numberRGB-17:0] R_sig
	.G(G_graph[7:0]) ,	// output [numberRGB-17:0] G_sig
	.B(B_graph[7:0]) ,	// output [numberRGB-17:0] B_sig
	.reset_n(1'b1) ,	// input  reset_n_sig
	.pause_graph(1'b0) 	// input  pause_graph_sig
);

plot_graph plot_graph2
(
	.CountX_in(CounterX) ,	// input [Xcount-1:0] CountX_in_sig
	.CountY_in(CounterY) ,	// input [Ycount-1:0] CountY_in_sig
	.data_graph({~actual_selected_signal[11],actual_selected_signal[10:4]}) ,	// input [size_data-1:0] data_graph_sig
	.data_graph_rdy(sampler) ,	// input  data_graph_rdy_sig
	.display_clk(video_clk_40Mhz) ,	// input  display_clk_sig
	.color_graph(24'h00ff30) ,	// input [numberRGB-1:0] color_graph_sig
	.scroll_en(graph_enable_scroll) ,	// input  scroll_en_sig
	.Pos_X(11'd0) ,	// input [Xcount-1:0] Pos_X_sig
	.Pos_Y(11'd438) ,	// input [Ycount-1:0] Pos_Y_sig
	.Witdh(11'd640) ,	// input [width_grp-1:0] Witdh_sig
	.Height(3'd2) ,	// input [height_grp-1:0] Height_sig
	.line_width(2'd0) ,	// input [2:0] line_width_sig
	.Graph_en(1'b1) ,	// input  Graph_en_sig
	.Graph_on(graph_on2) ,	// output  Graph_on_sig 
	.R(R_graph2[7:0]) ,	// output [numberRGB-17:0] R_sig
	.G(G_graph2[7:0]) ,	// output [numberRGB-17:0] G_sig
	.B(B_graph2[7:0]) ,	// output [numberRGB-17:0] B_sig
	.reset_n(1'b1) ,	// input  reset_n_sig
	.pause_graph(1'b0) 	// input  pause_graph_sig
);



/*MUX DE VIDEO*/

// VGA NIOS II
reg [7:0]video_r_out;
reg [7:0]video_g_out;
reg [7:0]video_b_out;


always@(negedge video_clk_40Mhz)
begin	
	if(cursor_on)
	begin
				video_r_out[7:0]<={cursor_RGB[11:8],4'd0};
				video_g_out[7:0]<={cursor_RGB[7:4],4'd0};
				video_b_out[7:0]<={cursor_RGB[3:0],4'd0};
	end
	else if(graph_on || graph_on2 || graph_on3 )
	begin
				video_r_out[7:0]<=graph_on ? R_graph[7:0]:
									   graph_on2 ? R_graph2[7:0]:
										R_graph3[7:0];
				video_g_out[7:0]<=graph_on ? G_graph[7:0]:
									   graph_on2 ? G_graph2[7:0]:
										G_graph3[7:0];
				video_b_out[7:0]<=graph_on ? B_graph[7:0]:
									   graph_on2 ? B_graph2[7:0]:
										B_graph3[7:0];
	end
	 else if(bar_on)
	 begin
				video_r_out[7:0]<=8'd255;
				video_g_out[7:0]<=Left_channel[7:0]+Left_channel[15:8];
				video_b_out[7:0]<=Right_channel[7:0];
	 end
	else
	begin
				video_r_out[7:0]<=vga_R[7:0];
				video_g_out[7:0]<=vga_G[7:0];
				video_b_out[7:0]<=vga_B[7:0];
	end			
end
 
 assign {VGA_R[9:2],VGA_G[9:2],VGA_B[9:2]}={video_r_out[7:0],video_g_out[7:0],video_b_out[7:0]};

 extern module audio_subsystem_w_histogram	
(
  input [31:0]DATA_AUDIO, 
   input logic WRREQ,
   input logic WRCLK,
   input logic STOP,
   input logic PAUSE,
   input logic [31:0]DATA_DIV_FREG, 
   input CLOCK_50,
   output FIFO_FULL,
   output EMPTY,
   input video_clk_40Mhz,
   input              AUD_ADCDAT,
   inout              AUD_ADCLRCK,
   inout              AUD_BCLK,
   output             AUD_DACDAT,
   inout              AUD_DACLRCK,
   output             AUD_XCK,
   output             FPGA_I2C_SCLK,
   inout              FPGA_I2C_SDAT,
   input  logic [10:0]CounterX,
   input  logic [10:0]CounterY,
   output logic       bar_on,
   output logic [11:0]used_fifo
);	 
  
  extern module audio_subsystem_no_histogram	
(
  input [31:0]DATA_AUDIO, 
   input logic WRREQ,
   input logic WRCLK,
   input logic STOP,
   input logic PAUSE,
   input logic [31:0]DATA_DIV_FREG, 
   input CLOCK_50,
   output FIFO_FULL,
   output EMPTY,
   input video_clk_40Mhz,
   input              AUD_ADCDAT,
   inout              AUD_ADCLRCK,
   inout              AUD_BCLK,
   output             AUD_DACDAT,
   inout              AUD_DACLRCK,
   output             AUD_XCK,
   output             FPGA_I2C_SCLK,
   inout              FPGA_I2C_SDAT,
   input  logic [10:0]CounterX,
   input  logic [10:0]CounterY,
   output logic       bar_on,
   output logic [11:0]used_fifo
);	 
  
 `ifdef ENABLE_AUDIO_DEMO 
			  generate 
                          if (COMPILE_HISTOGRAM_SUPPORT)
						  begin										 
								 audio_subsystem_w_histogram
								 audio_subsystem_inst
								 (
								 .*
								 );
						  end
						  else
						  begin						  
								 audio_subsystem_no_histogram
								 audio_subsystem_inst
								 (
								 .*
								 );						  								 
						  end						  
			endgenerate				
`endif 

endmodule
`default_nettype wire
