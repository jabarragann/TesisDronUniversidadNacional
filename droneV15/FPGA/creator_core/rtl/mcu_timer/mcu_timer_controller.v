`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Juan Antonio Barragán
// 
// Create Date:    10:02:32 10/22/2016 
// Design Name: 
// Module Name:    mcu_timer_controller 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module mcu_timer_controller #(
	parameter adr_width = 8,
	parameter data_width = 8
) (
  input reset,
  //MCU SAM
  input  timer_clk,
  input  timer_nwe,
  input  timer_ncs,
  input  timer_nrd,
  input  [7:0] timer_addr, 
  input  [7:0] timer_sram_data 
  );

/* Needed Signals*/
wire [15:0] timerOut1;
wire timerEnable1;

/* synchronize signals */
reg    timer_sncs;
reg    timer_snwe;
reg    [7:0] timer_buffer_addr;
reg    [7:0] timer_buffer_data;  

/* bram interfaz signals */
reg    timer_we;
reg    timer_w_st;

reg    [7:0] timer_wdBus;
wire   [7:0] timer_rdBus;

reg [7:0] timer_rdBus2;

assign timer_rdBus=timer_rdBus2;

/* interefaz signals assignments */
wire   T = (timer_nrd | timer_ncs);

//pin tri-state
assign timer_sram_data = T ? 8'bZ:timer_rdBus;


mcu_timer timer1 (
  	.reset(reset),
  	.clk  (input_clk),
  	.enable(1), 
  	.timerOut(timerOut1)
  	);


// synchronize assignment 
always  @(negedge timer_clk)
begin
  timer_sncs   <= timer_ncs;
  timer_snwe   <= timer_nwe;
  timer_buffer_data <= timer_sram_data;
  timer_buffer_addr <= timer_addr;
end

// write access cpu to bram 
always @(posedge timer_clk)
begin
 timer_wdBus <= timer_buffer_data;
 case (timer_w_st)
   0: begin
        timer_we <= 0;
        if (timer_sncs | timer_snwe) 
          timer_w_st <= 1;
      end
   1: begin
        if (~(timer_sncs | timer_snwe)) begin
          timer_we    <= 1;
          timer_w_st  <= 0;
        end	
          else timer_we <= 0;
      end
  endcase
end

always @(posedge timer_clk) begin
 case(timer_buffer_addr[2:0])
 0: timer_rdBus2[7:0]=8'hC8;
 1: timer_rdBus2[7:0]=8'hC9;
 2: timer_rdBus2[7:0]=8'hCA;
 3: timer_rdBus2[7:0]=8'hCB;
 4: timer_rdBus2[7:0]=8'hCC;
 5: timer_rdBus2[7:0]=8'hCD;
 default:
	timer_rdBus2[7:0]=8'hBB;
 endcase
end
	
  
endmodule
