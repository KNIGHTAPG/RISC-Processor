`timescale 1ns / 1ps

module risc_processor_tb( );
reg clk1, start;
      
wire read_i_s,
     read_r_s,
     write_ra_s,
     write_rb_s,     
     ALUon_s,
     read_d_s,
     mux_signal_s,
     pc_signal_s;
wire [0:7] FR_s;
wire [31:0] I_s;
wire [2:0] state_s;
wire [31:0] A_s, B_s, L_s, Z_s, F_s, acc_s; 
wire [15:0] count_s, S_s;     
integer k=0,m=0;



risc_processor DP(start,clk1);

initial begin 
        #0 clk1 = 0; 
        
        
           DP.RAM.ram_mem[0] = 7;               // these 6 values are stored inside the RAM
           DP.RAM.ram_mem[1] = 2;               // the u.p. has to fetch these values one by one from RAM
           DP.RAM.ram_mem[2] = 9;               // the u.p. has to then compare these values to find largest value.
           DP.RAM.ram_mem[3] = 14; 
           DP.RAM.ram_mem[4] = 45; 
           DP.RAM.ram_mem[5] = 23; 
                
           for(k=0; k<8; k=k+1) 
           begin DP.RB.regbank[k] = 0; end      // all the GPR are intialized to zero

           DP.ROM.rom_mem[0] = 32'h30020000;    // MOVU - - R2 16'h00
           DP.ROM.rom_mem[1] = 32'h32020006;    // MOVL - - R2 16'h06
           DP.ROM.rom_mem[2] = 32'h30050000;    // MOVU - - R5 16'h00
           DP.ROM.rom_mem[3] = 32'h32050001;    // MOVL - - R5 16'h01
           DP.ROM.rom_mem[4] = 32'h0F000000;    // LOAD R4 - R0 0
           DP.ROM.rom_mem[5] = 32'h032C0000;    // ADD R4 R5 R4 - 
           DP.ROM.rom_mem[6] = 32'h0F030000;    // LOAD R4 - R3 0 
           DP.ROM.rom_mem[7] = 32'h032C0000;    // ADD R4 R5 R4 - 
           DP.ROM.rom_mem[8] = 32'h04AA0000;    // SUB R2 R5 R2 - 
           DP.ROM.rom_mem[9] = 32'h1880000F;    // JZ R2 - - 15
           DP.ROM.rom_mem[10] = 32'h1CC00006;   // JE R3 R0 - 6
           DP.ROM.rom_mem[11] = 32'h0AC00000;   // LT R3 R0 - - 
           DP.ROM.rom_mem[12] = 32'h2E000006;   // JFNZ - - - 6
           DP.ROM.rom_mem[13] = 32'h2AC00000;   // MOVR R3 - R0 -
           DP.ROM.rom_mem[14] = 32'h20000006;   // JMP - - - 6
           DP.ROM.rom_mem[15] = 32'h03810000;   // ADD R6 R0 R1 - 
           DP.ROM.rom_mem[16] = 32'h12000000;   // END - - - -         
        end
        
initial begin
        repeat(500)
        begin
        #5 clk1 = 1; #5 clk1 = 0;
        end
        end 


assign read_i_s = DP.read_i;
assign read_r_s = DP.read_r;
assign write_ra_s = DP.write_ra;
assign write_rb_s = DP.write_rb;
assign ALUon_s = DP.ALUon;
assign read_d_s = DP.read_d;
assign mux_signal_s = DP.mux_signal;
assign pc_signal_s = DP.pc_signal;
assign FR_s = DP.FR;
assign I_s = DP.I;

assign state_s = DP.CS.state;
assign S_s = DP.S;
assign A_s = DP.A; 
assign B_s = DP.B; 
assign L_s = DP.L;
assign Z_s = DP.Z;
assign F_s = DP.F; 
assign count_s = DP.count;
assign acc_s = DP.acc;



        
initial begin
        // #0 DP.CS.state = 3'b000;  
        // already state is initialzed to SX in CS module
        #20 start = 1;
        #100 start = 0; 
        $monitor ($time, "R0=%d, R1=%d, R2=%d, R3=%d, R4=%d, R5=%d", 
        DP.RB.regbank[0], DP.RB.regbank[1], DP.RB.regbank[2], DP.RB.regbank[3], DP.RB.regbank[4], DP.RB.regbank[5]);
        #10000 $finish;
        end         

endmodule
