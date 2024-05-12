`timescale 1ns / 1ps

module risc_processor (start,clk1);      // use pushbutton for start & fn. generator for clk1
input start,clk1;


wire [2:0] rs1, rs2, rd;           // rs1 & rs2 are source registers, rd is destination register
wire [6:0] opcode;                 // opcode tells arithmatic logic unit, which operation has to be performed
wire [15:0] S;                     // it has multiple uses, generally used as address for branch intruction
wire [15:0] count;                 // count, holds address of next instruction to be executed
wire [31:0] A, B, Z, F, L, acc;    // A,B is data loaded from register bank
                                   // Z is output of arithmetic logic unit
                                   // F is final data that will be put into register bank
                                   // L is the data loaded from RAM 
                                   // acc & Z are used together to store 64 bit product


wire read_i;                       // read_i = 1 ROM send next instruction to decoder
wire read_r;                       // read_r = 1 depending on rs1, rs2 values, data from RB is given to ALU
wire ALUon;                        // ALUon = 1 ALU performs operation on data recieved from RB
wire read_d                        // read_d = 1 read data from RAM, read_d = 0 write data into RAM
wire mux_signal;                   // mux_signal = 1 data from RAM gets stored into RB
                                   // mux_signal = 0 data from ALU gets stored into RB
wire write_ra;                     // write_ra = 1  reg[address] = F
                                   // write_ra = 0  reg[address] = F   reg[address+1] = acc
wire write_rb;                     // write_rb = 1  reg[31:16] = address
                                   // write_rb = 0  reg[15:0] = address
wire pc_signal;                    // pc_signal = 1 increment current address
                                   // pc_signal = 0 load new address
wire [31:0] I;                     // 32 bit instruction = | opcode(7) | rs1(3) | rs2(3) | rd(3) | address(16) | 
wire [0:7] FR;                     // 8 bit flag register
                                   // FR[1] = 1 for all intructions except MUL,MOVU,MOVL
                                   // FR[2] = 1 to load new address, FR[2] = 0 to increase address
                                   // FR[3] = 1 for rs1 < rs2, FR[3] = 0 for rs1 > rs2
                                   // FR[4] = 1 for only MUL instruction
                                   // FR[5] = 1 for only MOVU instruction
                                   // FR[6] = 1 for only MOVL instruction
                                   // FR[0] and FR[7] are unused;

parameter ADD=7'b0000001,  SUB=7'b0000010,   AND=7'b0000011,   OR=7'b0000100,
          LT=7'b0000101,   GT=7'b0000110,    LOAD=7'b0000111,  STORE=7'b0001000, 
          END=7'b0001001,  RR=7'b0001010,    RL=7'b0001011,    JZ=7'b0001100,
          JNZ=7'b0001101,  JE=7'b0001110,    JNE=7'b0001111,   JMP=7'b0010000,
          MUL=7'b0010001,  DIV=7'b0010010,   MOD=7'b0010011,   NOT=7'b0010100,
          MOVR=7'b0010101, JFZ=7'b0010110,   JFNZ=7'b0010111,  MOVU=7'b0011000,
          MOVL=7'b0011001;
          

read_only_memory ROM(.address(count), 
                     .dout(I), 
                     .clk(clk1), 
                     .read(read_i));
decoder DEC(.din(I), 
            .dout0(rs1), 
            .dout1(rs2), 
            .dout2(rd), 
            .dout3(opcode), 
            .dout4(S));
random_access_memory RAM(.address(S), 
                         .dout(L), 
                         .din0(B), 
                         .din1(A), 
                         .clk(clk1), 
                         .read(read_d));
multiplexer MUX(.dout(F), 
                .din0(Z), 
                .din1(L), 
                .select(mux_signal), 
                .clk(clk1));
program_counter PC(.out(count), 
                   .in(S), 
                   .pc_signal(pc_signal), 
                   .clk(clk1));
register_bank RB(.destination_reg1_d(F), 
                 .destination_reg2_d(acc), 
                 .destination_reg3_d(S), 
                 .source_reg1_d(A), 
                 .source_reg2_d(B), 
                 .destination_reg_a(rd), 
                 .source_reg1_a(rs1), 
                 .source_reg2_a(rs2), 
                 .clk(clk1), 
                 .load(read_r), 
                 .store1(write_ra),                   
                 .store2(write_rb));
arithmatic_logic_unit ALU(.dout0(Z), 
                          .FR(FR), 
                          .dout5(acc), 
                          .din0(A), 
                          .din1(B), 
                          .select(opcode), 
                          .clk(clk1), 
                          .enable(ALUon));
control_signals CS    (.read_i(read_i),
                       .read_r(read_r),
                       .ALUon(ALUon),
                       .read_d(read_d),
                       .mux_signal(mux_signal),
                       .write_ra(write_ra),                       
                       .write_rb(write_rb),                       
                       .pc_signal(pc_signal),
                       .I(I),
                       .FR(FR),
                       .clk1(clk1),
                       .start(start));                          

endmodule

////////////////////////////////////////////////////////////////////////////

module decoder(din,
               dout0,
               dout1,
               dout2,
               dout3,
               dout4);
input [31:0] din;
output reg[2:0]  dout0, dout1, dout2;     
output reg[6:0]  dout3;
output reg[15:0] dout4;



always@(din)
begin
dout0 <= din[24:22];            // opcode 7 bit
dout1 <= din[21:19];            // rs1 3 bit
dout2 <= din[18:16];            // rs2 3 bit
dout3 <= din[31:25];            // rd  3 bit
dout4 <= din[15:0];             // address 16 bit
end

endmodule


//////////////////////////////////////////////////////////////////////////////


module program_counter(out,
                       in,
                       pc_signal,
                       clk);     
input[15:0] in;                              // 16 bit synchronous up counter
output reg[15:0] out;                        // counts from 0000 to FFFF
input pc_signal, clk;                        // counts from 0 to 65535
                                             // can access 64 Kb of program memory (ROM)
initial 
begin
#0 out <= 16'h0000; 
end

always@(posedge clk)
begin
if(pc_signal == 1'b0) out <= in;
else if(pc_signal == 1'b1) out <= out + 1;
else out <= out;
end
endmodule

//////////////////////////////////////////////////////////////////////////////////

module read_only_memory (address, 
                         dout, 
                         clk, 
                         read);   
input[15:0] address;            // 16 bit address from program counter
output reg[31:0] dout;          // 32 bit instruction is output given by ROM
input clk,read;                 // when read = 1 & clk = posedge, then instruction is generated 
reg [31:0] rom_mem [65535:0];   // 2^16 = 65536 = 64Kb of program memory (ROM)
integer i;                      // to initialize all memory locations with zero values
                                // ROM has 65536 memory locations, each of size 32 bits

initial 
begin
for(i=0; i<65535; i=i+1)
begin
rom_mem [i] = 32'h00000000; 
end
end

always@ (posedge clk) 
begin
if(read == 1'b1) dout <= rom_mem[address];
else             dout <= dout;
end
endmodule





//////////////////////////////////////////////////////////////////



module arithmatic_logic_unit (dout0,
                              FR,
                              dout5,
                              din0,
                              din1,
                              select,
                              clk,
                              enable);
input [31:0] din0, din1;                       // din0 = A     din1 = B               
output reg[31:0] dout0, dout5;                 // dout0 = Z    dout5 = acc   
output reg [0:7] FR;                           // FR is 8 bit flag register
input [6:0] select;                            // select = opcode
input clk, enable;                             // when enable = 1 & clk = posedge, ALU generates output
                                               

parameter ADD=7'b0000001,  SUB=7'b0000010,   AND=7'b0000011,   OR=7'b0000100,
          LT=7'b0000101,   GT=7'b0000110,    LOAD=7'b0000111,  STORE=7'b0001000, 
          END=7'b0001001,  RR=7'b0001010,    RL=7'b0001011,    JZ=7'b0001100,
          JNZ=7'b0001101,  JE=7'b0001110,    JNE=7'b0001111,   JMP=7'b0010000,
          MUL=7'b0010001,  DIV=7'b0010010,   MOD=7'b0010011,   NOT=7'b0010100,
          MOVR=7'b0010101, JFZ=7'b0010110,   JFNZ=7'b0010111,  MOVU=7'b0011000,
          MOVL=7'b0011001;

always @(posedge clk)
begin
if(enable == 1'b1)
begin
case(select)
ADD:     begin dout0 <= din0 + din1;                        FR = 8'b010x0000; end
SUB:     begin dout0 <= din0 - din1;                        FR = 8'b010x0000; end
AND:     begin dout0 <= din0 & din1;                        FR = 8'b010x0000; end
OR:      begin dout0 <= din0 | din1;                        FR = 8'b010x0000; end
LT:      begin 
               if( din0 < din1 ) begin                      FR = 8'b00010000; end   
               else              begin                      FR = 8'b00000000; end   
         end
GT:      begin if( din0 > din1 ) begin                      FR = 8'b00000000; end   
               else              begin                      FR = 8'b00010000; end
         end
LOAD:    begin dout0 <= 32'hxxxxxxxx;                       FR = 8'b010x0000; end
STORE:   begin dout0 <= 32'hxxxxxxxx;                       FR = 8'b000x0000; end
END:     begin dout0 <= dout0;  dout5 <= dout5;                  FR <= FR; end
RR:      begin dout0 <= (din0 >> 4)|(din0 << 32-4);         FR = 8'b010x0000; end
RL:      begin dout0 <= (din0 << 4)|(din0 >> 32-4);         FR = 8'b010x0000; end
JZ:      begin 
         if(din0 == 0)                                begin FR = 8'b001x0000; end
         else                                         begin FR = 8'b000x0000; end
         end
JNZ:     begin 
         if(din0 != 0)                                begin FR = 8'b001x0000; end
         else                                         begin FR = 8'b000x0000; end
         end 
JE:      begin 
         if(din0 == din1)                             begin FR = 8'b001x0000; end
         else                                         begin FR = 8'b000x0000; end
         end 
JNE:     begin 
         if(din0 != din1)                             begin FR = 8'b001x0000; end
         else                                         begin FR = 8'b000x0000; end
         end
JMP:                                                  begin FR = 8'b001x0000; end
MUL:     begin {dout5[31:0],dout0[31:0]} = din0*din1;       FR = 8'b000x1000; end   
DIV:     begin dout0[31:0] = din0/din1;                     FR = 8'b010x0000; end
MOD:     begin dout0[31:0] = din0%din1;                     FR = 8'b010x0000; end
NOT:     begin dout0 = ~din0;                               FR = 8'b010x0000; end
MOVR:    begin dout0 = din0;                                FR = 8'b010x0000; end
JFZ:     begin 
               if(FR[3] == 0)                         begin FR = 8'b00100000; end 
               else                                   begin FR = 8'b00010000; end 
         end
JFNZ:    begin 
               if(FR[3] == 1)                         begin FR = 8'b00110000; end 
               else                                   begin FR = 8'b00000000; end 
         end  
MOVU:                                                 begin FR = 8'b000x0100; end 
MOVL:                                                 begin FR = 8'b000x0010; end               
default: begin dout0 <= dout0;                              FR = 8'bxxxxxxxx; end
endcase
end
end
endmodule


       
///////////////////////////////////////////////////////////////////////////
       
module random_access_memory(address,
                            dout,
                            din0,
                            din1,
                            clk,
                            read);
input [15:0] address;              // 16 bit address from decoder
input [31:0] din0,din1;            // din0 = B  din1 = A
output reg [31:0] dout;            // data stored in RAM given as output
input clk,read;                    // read = 1 data is read from RAM, read = 0 data stored into RAM
reg [31:0] ram_mem [65535:0];      // 2^16 = 65536 = 64Kb of data memory (RAM)
integer i;                         // to initialize all memory location with zero

initial 
begin
for(i=0; i<65535; i=i+1)
begin
ram_mem [i] = 32'h00000000; 
end
end

always@(posedge clk)
begin
if(read == 0)     ram_mem[address + din1] <= din0;
else if(read == 1) dout <= ram_mem[address + din1];
end
endmodule

/////////////////////////////////////////////////////////////////////////

module multiplexer(dout, 
                   din0, 
                   din1, 
                   select, 
                   clk);
input [31:0] din0, din1;                // din0 = Z,  din1 = L
output reg [31:0] dout;                 // dout = F
input select, clk;                      // select = 1 then data from RAM gets stored into RB
                                        // select = 0 then data from ALU gets stored into RB

always@ (posedge clk)
begin
if (select == 1'b0) dout <= din0;
else if (select == 1'b1) dout <= din1;
else dout <= dout;
end
endmodule

/////////////////////////////////////////////////////////////////////////////

module register_bank (destination_reg1_d, 
                      destination_reg2_d, 
                      destination_reg3_d, 
                      source_reg1_d, 
                      source_reg2_d, 
                      destination_reg_a, 
                      source_reg1_a, 
                      source_reg2_a, 
                      clk, 
                      load, 
                      store1,                        
                      store2);

output reg [31:0] source_reg1_d, source_reg2_d; 
input [31:0] destination_reg1_d, destination_reg2_d; 
input [15:0] destination_reg3_d;
input [2:0] destination_reg_a, source_reg1_a, source_reg2_a;
input clk, load, store1, store2; 
reg [31:0] regbank[7:0];  // 8 GPR each of size 32 bits                                             
reg [31:0] TEMP;
integer k;

always @ (posedge clk)
begin 
   case ({load,store1,store2}) 
   5'b0xx:  begin
           source_reg1_d <= source_reg1_d;   //  no change in wire A wire B
           source_reg2_d <= source_reg2_d;
           end 
   5'b1xx:  begin       
           source_reg1_d <= regbank[source_reg1_a];  //  simple store operation
           source_reg2_d <= regbank[source_reg2_a];
           end 
   5'b01x:  begin      
           regbank[destination_reg_a] <= destination_reg1_d;  // rd <= Z (simple load operation)
           end          
   5'b00x:  begin       
           regbank[destination_reg_a]     <= destination_reg1_d;  //rd <= Z (multiplication operation)
           regbank[destination_reg_a + 1] <= destination_reg2_d;  //rd + 1 <= acc
           end            
   5'b0x1:  begin
           TEMP = regbank[destination_reg_a];
           TEMP = {destination_reg3_d , TEMP[15:0]};       
           regbank[destination_reg_a] = TEMP;                    //rd <= S (MOVU operation)
           TEMP = 32'h00000000;
           end 
   5'b0x0:  begin  
           TEMP = regbank[destination_reg_a];
           TEMP = {TEMP[31:16] , destination_reg3_d};
           regbank[destination_reg_a] = TEMP;                    //rd <= S (MOVL operation)
           TEMP = 32'h00000000;
           end         
   default:begin end  
   endcase   
end
endmodule

///////////////////////////////////////////////////////////////////////////////

module control_signals(read_i,
                       read_r,
                       ALUon,
                       read_d,
                       mux_signal,
                       write_ra,
                       write_rb,                       
                       pc_signal,
                       I,
                       FR,
                       clk1,
                       start);

output reg read_i,read_r,ALUon,read_d,mux_signal,write_ra,write_rb,pc_signal;
input [31:0] I;
input clk1,start;
input [0:7] FR;
reg [2:0] state;


parameter ADD=7'b0000001,  SUB=7'b0000010,   AND=7'b0000011,   OR=7'b0000100,
          LT=7'b0000101,   GT=7'b0000110,    LOAD=7'b0000111,  STORE=7'b0001000, 
          END=7'b0001001,  RR=7'b0001010,    RL=7'b0001011,    JZ=7'b0001100,
          JNZ=7'b0001101,  JE=7'b0001110,    JNE=7'b0001111,   JMP=7'b0010000,
          MUL=7'b0010001,  DIV=7'b0010010,   MOD=7'b0010011,   NOT=7'b0010100,
          MOVR=7'b0010101, JFZ=7'b0010110,   JFNZ=7'b0010111,  MOVU=7'b0011000,
          MOVL=7'b0011001;
          
parameter SX=3'b000, S0=3'b001, S1=3'b010, S2=3'b011, S3=3'b100, S4=3'b101;         

initial begin #0 state <= SX; end
          
always @(posedge clk1)
begin
case(state)
SX:      if(start == 1)             state <= S0;
         else if(start == 0)        state <= SX;
         else                       state <= SX;
S0:      state <= S1;
S1:      state <= S2;
S2:      state <= S3;
S3:      state <= S4;
S4:      if ( I == 32'h12000000 ) state <= SX;
         else                     state <= S0;
default: state <= SX;
endcase

end



always @(state)
begin
case(state)

SX: begin  
    read_i = 0; 
    ALUon = 0; 
    read_d = 1'bx;
    mux_signal = 1'bx;
    write_ra = 1'bx; write_rb = 1'bx; read_r = 0;     
    pc_signal = 1'bx;                 
    end
    
S0: begin  
    read_i = 1; 
    ALUon = 0; 
    read_d = 1'bx;
    mux_signal = 1'bx;
    write_ra = 1'bx; write_rb = 1'bx; read_r = 0;
    pc_signal = 1'bx;                  
    end
    
S1: begin  
    read_i = 0; 
    ALUon = 0; 
    read_d = 1'bx;
    mux_signal = 1'bx;
    write_ra = 1'bx; write_rb = 1'bx; read_r = 1; 
    pc_signal = 1'bx;                   
    end

S2: begin  
    read_i = 0; 
    ALUon = 1;     
    if(I[31:25] == LOAD)       read_d = 1;
    else if(I[31:25] == STORE) read_d = 0;
    mux_signal = 1'bx;
    write_ra = 1'bx; write_rb = 1'bx; read_r = 0;
    pc_signal = 1'bx;                   
    end

S3: begin  
    read_i = 0; 
    ALUon = 0; 
    read_d = 1'bx;
    if(I[31:25] == LOAD) mux_signal = 1;
    else                 mux_signal = 0;
    write_ra = 1'bx; write_rb = 1'bx; read_r = 0; 
    pc_signal = 1'bx;                  
    end
    
S4: begin  
    read_i = 0; 
    ALUon = 0; 
    read_d = 1'bx;
    mux_signal = 1'bx;
    if(FR[2] == 0)      begin pc_signal = 1; end
    else if(FR[2] == 1) begin pc_signal = 0; end
    else                begin pc_signal = 1'bx; end   
    
    if(FR[1] == 1)      begin write_ra = 1'b1; write_rb = 1'bx; read_r = 0; end
    else if(FR[4] == 1) begin write_ra = 1'b0; write_rb = 1'bx; read_r = 0; end 
    else if(FR[5] == 1) begin write_ra = 1'bx; write_rb = 1'b1; read_r = 0; end
    else if(FR[6] == 1) begin write_ra = 1'bx; write_rb = 1'b0; read_r = 0; end                                         
    end

default: begin  
         read_i = 0; 
         ALUon = 0; 
         read_d = 1'bx;
         mux_signal = 1'bx;
         write_ra = 1'bx; write_rb = 1'bx; read_r = 0;
         pc_signal = 1'bx;                  
         end
         
endcase
end

endmodule   
