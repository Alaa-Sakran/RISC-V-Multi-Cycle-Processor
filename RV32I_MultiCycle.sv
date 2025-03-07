module RV32I_MultiCycle(input clk,reset);

logic [31:0] PC,ALUOUT,MUX2OUTPUT,PCNEXT,Instruction;
logic [3:0] ALUOP;
logic [2:0] EXTDEC_CODE;
logic WEM,M1,WER,zero,LessThan,LUI,EN1,EN2,M5,BEQ,BLT,M2;
logic [1:0] M3,M4;

datapath dp(LUI,clk,reset,Instruction,PC,ALUOUT,MUX2OUTPUT,PCNEXT,LessThan,zero,
M2,M3, WEM,M1,WER,EXTDEC_CODE,ALUOP, EN1,EN2,BEQ,BLT,M5,M4); 

controller control_unit(clk,reset,LUI,Instruction,M2,M3,WEM,M1,WER,EXTDEC_CODE,zero,LessThan,ALUOP, M4, EN1,EN2,M5,BEQ,BLT);

endmodule 

module controller(input logic clk,reset,output logic LUI, input logic [31:0] Instruction,output logic M2, output logic [1:0] M3, output logic WEM,M1,WER,output logic [2:0] EXTDEC_CODE,
input logic zero,LessThan, output logic [3:0] ALUOP, output logic [1:0] M4, output logic EN1,EN2,M5,BEQ,BLT);

logic [2:0] ALUOPDEC;
logic [3:0] current_state;
main_dec U1(clk,reset,LUI,Instruction[6:0],M2,WER,WEM,M1,J,EXTDEC_CODE,ALUOPDEC,current_state,EN1,EN2,M5,M3,M4);
ALUDEC aludec(current_state,ALUOPDEC,Instruction[14:12], Instruction[31:25],ALUOP,BEQ,BLT);

endmodule

module datapath(input logic LUI,input logic clk,reset, output logic [31:0] Instruction, output logic [31:0] PC,ALUOUT,MUX2OUTPUT,PCNEXT, output logic LessThan,zero,
input logic M2,input logic [1:0] M3,
input logic WEM,M1,WER, input logic [2:0] EXTDEC_CODE, input logic [3:0] ALUOP,input logic EN1,EN2,BEQ,BLT,M5 ,input logic [1:0] M4);

logic [31:0] RDA1,RDA2,MUX1OUTPUT,EXTOUT,RDB1,MUX3OUTPUT,MUX4OUTPUT,MUX5OUTPUT,F3_D,F4_1_D, F4_2_D ,F5_D;
logic ENF1;
assign ENF1=EN1|(BEQ&zero)|(BLT&LessThan);
REGFILE regfile(LUI,Instruction[19:15], Instruction[24:20], Instruction[11:7], WER,clk,reset,MUX5OUTPUT,RDA1,RDA2);
Extendor E1(EXTDEC_CODE,Instruction[31:7],EXTOUT);


ALU alu(MUX2OUTPUT,MUX3OUTPUT,ALUOP,ALUOUT,zero,LessThan);


data_memory DM(MUX1OUTPUT,RDA2,WEM,clk,RDB1);

MUX2TO1 #(32) MUX1(PC,F5_D,M1,MUX1OUTPUT);
MUX2TO1 #(32) MUX2(PC,F4_1_D,M2,MUX2OUTPUT);
MUX3TO1 #(32) MUX3(F4_2_D,32'd4,EXTOUT,M3,MUX3OUTPUT);
MUX4TO1 #(32) MUX4(F5_D,F3_D,PC,EXTOUT,M4,MUX4OUTPUT);
MUX2TO1 #(32) MUX5(MUX4OUTPUT,PC,M5,MUX5OUTPUT);
Enabled_FF #(32) F1(clk,reset,ENF1,MUX4OUTPUT,PC);
Enabled_FF #(32) F2(clk,reset,EN2,RDB1,Instruction);
flop #(32) F3(clk,reset,RDB1,F3_D);
flop #(32) F4(clk,reset,RDA1,F4_1_D);
flop #(32) F4_2(clk,reset,RDA2,F4_2_D);
flop #(32) F5(clk,reset,ALUOUT,F5_D);




endmodule

module ALU( input logic [31:0] operand1,operand2, input logic [3:0] ALUOP, output logic [31:0] result, output logic zero,LessThan);
always_comb //operand1 is RDA1 while operand2 is the output of MUX2, result is also connected to both data memory and MUX3
begin
case(ALUOP)
4'b0000: result=operand1+operand2;
4'b0001: result=operand1-operand2;
4'b0010: result=operand1&operand2;
4'b0011: result=operand1|operand2;
4'b0100: result=operand1^operand2;
4'b0101: result=operand1>>operand2;
4'b0110: result=operand1<<operand2;
4'b0111: result=operand1>>>operand2;
4'b1000: result=(operand1<operand2)?32'd1:32'd0;
default: result=result;
endcase
end
assign zero=(result==32'b0);
assign LessThan=result[31];
endmodule 


module Extendor(input logic [2:0] EXT_CODE, input logic [24:0]IMMIN, output logic [31:0]IMMOUT); //extendor will have [31:7]
always_comb
begin
case(EXT_CODE)
3'b000: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24:13]}; //lw,immediate
3'b001: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24:18], IMMIN[4:0]};//s-type
3'b010: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24], IMMIN[0], IMMIN[23:18],IMMIN[4:1],1'b0};//branch
3'b011: IMMOUT = {{11{IMMIN[24]}}, IMMIN[24], IMMIN[12:5], IMMIN[13], IMMIN[23:14], 1'b0 }; //jump
3'b111: IMMOUT = {IMMIN[24:5],{12{1'bx}}};//Lui
default: IMMOUT=IMMOUT;
endcase

end

endmodule

module flop #(parameter WIDTH=8) (input logic clk, reset, input logic [WIDTH-1:0] D, output logic [WIDTH-1:0] Q);

always_ff @(posedge clk, negedge reset)
begin
if(!reset)
Q<=0;
else
Q<=D;
end
endmodule

module MUX2TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2, input logic select, output logic [WIDTH-1:0] MUXOUT);
assign MUXOUT = select?MUXIN2:MUXIN1;
endmodule

module MUX4TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2,MUXIN3,MUXIN4 , input logic [1:0] select, output logic [WIDTH-1:0] MUXOUT);
always_comb
begin
case(select)
2'b00: MUXOUT=MUXIN1;
2'b01: MUXOUT=MUXIN2;
2'b10: MUXOUT=MUXIN3;
2'b11: MUXOUT=MUXIN4;
default: MUXOUT=MUXOUT;
endcase
end
endmodule
module MUX3TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2,MUXIN3, input logic [1:0] select, output logic [WIDTH-1:0] MUXOUT);
always_comb
begin
case(select)
2'b00: MUXOUT=MUXIN1;
2'b01: MUXOUT=MUXIN2;
2'b10: MUXOUT=MUXIN3;
default: MUXOUT=MUXOUT;
endcase
end
endmodule


module REGFILE (input logic LUI,input logic [4:0] A1,A2,A3, input logic WER,clk,reset, input logic [31:0] WDR, output logic [31:0]RDA1, RDA2);
logic [31:0] REGS[31:0];


always_ff @(posedge clk, negedge reset)
begin
if(!reset) for (int i=0; i<32; i++)begin REGS[i]<=32'b0;  end
if(WER) if(A3!=0)begin if(LUI)begin REGS[A3][31:12]<=WDR[31:12];  end else REGS[A3]<=WDR; end
end
always_comb
begin
RDA1=REGS[A1];
RDA2=REGS[A2];
end
endmodule

module main_dec(input logic clk,reset, output logic LUI, input logic [6:0] OPCODE, output logic  M2, output logic WER,WEM,M1,J, output logic [2:0]EXT_CODE, output logic [2:0] ALUOPDEC,
output logic [3:0] current_state,output logic EN1,EN2,M5, output logic [1:0] M3,M4);
localparam [3:0] 
S0_Fetch=4'd0, S1_Decode=4'd1, S2_LWSW_ALU=4'd2, S3_LW_readData=4'd3, S4_LW_savedata=4'd4, S5_SW_savedata=4'd5, S6_R_ALU=4'd6,
S7_R_I_savedata=4'd7, S8_I_JALR_ALU=4'd8, S9_BRANCH=4'd9, S10_JALR_JAL=4'd10, S11_LUI=4'd11;
logic [3:0] next_state,intermediate_state;
always_ff@(posedge clk, negedge reset)
begin
if(!reset) current_state<=S0_Fetch;
else
	begin
	current_state<=next_state;
	end

end
always_comb
begin
case(current_state)
S0_Fetch:begin M1=1'b0; EN2=1'b1;M2=1'b0; M3=2'b01; next_state=S1_Decode; WER=0; WEM=0; EN1=0;  end
S1_Decode:begin M4=2'b00; EN1=1'b1;M2=1'b0; M3=2'b10; next_state=intermediate_state; WER=0; WEM=0; EN2=0;  end
S2_LWSW_ALU:begin  M2=1'b1; M3=2'b10; WER=0; WEM=0; EN1=0; EN2=0; if(intermediate_state==S5_SW_savedata)next_state=intermediate_state; else next_state=S3_LW_readData;   end
S3_LW_readData:begin M4=2'b01; M1=1'b1; WER=0; WEM=0; EN1=0; EN2=0; next_state=S4_LW_savedata;   end
S4_LW_savedata:begin M4=2'b01; WER=1'b1; LUI=0; M5=1'b0; WEM=0; EN1=0; EN2=0; next_state=S0_Fetch;   end
S5_SW_savedata:begin WEM=1;  M1=1'b1; WER=0; EN1=0; EN2=0; next_state=S0_Fetch;   end
S6_R_ALU:begin M3=2'B00;  M2=1'b1; WER=0; WEM=0; EN1=0; EN2=0; next_state=S7_R_I_savedata;   end
S7_R_I_savedata:begin M4=2'B00;  M5=1'b0; WER=1; LUI=0; WEM=0; EN1=0; EN2=0; next_state=S0_Fetch;   end
S8_I_JALR_ALU:begin M3=2'B10;  M2=1'b1; WER=0; WEM=0; EN1=0; EN2=0; if(OPCODE[6]==1)next_state=S10_JALR_JAL; else next_state=S7_R_I_savedata;   end
S9_BRANCH:begin M4=2'B00;M2=1'b1; M3=2'B00;   WER=0; WEM=0; EN1=0; EN2=0; next_state=S0_Fetch;   end
S10_JALR_JAL:begin M4=2'B00;   M5=1; WER=1; LUI=0; WEM=0; EN1=1; EN2=0; next_state=S0_Fetch;   end
S11_LUI:begin M4=2'B11;   LUI=1; M5=0;  WER=1; WEM=0; EN1=0; EN2=0; next_state=S0_Fetch;   end
default:begin WER=0; WEM=0; EN1=0; EN2=0; end
endcase
end

always_comb
begin
case(OPCODE)
7'b0000011:begin  ALUOPDEC=3'b010; EXT_CODE=3'b000; intermediate_state=S2_LWSW_ALU;  end //load
7'b0010011:begin  ALUOPDEC=3'b000; EXT_CODE=3'b000; intermediate_state=S8_I_JALR_ALU; end //addi..
7'b1100111:begin  ALUOPDEC=3'b000; EXT_CODE=3'b000;  intermediate_state=S8_I_JALR_ALU; end //jalr
7'b0100011:begin  ALUOPDEC=3'b010; EXT_CODE=3'b001;  intermediate_state=S5_SW_savedata; end //s-type
7'b0110011:begin  ALUOPDEC=3'b100; intermediate_state=S6_R_ALU;   end //R-type
7'b0110111:begin  EXT_CODE=3'b111; intermediate_state=S11_LUI;  end //U-type
7'b1100011:begin  ALUOPDEC=3'b110; EXT_CODE=3'b010; intermediate_state=S9_BRANCH;  end //B-type
7'b1101111:begin  EXT_CODE=3'b011; intermediate_state=S10_JALR_JAL;  end //J
default:  begin  intermediate_state=4'bx; end //make sure if a false OPCODE was inserted no writing to existing data occurs
endcase 
end
endmodule

module ALUDEC(input logic [3:0] current_state,input logic [2:0] ALUOPDEC, input logic [2:0] FUNCT3,input logic [6:0] FUNCT7 , 
output logic [3:0] ALUOP, output logic BEQ,BLT);
always_comb
begin
case(current_state)
4'd0:begin ALUOP=4'b0000; BEQ=0; BLT=0; end
4'd1:begin ALUOP=4'b0000; BEQ=0; BLT=0; end
default: begin
case(ALUOPDEC)
3'b000:begin  BEQ=0; BLT=0;
	case(FUNCT3)
	3'b000:begin ALUOP=4'b0000;  end  //addi
	3'b010:begin ALUOP=4'b1000; end //slti
	3'b100:begin ALUOP=4'b0100; end //xori
	3'b110:begin ALUOP=4'b0011; end //ori
	3'b111:begin ALUOP=4'b0010; end //andi
	3'b001:begin ALUOP=4'b0110; end //slli
	3'b101:begin if(FUNCT7==7'd0) begin ALUOP=4'b0101;  end if(FUNCT7==7'b0100000) begin ALUOP=4'b0111;  end end //srl, sra
	default: begin ALUOP=4'bxxxx;  end //ALUOP could be anything it won't matter as long as write controls are LOW
	endcase   end

3'b100:begin BEQ=0; BLT=0;  if(FUNCT7==7'd0) begin
	case(FUNCT3)
	3'b000:begin ALUOP=4'b0000; end  //add
	3'b010:begin ALUOP=4'b1000;  end //slt
	3'b100:begin ALUOP=4'b0100;  end //xor
	3'b110:begin ALUOP=4'b0011;  end //or
	3'b111:begin ALUOP=4'b0010;  end //and
	3'b001:begin ALUOP=4'b0110;  end //sll
	3'b101:begin  ALUOP=4'b0101;  end 
	default: begin ALUOP=4'bxxxx;  end
	endcase   end
   
if(FUNCT7==7'b0100000) begin  case(FUNCT3) 
	 3'b000:begin ALUOP=4'b0001;  end //sub
	 3'b101:begin ALUOP=4'b0111;  end //sra
	 default: begin ALUOP=4'bxxxx; end
	 endcase end
end
3'b110:begin 
case(FUNCT3)
3'b000: begin ALUOP=4'b0001; BEQ=1; BLT=0; end //beq
3'b100: begin ALUOP=4'b0001;  BLT=1; BEQ=0; end //blt
default:begin ALUOP=4'bxxxx; BEQ=0; BLT=0; end
endcase

end
3'b010:begin BEQ=0; BLT=0; if(FUNCT3==3'b010)begin ALUOP=4'b0000;  end end //lw,sw
default: begin ALUOP=4'bxxxx;  end
endcase
end
endcase
end
endmodule

module data_memory(input logic [31:0] B1,WDM,input logic WEM,clk, output logic [31:0] RDB1);
logic [31:0] RAM [63:0];

always_comb
begin
RDB1=RAM[B1[31:2]];
end
always_ff@(posedge clk)
begin
if(WEM) RAM[B1[31:2]]<=WDM;
end
endmodule


module Enabled_FF #(parameter WIDTH=8)(input logic clk,reset,enable, input logic [WIDTH-1:0]Q, output logic [WIDTH-1:0]D);
always_ff@(posedge clk,negedge reset)
begin
if(!reset) D<=0;
else
	begin
	if(enable) D<=Q;
	end
end

endmodule

module test_multi();
logic clk,reset;
RV32I_MultiCycle TEST(clk,reset);
initial begin $readmemh("instruction_memory.txt", TEST.dp.DM.RAM);   end
initial begin reset<=1; #7 reset<=0; #5 reset<=1; end
initial
begin
#12
clk<=0;
forever begin #5 clk<=~clk; end
end
initial begin
$monitor("Time is %d  clk is %b current instruction is %h, \n REGs from 0 to 18 are: x00 is %h x01 is %h x02 is %h x03 is %h  \n x04 is %h x05 is %h x06 is %h x07 is %h x08 is %h \n x09 is %h x10 is %h x11 is %h x12 is %h \n x13 is %h x14 is %h x15 is %h \n x16 is %h x17 is %h x18 is %h PC %d state %d int %d instr %h B1 %d RDB1 %d %h",$time,
clk,
TEST.Instruction,TEST.dp.regfile.REGS[0],TEST.dp.regfile.REGS[1],TEST.dp.regfile.REGS[2],
TEST.dp.regfile.REGS[3],TEST.dp.regfile.REGS[4],TEST.dp.regfile.REGS[5],TEST.dp.regfile.REGS[6],
TEST.dp.regfile.REGS[7],TEST.dp.regfile.REGS[8],TEST.dp.regfile.REGS[9],TEST.dp.regfile.REGS[10],
TEST.dp.regfile.REGS[11],TEST.dp.regfile.REGS[12],TEST.dp.regfile.REGS[13],TEST.dp.regfile.REGS[14],
TEST.dp.regfile.REGS[15],TEST.dp.regfile.REGS[16],TEST.dp.regfile.REGS[17],TEST.dp.regfile.REGS[18],TEST.PC,TEST.control_unit.current_state
,TEST.control_unit.U1.intermediate_state,TEST.Instruction,TEST.dp.DM.B1,TEST.dp.DM.RDB1,TEST.dp.DM.RAM[1]
);
end
endmodule