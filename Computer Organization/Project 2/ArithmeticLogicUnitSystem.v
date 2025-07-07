`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.04.2025 15:49:13
// Design Name: 
// Module Name: ArithmeticLogicUnitSystem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module ArithmeticLogicUnitSystem(
input wire Clock,

input  wire [1:0]  MuxASel,
input  wire [1:0]  MuxBSel,
input  wire [1:0]  MuxCSel,
input  wire        MuxDSel,

input wire [2:0] RF_OutASel,
input wire [2:0] RF_OutBSel,
input wire [2:0] RF_FunSel,
input wire [3:0] RF_RegSel,
input wire [3:0] RF_ScrSel,

input wire [4:0] ALU_FunSel,
input wire ALU_WF,
  
input wire [1:0] ARF_OutCSel,
input wire [1:0] ARF_OutDSel,
input wire [1:0] ARF_FunSel,
input wire [2:0] ARF_RegSel,
  
input wire IR_LH,
input wire IR_Write,
input wire Mem_WR, Mem_CS,

input wire [1:0] DR_FunSel,
input wire DR_E
);

wire[31:0] DROut;
wire [7:0] MemOut;

wire [31:0] MuxAOut;
wire [31:0] MuxBOut;
wire [7:0] MuxCOut;
wire [31:0] MuxDOut;

wire [31:0] OutA;
wire [31:0] OutB;
wire [15:0] OutC;
wire [15:0] Address;

wire[31:0] ALUOut;
wire Z,C,N,O;




wire [15:0] IROut;

AddressRegisterFile ARF(.I(MuxBOut),.Clock(Clock),.FunSel(ARF_FunSel),.RegSel(ARF_RegSel),.OutCSel(ARF_OutCSel),.OutDSel(ARF_OutDSel),.OutC(OutC),.OutD(Address));
RegisterFile RF(.I(MuxAOut) , .OutASel(RF_OutASel) , .OutBSel(RF_OutBSel) , .RegSel(RF_RegSel) , .ScrSel(RF_ScrSel) , .FunSel(RF_FunSel) , .Clock(Clock) , .OutA(OutA) ,.OutB(OutB) );
ArithmeticLogicUnit ALU(.A(MuxDOut), .B(OutB), .FunSel(ALU_FunSel), .WF(ALU_WF),.Clock(Clock) , .FlagsOut({Z,C,N,O}), .ALUOut(ALUOut) );
Memory MEM(.Address(Address), .Data(MuxCOut), .WR(Mem_WR), .CS(Mem_CS), .Clock(Clock), .MemOut(MemOut) );
DataRegister DR(.I(MemOut), .Clock(Clock), .E(DR_E), .FunSel(DR_FunSel), .DROut(DROut));
InstructionRegister IR(.Clock(Clock), .LH(IR_LH), .Write(IR_Write), .I(MemOut), .IROut(IROut)  );

assign MuxAOut = (MuxASel == 2'b00) ? ALUOut :
                     (MuxASel == 2'b01) ? {16'b0, OutC} :
                     (MuxASel == 2'b10) ? DROut :
                     (MuxASel == 2'b11) ? {24'b0, IROut[7:0]} :32'h0;
                     
assign MuxBOut = (MuxBSel == 2'b00) ? ALUOut :
            (MuxBSel == 2'b01) ? {16'b0, OutC} :
            (MuxBSel == 2'b10) ? DROut :
            (MuxBSel == 2'b11) ? {24'b0, IROut[7:0]} :32'h0;
assign MuxCOut = (MuxCSel == 2'b00) ? ALUOut[7:0]  :
            (MuxCSel == 2'b01) ? ALUOut[15:8] :
            (MuxCSel == 2'b10) ? ALUOut[23:16] :
            (MuxCSel == 2'b11) ? ALUOut[31:24] :8'h0;
assign MuxDOut = (MuxDSel == 1'b0) ? OutA : {16'b0, OutC};


endmodule
