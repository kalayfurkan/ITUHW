`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.03.2025 20:16:51
// Design Name: 
// Module Name: AddressRegisterFile
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


module AddressRegisterFile(
input wire [15:0] I,
input wire Clock,
input wire [1:0] FunSel,
input wire [2:0] RegSel,
input wire [1:0] OutCSel,
input wire [1:0] OutDSel,
output wire [15:0] OutC,
output wire [15:0] OutD
);

wire [15:0] PCo, ARo, SPo;

wire PCe=RegSel[2];
wire SPe=RegSel[1];
wire ARe=RegSel[0];

Register16bit PC(.I(I), .FunSel(FunSel), .E(PCe), .Clock(Clock), .Q(PCo));
Register16bit AR(.I(I), .FunSel(FunSel), .E(ARe), .Clock(Clock), .Q(ARo));
Register16bit SP(.I(I), .FunSel(FunSel), .E(SPe), .Clock(Clock), .Q(SPo));

reg [15:0] muxCo, muxDo;

always @(*) begin
    case(OutCSel)
        2'b00: muxCo = PCo;
        2'b01: muxCo = SPo;
        2'b10: muxCo = ARo;
        2'b11: muxCo = ARo;
    endcase
end
    
always @(*) begin
    case(OutDSel)
        2'b00: muxDo = PCo;
        2'b01: muxDo = SPo;
        2'b10: muxDo = ARo;
        2'b11: muxDo = ARo;
    endcase
end
    
assign OutC = muxCo;
assign OutD = muxDo;
endmodule
