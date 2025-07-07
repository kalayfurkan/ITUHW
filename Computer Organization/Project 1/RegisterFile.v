`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.03.2025 15:48:41
// Design Name: 
// Module Name: RegisterFile
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


module RegisterFile(
input wire [31:0] I,
input wire [2:0] OutASel, OutBSel,
input wire [3:0] RegSel, ScrSel,
input wire [2:0] FunSel,
input wire Clock,
output wire [31:0] OutA, OutB
);

wire [31:0] R1o, R2o, R3o, R4o;
wire [31:0] S1o, S2o, S3o, S4o;

wire R1e = RegSel[3];
wire R2e = RegSel[2];
wire R3e = RegSel[1];
wire R4e = RegSel[0];

wire S1e = ScrSel[3];
wire S2e = ScrSel[2];
wire S3e = ScrSel[1];
wire S4e = ScrSel[0];

Register32bit R1(.I(I), .FunSel(FunSel), .E(R1e), .Clock(Clock), .Q(R1o));
Register32bit R2(.I(I), .FunSel(FunSel), .E(R2e), .Clock(Clock), .Q(R2o));
Register32bit R3(.I(I), .FunSel(FunSel), .E(R3e), .Clock(Clock), .Q(R3o));
Register32bit R4(.I(I), .FunSel(FunSel), .E(R4e), .Clock(Clock), .Q(R4o));
Register32bit S1(.I(I), .FunSel(FunSel), .E(S1e), .Clock(Clock), .Q(S1o));
Register32bit S2(.I(I), .FunSel(FunSel), .E(S2e), .Clock(Clock), .Q(S2o));
Register32bit S3(.I(I), .FunSel(FunSel), .E(S3e), .Clock(Clock), .Q(S3o));
Register32bit S4(.I(I), .FunSel(FunSel), .E(S4e), .Clock(Clock), .Q(S4o));

reg [31:0] muxAo, muxBo;    
always @(*) begin
    case(OutASel)
        3'b000: muxAo = R1o;
        3'b001: muxAo = R2o;
        3'b010: muxAo = R3o;
        3'b011: muxAo = R4o;
        3'b100: muxAo = S1o;
        3'b101: muxAo = S2o;
        3'b110: muxAo = S3o;
        3'b111: muxAo = S4o;
        endcase
    end
    
    always @(*) begin
        case(OutBSel)
            3'b000: muxBo = R1o;
            3'b001: muxBo = R2o;
            3'b010: muxBo = R3o;
            3'b011: muxBo = R4o;
            3'b100: muxBo = S1o;
            3'b101: muxBo = S2o;
            3'b110: muxBo = S3o;
            3'b111: muxBo = S4o;
        endcase
    end
    
    assign OutA = muxAo;
    assign OutB = muxBo;
 
endmodule
