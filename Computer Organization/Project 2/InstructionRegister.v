`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.03.2025 15:12:42
// Design Name: 
// Module Name: InstructionRegister
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


module InstructionRegister (
    input wire Clock,
    input wire LH,
    input wire Write,
    input wire [7:0] I,
    output reg [15:0] IROut
);

always @(posedge Clock) begin
    if (Write) begin
        if (LH == 1'b0)
            IROut[7:0] <= I;
        else
            IROut[15:8] <= I;
    end
end

endmodule