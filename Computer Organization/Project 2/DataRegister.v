`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.03.2025 15:25:36
// Design Name: 
// Module Name: DataRegister
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


module DataRegister(
output reg [31:0] DROut,
input wire [7:0] I,
input wire Clock,
input wire E,
input wire [1:0] FunSel
);

always@(posedge Clock) begin
    if(E) begin
        case(FunSel)
            2'b00: DROut<={{24{I[7]}}, I};
            2'b01: DROut<={24'b0, I};
            2'b10: DROut<= {DROut[23:0], I};
            2'b11: DROut<= {I, DROut[31:8]};
        endcase
     end
end
        
endmodule
