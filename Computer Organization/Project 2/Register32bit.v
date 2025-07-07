`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.03.2025 13:18:23
// Design Name: 
// Module Name: Register32bit
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


module Register32bit(
input wire Clock,
input wire E,
input wire [2:0] FunSel,
input wire [31:0] I,
output reg [31:0] Q
);

always @(posedge Clock) begin
    if (E) begin
        case (FunSel)
             3'b000: Q <= Q - 1;
             3'b001: Q <= Q + 1;
             3'b010: Q <= I;
             3'b011: Q <= 32'b0;
             3'b100: Q <= {24'b0, I[7:0]};
             3'b101: Q <= {16'b0, I[15:0]};
             3'b110: Q <= {Q[23:0], I[7:0]};
             3'b111: Q <= {{16{I[15]}}, I[15:0]};
         endcase
    end
end
endmodule
