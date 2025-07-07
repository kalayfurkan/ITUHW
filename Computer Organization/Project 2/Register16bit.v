`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.03.2025 13:17:42
// Design Name: 
// Module Name: Register16bit
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


module Register16bit(
input wire Clock,
input wire [1:0] FunSel,
input wire E,
input wire [15:0] I,
output reg [15:0] Q
);

always @(posedge Clock)
begin
    if(E)
        begin
            case(FunSel)
                2'b00: Q <= Q-1;
                2'b01: Q<= Q+1;
                2'b10: Q<=I;
                2'b11: Q<=16'b0;
            endcase
        end
end

endmodule
