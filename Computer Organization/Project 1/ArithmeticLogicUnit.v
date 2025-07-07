`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.03.2025 13:28:15
// Design Name: 
// Module Name: ArithmeticLogicUnit
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


module ArithmeticLogicUnit(
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [4:0] FunSel,
    input wire WF,
    input wire Clock,
    output reg [3:0] FlagsOut,
    output reg [31:0] ALUOut
);

reg Z;
reg C;
reg N;
reg O;

reg [31:0] tempResult;

reg [16:0] sum16bit;
reg [16:0] sum16bitCarried;
reg [15:0] sub16bit;

reg [32:0] sum32bit;
reg [32:0] sum32bitCarried;
reg [31:0] sub32bit;

always @(*) begin

    sum16bit={1'b0, A[15:0]} + {1'b0, B[15:0]};
    sum16bitCarried={1'b0, A[15:0]} + {1'b0, B[15:0]}+ {16'b0, FlagsOut[2]};
    sub16bit = A[15:0] - B[15:0];
    
    sum32bit = {1'b0, A} + {1'b0, B};
    sum32bitCarried = {1'b0, A} + {1'b0, B} + {32'b0, FlagsOut[2]};
    sub32bit = A - B;

    tempResult = 32'b0;
    
    case(FunSel)
        5'b00000:begin
            tempResult={16'b0,A[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
        end
        5'b00001:begin
            tempResult={16'b0,B[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
        end
        5'b00010:begin
            tempResult={16'b0, ~A[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
        end   
        5'b00011:begin
            tempResult={16'b0, ~B[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
        end
        5'b00100:begin
            tempResult={16'b0, sum16bit[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C=sum16bit[16];
            O=(A[15] == B[15]) && (sum16bit[15] != A[15]);
        end
        5'b00101:begin
            tempResult={16'b0,sum16bitCarried[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C=sum16bitCarried[16];
            O = (A[15] == B[15]) && (sum16bitCarried[15] != A[15]);
        end
        5'b00110:begin
            tempResult={16'b0, sub16bit[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C = (B[15:0] > A[15:0]);
            O = (A[15] == ~B[15]) && (sub16bit[15] != A[15]);
        end
        5'b00111:begin
            tempResult={16'b0, A[15:0] & B[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
        end
        5'b01000:begin
            tempResult={16'b0, A[15:0] | B[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15]; 
         end
         5'b01001:begin
            tempResult={16'b0, A[15:0] ^ B[15:0]};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
         end
         5'b01010:begin
            tempResult={16'b0, ~(A[15:0] & B[15:0])};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
         end
         5'b01011:begin
            tempResult={16'b0, A[15:0] << 1};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C=A[15];
         end
         5'b01100:begin
            tempResult={16'b0, A[15:0] >> 1};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C=A[0];
         end
         5'b01101:begin
            tempResult={16'b0, $signed(A[15:0]) >>> 1};
            Z=(tempResult[15:0] == 16'b0);
         end
         5'b01110:begin
            tempResult={16'b0, {A[14:0], A[15]}};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C=A[15];
         end
         5'b01111:begin
            tempResult={16'b0, {A[0], A[15:1]}};
            Z=(tempResult[15:0] == 16'b0);
            N = tempResult[15];
            C=A[0];
         end
         5'b10000:begin
            tempResult=A;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end          
         5'b10001:begin
            tempResult=B;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end          
         5'b10010:begin
            tempResult=~A;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end          
         5'b10011:begin
            tempResult=~B;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end
         5'b10100:begin
            tempResult=sum32bit[31:0];
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C = sum32bit[32];
            O = (A[31] == B[31]) && (sum32bit[31] != A[31]);
         end                   
         5'b10101:begin
            tempResult=sum32bitCarried[31:0];
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C = sum32bitCarried[32];
            O = (A[31] == B[31]) && (sum32bitCarried[31] != A[31]);
         end             
         5'b10110:begin
            tempResult=sub32bit;
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C = (B > A);
            O = (A[31] == ~B[31]) && (sub32bit[31] != A[31]);
         end            
         5'b10111:begin
            tempResult=A & B;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end  
         5'b11000:begin
            tempResult=A | B;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end 
         5'b11001:begin
            tempResult=A ^ B;
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end         
         5'b11010:begin
            tempResult=~(A & B);
            Z=(tempResult==32'b0);
            N = tempResult[31];
         end 
         5'b11011:begin
            tempResult=A << 1;
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C=A[31];
         end
         5'b11100:begin
            tempResult=A >> 1;
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C=A[0];
         end         
         5'b11101:begin
            tempResult=$signed(A) >>> 1;
            Z=(tempResult==32'b0);
         end        
         5'b11110:begin
            tempResult={A[30:0], A[31]};
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C = A[31];
         end 
         5'b11111:begin
            tempResult={A[0], A[31:1]};
            Z=(tempResult==32'b0);
            N = tempResult[31];
            C = A[0];
         end 
         default:;                                                 
    endcase
    ALUOut = tempResult;
end

always @(posedge Clock) begin
    
    if (WF) begin
        FlagsOut <= {Z, C, N, O};
    end
end


endmodule
