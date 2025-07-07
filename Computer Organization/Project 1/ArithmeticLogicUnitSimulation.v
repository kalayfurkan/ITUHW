`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Kadir Ozlem
// Project Name: BLG222E Project 1 Simulation
//////////////////////////////////////////////////////////////////////////////////

module ArithmeticLogicUnitSimulation();
    reg[31:0] A, B;
    reg[4:0] FunSel;
    reg WF;
    wire[31:0] ALUOut;
    wire[3:0] FlagsOut;
    integer test_no;
    wire Z, C, N, O;
    CrystalOscillator clk();
    ArithmeticLogicUnit ALU( .A(A), .B(B), .FunSel(FunSel), .WF(WF), 
                            .Clock(clk.clock), .ALUOut(ALUOut), .FlagsOut(FlagsOut));
        
    FileOperation F();
    
    assign {Z,C,N,O} = FlagsOut;
    
    initial begin
        F.SimulationName ="ArithmeticLogicUnit";
        F.InitializeSimulation(0);
        clk.clock = 0;
        
        //Test 1
        test_no = 1;
        A = 32'h12341234;
        B = 32'h43214321;
        ALU.FlagsOut = 4'b1111;
        FunSel =5'b10100;
        WF =1;
        #5
        F.CheckValues(ALUOut,32'h55555555, test_no, "ALUOut");
        F.CheckValues(Z,1, test_no, "Z");
        F.CheckValues(C,1, test_no, "C");
        F.CheckValues(N,1, test_no, "N");
        F.CheckValues(O,1, test_no, "O");
        
        //Test 2
        test_no = 2;
        clk.Clock();
        F.CheckValues(ALUOut,32'h55555555, test_no, "ALUOut");
        F.CheckValues(Z,0, test_no, "Z");
        F.CheckValues(C,0, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");

        //Test 3 
        test_no = 3;
        A = 32'h77777777;
        B = 32'h88888888; //16'b8889;
        ALU.FlagsOut = 4'b0100; //4'b0100;
        FunSel =5'b10101;
        WF =1;
        #5
        clk.Clock();
        F.CheckValues(ALUOut,32'h00000000, test_no, "ALUOut");
        F.CheckValues(Z,1, test_no, "Z");
        F.CheckValues(C,1, test_no, "C");
        F.CheckValues(N,0, test_no, "N");
        F.CheckValues(O,0, test_no, "O");
         
         
        F.FinishSimulation();
    end
endmodule