`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Kadir Ozlem
// Project Name: BLG222E Project 1 Simulation
//////////////////////////////////////////////////////////////////////////////////

module Register32bitSimulation();
    reg[31:0] I;
    reg E;
    reg[2:0] FunSel;
    wire[31:0] Q;
    integer test_no;
    
    CrystalOscillator clk();
    Register32bit R(.I(I), .E(E), .FunSel(FunSel), .Clock(clk.clock), .Q(Q));
        
    FileOperation F();
    
    initial begin
        F.SimulationName ="Register32bit";
        F.InitializeSimulation(0);
        clk.clock = 0;
        
        //Test 1
        test_no = 1; 
        R.Q=32'h00001234; FunSel=3'b000; I=32'h00005678;  E=0; #5;
        clk.Clock();
        F.CheckValues(R.Q,32'h00001234, test_no, "Q");
        
        //Test 2 
        test_no = 2;
        R.Q=32'h00001234; FunSel=3'b000; E=1; #5;
        clk.Clock();
        F.CheckValues(R.Q,32'h00001233, test_no, "Q"); 
        
        //Test 3 
        test_no = 3;
        R.Q=32'h00001234; FunSel=3'b001; E=0; #5;
        clk.Clock();
        F.CheckValues(R.Q,32'h00001234, test_no, "Q");
        
        //Test 4 
        test_no = 4;
        R.Q=32'h00001234; FunSel=3'b001; E=1; #5;
        clk.Clock();
        F.CheckValues(R.Q,32'h00001235, test_no, "Q"); 
        
        F.FinishSimulation();
    end
endmodule