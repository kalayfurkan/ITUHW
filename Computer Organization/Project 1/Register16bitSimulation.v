`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Kadir Ozlem
// Project Name: BLG222E Project 1 Simulation
//////////////////////////////////////////////////////////////////////////////////

module Register16bitSimulation();
    reg[15:0] I;
    reg E;
    reg[1:0] FunSel;
    wire[15:0] Q;
    integer test_no;
    
    CrystalOscillator clk();
    Register16bit R(.I(I), .E(E), .FunSel(FunSel), .Clock(clk.clock), .Q(Q));
        
    FileOperation F();
    
    initial begin
        F.SimulationName ="Register16bit";
        F.InitializeSimulation(1);
        clk.clock = 0;
        
        //Test 1
        test_no = 1; 
        R.Q=16'h0025; FunSel=2'b00; I=16'h0072;  E=0; #5;
        clk.Clock();
        F.CheckValues(R.Q,16'h0025, test_no, "Q");
        
        //Test 2 
        test_no = 2;
        R.Q=16'h0025; FunSel=2'b00; E=1; #5;
        clk.Clock();
        F.CheckValues(R.Q,16'h0024, test_no, "Q"); 
        
        //Test 3 
        test_no = 3;
        R.Q=16'h0025; FunSel=2'b01; E=0; #5;
        clk.Clock();
        F.CheckValues(R.Q,16'h0025, test_no, "Q");
        
        //Test 4 
        test_no = 4;
        R.Q=16'h0025; FunSel=2'b01; E=1; #5;
        clk.Clock();
        F.CheckValues(R.Q,16'h0026, test_no, "Q"); 
        
        
        F.FinishSimulation();
    end
endmodule