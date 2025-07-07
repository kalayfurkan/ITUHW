`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Kadir Ozlem
// Project Name: BLG222E Project 1 Simulation
//////////////////////////////////////////////////////////////////////////////////

module DataRegisterSimulation();
    reg[7:0] I;
    reg E;
    reg[1:0] FunSel;
    wire[31:0] DROut;
    integer test_no;
    
    CrystalOscillator clk();
    DataRegister R(.I(I), .E(E), .FunSel(FunSel), .Clock(clk.clock), .DROut(DROut));
        
    FileOperation F();
    
    initial begin
        F.SimulationName ="DataRegister";
        F.InitializeSimulation(0);
        clk.clock = 0;
        
        //Test 1
        test_no = 1; 
        R.DROut=32'h00022018; FunSel=2'b00; I=8'h25;  E=0; #5;
        clk.Clock();
        F.CheckValues(R.DROut,32'h00022018, test_no, "DROut");
        
        //Test 2 
        test_no = 2;
        R.DROut=32'h12345678; FunSel=2'b00; I=8'h25; E=1; #5;
        clk.Clock();
        F.CheckValues(R.DROut,32'h00000025, test_no, "DROut"); 
        
        //Test 3 
        test_no = 3;
        R.DROut=32'h12345678; FunSel=2'b01; I=8'h25; E=0; #5;
        clk.Clock();
        F.CheckValues(R.DROut,32'h12345678, test_no, "DROut");
        
        //Test 4 
        test_no = 4;
        R.DROut=32'h12345678; FunSel=2'b01; I=8'h25; E=1; #5;
        clk.Clock();
        F.CheckValues(R.DROut,32'h00000025, test_no, "DROut"); 

        
        F.FinishSimulation();
    end
endmodule