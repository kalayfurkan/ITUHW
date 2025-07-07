`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Omur F Erzurumluoglu
// Project Name: BLG222E Project 2 Simulation
//////////////////////////////////////////////////////////////////////////////////


module CPUSystemSimulation();
    wire [11:0] T;
    integer test_no;
    wire clock;
    wire reset;
   
    wire[5:0] Opcode;
    wire[1:0] RegSel;
    wire[7:0] Address;
    wire[2:0] DestReg;
    wire[2:0] SrcReg1;
    wire[2:0] SrcReg2;

    CrystalOscillator clk();
    ResetGenerator rg();

    CPUSystem CPUSys(
        .Clock(clk.clock),
        .Reset(rg.reset),
        .T(T) 
    ); 
    FileOperation F();
    
    assign clock = clk.clock;
    assign reset = rg.reset;
    
    task ClearRegisters;
        begin
            CPUSys.ALUSys.RF.R1.Q = 32'h0;
            CPUSys.ALUSys.RF.R2.Q = 32'h0;
            CPUSys.ALUSys.RF.R3.Q = 32'h0;
            CPUSys.ALUSys.RF.R4.Q = 32'h0;
            CPUSys.ALUSys.RF.S1.Q = 32'h0;
            CPUSys.ALUSys.RF.S2.Q = 32'h0;
            CPUSys.ALUSys.RF.S3.Q = 32'h0;
            CPUSys.ALUSys.RF.S4.Q = 32'h0;
            CPUSys.ALUSys.ARF.PC.Q = 16'h0;
            CPUSys.ALUSys.ARF.AR.Q = 16'h0;
            CPUSys.ALUSys.ARF.SP.Q = 16'h00FF;
            CPUSys.ALUSys.IR.IROut = 16'h0;
            CPUSys.ALUSys.DR.DROut = 32'h0;
            CPUSys.ALUSys.ALU.FlagsOut = 4'b0000;
        end
    endtask
        
    task SetRegisters16;
        input [15:0] value;
        begin
            CPUSys.ALUSys.ARF.PC.Q = value;
            CPUSys.ALUSys.ARF.AR.Q = value;
            CPUSys.ALUSys.ARF.SP.Q = value;
            CPUSys.ALUSys.IR.IROut = value;
        end
    endtask

    task SetRegisters32;
        input [31:0] value;
        begin
            CPUSys.ALUSys.RF.R1.Q = value;
            CPUSys.ALUSys.RF.R2.Q = value;
            CPUSys.ALUSys.RF.R3.Q = value;
            CPUSys.ALUSys.RF.R4.Q = value;
            CPUSys.ALUSys.RF.S1.Q = value;
            CPUSys.ALUSys.RF.S2.Q = value;
            CPUSys.ALUSys.RF.S3.Q = value;
            CPUSys.ALUSys.RF.S4.Q = value;
            CPUSys.ALUSys.DR.DROut = value;
        end
    endtask

    task DisableAll;
        begin
            CPUSys.RF_RegSel = 4'b0000;
            CPUSys.RF_ScrSel = 4'b0000;
            CPUSys.ARF_RegSel = 3'b000;
            CPUSys.IR_Write = 0;
            CPUSys.ALU_WF = 0;
            CPUSys.Mem_CS = 1;
            CPUSys.Mem_WR = 0;
            CPUSys.DR_E = 0;
            CPUSys.T_Reset = 1; // T_Reset is used to reset T of CPUSys
        end
    endtask

    task ResetT;
        begin
            CPUSys.T_Reset = 1;
        end
    endtask
    
    assign Opcode = CPUSys.Opcode;
    assign RegSel = CPUSys.RegSel;
    assign Address = CPUSys.Address;
    assign DestReg = CPUSys.DestReg;
    assign SrcReg1 = CPUSys.SrcReg1;
    assign SrcReg2 = CPUSys.SrcReg2;
    
    initial begin
        F.SimulationName ="CPUSystem";
        F.InitializeSimulation(0);
        clk.clock = 0;
        
        //Test 1
        test_no = 1;
        DisableAll();
        ClearRegisters();
        
        SetRegisters32(32'h77777777);
        F.CheckValues(CPUSys.ALUSys.RF.R2.Q, 32'h77777777, test_no, "R2");
        rg.ActivateReset();
        clk.Clock();
        rg.DeactivateReset();
        F.CheckValues(CPUSys.ALUSys.RF.R2.Q, 32'h00000000, test_no, "R2");
        CPUSys.ALUSys.ARF.PC.Q = 16'h0056;

        //Test 2 MOVL R1, 0x01
        test_no = 2;
        CPUSys.ALUSys.IR.IROut = 16'h6401;
        CPUSys.T = 12'b0000_0000_0100;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.RF.R1.Q, 32'h00000001, test_no, "R1");
        F.CheckValues(CPUSys.ALUSys.ALU.FlagsOut[3], 0, test_no, "Z");

        //Test 3 DEC R1, R2
        test_no = 3;
        CPUSys.ALUSys.IR.IROut = 16'h2AC0;
        CPUSys.T = 12'b0000_0000_0100;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.RF.R1.Q, 32'h00000001, test_no, "R1");
        F.CheckValues(CPUSys.ALUSys.RF.R2.Q, 32'h00000000, test_no, "R2");
        F.CheckValues(CPUSys.ALUSys.ALU.FlagsOut[3], 1, test_no, "Z");

        //Test 4 LDAL R3, 0x00
        test_no = 4;
        CPUSys.ALUSys.IR.IROut = 16'h7A00;
        CPUSys.T = 12'b0000_0000_0100;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.DR.DROut, 32'h00000600, test_no, "DROut");
        F.CheckValues(CPUSys.ALUSys.RF.R3.Q, 32'h00000600, test_no, "R3");
        F.CheckValues(CPUSys.ALUSys.ARF.AR.Q, 16'h0001, test_no, "AR");

        //Test 5 LDAH R3, 0x00
        test_no = 5;
        CPUSys.ALUSys.IR.IROut = 16'h7E00;
        CPUSys.T = 12'b0000_0000_0100;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.DR.DROut, 32'h06000a08, test_no, "DROut");
        F.CheckValues(CPUSys.ALUSys.RF.R3.Q, 32'h06000a08, test_no, "R3");
        F.CheckValues(CPUSys.ALUSys.ARF.AR.Q, 16'h0003, test_no, "AR");

        //Test 6 STAR R3
        test_no = 6;
        CPUSys.ALUSys.IR.IROut = 16'h7460;
        CPUSys.T = 12'b0000_0000_0100;
        CPUSys.ALUSys.ARF.AR.Q = 16'h0008;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.RF.R3.Q, 32'h06000a08, test_no, "R3");
        F.CheckValues(CPUSys.ALUSys.ARF.AR.Q, 16'h000b, test_no, "AR");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h08],8'h06, test_no, "MEM[AR]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h09],8'h00, test_no, "MEM[AR]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h0A],8'h0A, test_no, "MEM[AR]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h0B],8'h08, test_no, "MEM[AR]");

        //Test 7 CALL 0x32
        test_no = 7;
        CPUSys.ALUSys.IR.IROut = 16'h1C32;
        CPUSys.T = 12'b0000_0000_0100;
        CPUSys.ALUSys.ARF.PC.Q = 16'hAABB;
        CPUSys.ALUSys.ARF.SP.Q = 16'h00FF;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.ARF.PC.Q, 16'h0032, test_no, "PC");
        F.CheckValues(CPUSys.ALUSys.ARF.SP.Q, 16'h00FD, test_no, "SP");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFE],8'hAA, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFF],8'hBB, test_no, "MEM[SP]");

        //Test 8 ORR R1, R1, AR
        test_no = 8;
        CPUSys.ALUSys.IR.IROut = 16'h4A46;
        CPUSys.T = 12'b0000_0000_0100;
        CPUSys.ALUSys.RF.R1.Q = 32'h000000FC;
		CPUSys.ALUSys.ARF.AR.Q = 16'h0012;

        while (CPUSys.T != 12'b0000_0000_0001) begin
            clk.Clock();
        end
        
        F.CheckValues(CPUSys.ALUSys.RF.R1.Q, 32'h000000FE, test_no, "R1");
        F.CheckValues(CPUSys.ALUSys.ARF.AR.Q, 16'h0012, test_no, "AR");

        F.FinishSimulation();
    end

endmodule