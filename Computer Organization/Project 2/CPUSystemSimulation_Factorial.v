`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: ITU Computer Engineering Department
// Engineer: Omur F Erzurumluoglu
// Project Name: BLG222E Project 2 Simulation
//////////////////////////////////////////////////////////////////////////////////


module CPUSystemSimulation_Factorial();
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

        test_no = 1;
        DisableAll();
        ClearRegisters();
        
        // Code to calculate Factorial
        //              MOVL    R2, 0x01    # 26 0x6501 # R2 is used for result
        //              MOVL    R1, 0x04  	# 28 0x6404 # R1 is used for iteration number
        //              MOV     R1, R1      # 2A 0x6240 # to pass through ALU
        //              CALL    FACTORIAL   # 2C 0x1C32
        //              STA     R2, 0x54 	# 2E 0x8154 # M[ADDRESS] ← R2
        //              BRA     FINISH      # 30 0x0052
        // FACTORIAL    BEQ     END_F       # 32 0x0844 # If R1 == 0, return
        //              MOVL    R3, 0x01    # 34 0x6601
        //              SUB     R3, R1, R3  # 36 0x5F4C
        //              BEQ     END_F       # 38 0x0844 # If R1 == 1, return
        //              PSHL    R1          # 3A 0x1000
        //              DEC     R1, R1      # 3C 0x2A40
        //              CALL    FACTORIAL   # 3E 0x1C32 # recursive
        //              POPL    R4          # 40 0x0F00
        //              CALL    MULTIPLY    # 42 0x1C46
        // END_F		RET                 # 44 0x2000
        // MULTIPLY	    MOV     R3, R2      # 46 0x6350 # for R2 ← R2*R4
        //              DEC     R4, R4      # 48 0x2BF0
        // M_LOOP	    ADD     R2, R2, R3  # 4A 0x56DC # R4 times R2 ← R2 + R3
        //              DEC     R4, R4      # 4C 0x2BF0
        //              BNE     M_LOOP	    # 4E 0x044A
        //              RET	                # 50 0x2000
        // FINISH                           # 52 NOOP

        CPUSys.ALUSys.ARF.PC.Q = 16'h0026;

        while (test_no <= 69) begin
            clk.Clock();
            if (CPUSys.T == 12'b0000_0000_0001)
                test_no = test_no + 1;
        end
        
        F.CheckValues(CPUSys.ALUSys.IROut,16'h0052, test_no, "IROut");

        F.CheckValues(CPUSys.ALUSys.ARF.PC.Q, 16'h0052, test_no, "PC");
        F.CheckValues(CPUSys.ALUSys.ARF.SP.Q, 16'h00FF, test_no, "SP");
        F.CheckValues(CPUSys.ALUSys.ARF.AR.Q, 16'h0057, test_no, "AR");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h54],8'h00, test_no, "MEM[ADDRESS]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h55],8'h00, test_no, "MEM[ADDRESS]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h56],8'h00, test_no, "MEM[ADDRESS]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'h57],8'h18, test_no, "MEM[ADDRESS]");

        F.CheckValues(CPUSys.ALUSys.RF.R1.Q,32'h00000001, test_no, "R1");
        F.CheckValues(CPUSys.ALUSys.RF.R2.Q,32'h00000018, test_no, "R2");
        F.CheckValues(CPUSys.ALUSys.RF.R3.Q,32'h00000006, test_no, "R3");
        F.CheckValues(CPUSys.ALUSys.RF.R4.Q,32'h00000000, test_no, "R4");

        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF2],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF3],8'h40, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF4],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF5],8'h44, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF6],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF7],8'h40, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF8],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hF9],8'h44, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFA],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFB],8'h40, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFC],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFD],8'h44, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFE],8'h00, test_no, "MEM[SP]");
        F.CheckValues(CPUSys.ALUSys.MEM.RAM_DATA[8'hFF],8'h2E, test_no, "MEM[SP]");

        F.FinishSimulation();
    end

endmodule