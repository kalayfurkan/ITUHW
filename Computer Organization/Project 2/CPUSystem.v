`timescale 1ns / 1ps
module CPUSystem(
    input Clock,
    output reg [11:0] T,
    input Reset
);

    reg[2:0] RF_OutASel, RF_OutBSel, RF_FunSel;
    reg[3:0] RF_RegSel, RF_ScrSel;
    reg[4:0] ALU_FunSel;
    reg ALU_WF; 
    reg[1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel;
    reg[2:0] ARF_RegSel;
    reg IR_LH, IR_Write, Mem_WR, Mem_CS;
    reg[1:0] MuxASel, MuxBSel, MuxCSel;
    reg MuxDSel;
    reg DR_E;
    reg[1:0] DR_FunSel;
    wire Z, C, N, O;

    assign Z = ALUSys.ALU.FlagsOut[3];
    assign C = ALUSys.ALU.FlagsOut[2];
    assign N = ALUSys.ALU.FlagsOut[1];
    assign O = ALUSys.ALU.FlagsOut[0];
    /*          Address
    LSB     0x001 
    MSB     0x002
*/

ArithmeticLogicUnitSystem ALUSys(.Clock(Clock), .MuxASel(MuxASel), .MuxBSel(MuxBSel), .MuxCSel(MuxCSel), .MuxDSel(MuxDSel),
.RF_OutASel(RF_OutASel), .RF_OutBSel(RF_OutBSel), .RF_FunSel(RF_FunSel), .RF_RegSel(RF_RegSel), .RF_ScrSel(RF_ScrSel),
.ALU_FunSel(ALU_FunSel), .ALU_WF(ALU_WF), .ARF_OutCSel(ARF_OutCSel), .ARF_OutDSel(ARF_OutDSel), .ARF_FunSel(ARF_FunSel),
.ARF_RegSel(ARF_RegSel), .IR_LH(IR_LH), .IR_Write(IR_Write), .Mem_WR(Mem_WR), .Mem_CS(Mem_CS), .DR_FunSel(DR_FunSel), .DR_E(DR_E));


reg[5:0] Opcode;
reg[1:0] RegSel;
reg[7:0] Address;
reg[2:0] DestReg;
reg[2:0] SrcReg1;
reg[2:0] SrcReg2;
always @(*) begin
    Opcode = ALUSys.IROut[15:10];
    RegSel = ALUSys.IROut[9:8];
    Address = ALUSys.IROut[7:0];
    DestReg = ALUSys.IROut[9:7];
    SrcReg1 = ALUSys.IROut[6:4];
    SrcReg2 = ALUSys.IROut[3:1];
    
end


//clock increment part
reg T_Reset=1'b0; //to make T zero 
always@(posedge Clock)begin
    if(T_Reset==1'b1)T<=12'b0000_0000_0001;
    else T <= {T[10:0], T[11]};
    T_Reset<=1'b1;
end

always@(negedge Reset)begin
    if(!Reset)begin
        ALUSys.ARF.PC.Q = 16'h0;
        ALUSys.ARF.AR.Q = 16'h0;
        ALUSys.ARF.SP.Q = 16'h00FF;
        ALUSys.IR.IROut = 16'h0;
        ALUSys.DR.DROut = 32'h0;
        ALUSys.ALU.FlagsOut=4'b0000;
        ALUSys.RF.R1.Q = 32'h0;
        ALUSys.RF.R2.Q = 32'h0;
        ALUSys.RF.R3.Q = 32'h0;
        ALUSys.RF.R4.Q = 32'h0;
        ALUSys.RF.S1.Q = 32'h0;
        ALUSys.RF.S2.Q = 32'h0;
        ALUSys.RF.S3.Q = 32'h0;
        ALUSys.RF.S4.Q = 32'h0; 

        ARF_RegSel <= 3'b000;
        IR_Write <= 1'b0;
        Mem_CS <= 1'b1;
        DR_E <= 1'b0;
        T <= 12'b000000000001;
        RF_RegSel <= 4'b0000;
        RF_ScrSel <= 4'b0000;
        ALU_WF <= 1'b0;
    end
end


// main part of control unit
    always @(*) begin
        //default values
        RF_RegSel=4'b0000;
        RF_ScrSel=4'b0000;
        ALU_WF=1'b0;
        ARF_RegSel=3'b000;
        IR_Write=1'b0;
        Mem_CS=1'b1;
        DR_E=1'b0;
        Mem_WR=1'b0;
        T_Reset=1'b0;

        //Memory has 16'b address, 8'b data
        //ARF_FunSel 00:decr 01:incr 10:load 11:clear
        //ARF_RegSel 100:PC 010:SP 001:AR
        //ARF_OutCSel 00:PC 01:SP 10:AR 11:AR
        //ARF_OutDSel 00:PC 01:SP 10:AR 11:AR
        //DR_FunSel 00:Load DR[7:0] with sign extend rest
        //          01:Load DR[7:0] with clear rest
        //          10:(8-bit Left Shift) and Load DR[7:0]
        //          11:(8-bit Right Shift) and Load DR[31:24]
        //RF_FunSel 000:decr 001:incr 010:load 011:clear 100:R[31-8]←Clear, R[7:0]←I[7:0]
        //          101:R[31:16]←Clear, R[15:0]←I[15:0] 110:8-bit Left Shift for R and R[7:0]←I[7:0]
        //          111:R[15:0]←I[15:0] Sign Extend rest
        //RF_OutASel 000:R1 001:R2 010:R3 011:R4 100:S1 101:S2 110:S3 111:S4
        
        case(T)
            12'b0000_0000_0001:begin
                IR_Write=1'b1;
                IR_LH=1'b0; //first load lsb 8 bits
                ARF_OutDSel=2'b00;

                Mem_CS=1'b0;//read enable
                Mem_WR=1'b0;

                ARF_RegSel=3'b100; //select PC
                ARF_FunSel=2'b01; //increment
            end
            12'b0000_0000_0010:begin
                IR_Write=1'b1;
                IR_LH=1'b1; //load msb 8 bits
                ARF_OutDSel=2'b00; // select PC for addressing

                Mem_CS=1'b0;//read enable
                Mem_WR=1'b0;

                ARF_RegSel=3'b100; //select PC
                ARF_FunSel=2'b01; //increment
            end

        default:begin
            case(ALUSys.IROut[15:10])
                6'h00: begin    //BRA
                    if (T == 12'b0000_0000_0100) begin
                        ARF_FunSel=2'b10; //load to PC
                        ARF_RegSel=3'b100;

                        MuxBSel=2'b11; // IR[7:0]
                        T_Reset=1'b1;
                    end
                end
                6'h01: begin
                    if (T == 12'b0000_0000_0100) begin
                        if(Z == 1'b0) begin
                            ARF_FunSel=2'b10;//load to PC
                            ARF_RegSel=3'b100;

                            MuxBSel=2'b11;//IR[7:0]
                            T_Reset=1'b1;
                        end
                    end
                end
                6'h02: begin
                    if (T == 12'b0000_0000_0100) begin  // sadece T=2de çalışacak //fuat
                        if(Z == 1'b1) begin
                            ARF_FunSel=2'b10;//load to PC
                            ARF_RegSel=3'b100;
                            
                            MuxBSel = 2'b11;//IR[7:0]
                            T_Reset=1'b1;
                        end
                    end
                end
                6'h03: begin //SP ← SP+1, Rx ← M[SP](16-bit)
                    case(T)
                        12'b0000_0000_0100: begin
                            ARF_RegSel = 3'b010; //select SP and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0000_1000: begin
                            ARF_OutDSel = 2'b01; //memo reads for SP
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b01;//Load DR[7:0] with clearing rest
                            DR_E = 1'b1;

                            ARF_RegSel = 3'b010; //select SP and increment 
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0001_0000: begin
                            ARF_OutDSel = 2'b01;//memo reads for SP
                            Mem_WR = 1'b0;//read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load DR[7:0]
                            DR_E = 1'b1;
                            
                        end
                        12'b0000_0010_0000: begin
                            MuxASel = 2'b10; // muxA selects DROut
                            //load the correct RF register
                            if(RegSel==2'b00)RF_RegSel=4'b1000;
                            else if(RegSel==2'b01)RF_RegSel=4'b0100;
                            else if(RegSel==2'b10)RF_RegSel=4'b0010;
                            else if(RegSel==2'b11)RF_RegSel=4'b0001;
                            RF_FunSel = 3'b010; //load
                            T_Reset=1'b1;
                        end
                    endcase
                end
                6'h04: begin//M[SP] ← Rx, SP ← SP – 1 (16-bit)
                    // M[SP] <- Rx (16 bit)
                    //T2: write LSB to M, dec SP
                    //T3: write MSB to M, dec SP

                    case(T)
                        12'b0000_0000_0100: begin
                            RF_OutASel = {1'b0,ALUSys.IROut[9:8]}; //Reg select
                            MuxDSel = 1'b0; //connect RF to ALU
                            ALU_FunSel = 5'b10000; //A (32 bit)
                            ALU_WF<=1'b1;
                            MuxCSel = 2'b00; //first lsb 8 bits

                            ARF_OutDSel = 2'b01;
                            Mem_WR = 1'b1;//write enable                 
                            Mem_CS=1'b0;

                            ARF_RegSel = 3'b010;//select SP
                            ARF_FunSel = 2'b00; //decrement
                        end
                        12'b0000_0000_1000: begin 
                            RF_OutASel = {1'b0,ALUSys.IROut[9:8]};
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b10000;
                            ALU_WF<=1'b1;
                            MuxCSel = 2'b01;//second 8 bits

                            ARF_OutDSel = 2'b01;
                            Mem_WR = 1'b1;
                            Mem_CS=1'b0;

                            ARF_RegSel = 3'b010;
                            ARF_FunSel = 2'b00;//decrement
                            T_Reset=1'b1;//fuat
                        end
                    endcase
                end

                6'h05: begin//SP ← SP+1, Rx ← M[SP](32-bit)
                        case(T)
                        12'b0000_0000_0100: begin
                            ARF_RegSel = 3'b010; //select SP and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0000_1000: begin
                            ARF_OutDSel = 2'b01; //memo reads for SP
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b01;//Load DR[7:0] with clearing rest
                            DR_E = 1'b1;

                            ARF_RegSel = 3'b010; //select SP and increment 
                            ARF_FunSel = 2'b01;
                        end
                        
                        12'b0000_0001_0000: begin
                            ARF_OutDSel = 2'b01;//memo reads for SP
                            Mem_WR = 1'b0;//read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load second 8 bits
                            DR_E = 1'b1;
                            
                            ARF_RegSel = 3'b010; //select SP and increment 
                            ARF_FunSel = 2'b01;
                        end

                        12'b0000_0010_0000: begin
                            ARF_OutDSel = 2'b01;//memo reads for SP
                            Mem_WR = 1'b0;//read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load third 8 bits
                            DR_E = 1'b1;
                            
                            ARF_RegSel = 3'b010; //select SP and increment 
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0100_0000: begin
                            ARF_OutDSel = 2'b01;//memo reads for SP
                            Mem_WR = 1'b0;//read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load last 8 bits
                            DR_E = 1'b1;
                            
                        end

                        12'b0000_0100_0000: begin
                            MuxASel = 2'b10; // muxA selects DROut
                            //load the correct RF register
                            if(RegSel==2'b00)RF_RegSel=4'b1000;
                            else if(RegSel==2'b01)RF_RegSel=4'b0100;
                            else if(RegSel==2'b10)RF_RegSel=4'b0010;
                            else if(RegSel==2'b11)RF_RegSel=4'b0001;
                            RF_FunSel = 3'b010; //load
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h06: begin//M[SP] ← Rx, SP ← SP – 1 (32-bit)
                    case(T)
                        12'b0000_0000_0100: begin
                            RF_OutASel = {1'b0,ALUSys.IROut[9:8]}; //Reg select
                            MuxDSel = 1'b0; //connect RF to ALU
                            ALU_FunSel = 5'b00000; //A (16 bit)
                            MuxCSel = 2'b11; //first msb 8 bits

                            ARF_OutDSel = 2'b01;
                            Mem_WR = 1'b1;//write enable                 
                            Mem_CS=1'b0;

                            ARF_RegSel = 3'b010;//select SP
                            ARF_FunSel = 2'b00; //decrement
                        end
                        12'b0000_0000_1000: begin 
                            RF_OutASel = {1'b0,ALUSys.IROut[9:8]};
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b00000;
                            MuxCSel = 2'b10;//second 8 bits

                            ARF_OutDSel = 2'b01;
                            Mem_WR = 1'b1;
                            Mem_CS=1'b0;

                            ARF_RegSel = 3'b010;
                            ARF_FunSel = 2'b00;//decrement
                        end
                        12'b0000_0000_1000: begin 
                            RF_OutASel = {1'b0,ALUSys.IROut[9:8]};
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b00000;
                            MuxCSel = 2'b01;//third 8 bits
                            
                            ARF_OutDSel = 2'b01;
                            Mem_WR = 1'b1;
                            Mem_CS=1'b0;

                            ARF_RegSel = 3'b010;
                            ARF_FunSel = 2'b00;//decrement
                        end
                        12'b0000_0000_1000: begin 
                            RF_OutASel = {1'b0,ALUSys.IROut[9:8]};
                            MuxDSel = 1'b0;
                            ALU_FunSel = 5'b00000;
                            MuxCSel = 2'b00;//lsb 8 bits

                            ARF_OutDSel = 2'b01;
                            Mem_WR = 1'b1;
                            Mem_CS=1'b0;

                            ARF_RegSel = 3'b010;
                            ARF_FunSel = 2'b00;//decrement
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h07: begin //test 7den geçti bu
                    case(T)
                        12'b0000_0000_0100: begin
                            ARF_OutCSel=2'b00;
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b00000;
                            MuxCSel=2'b00;

                            Mem_WR = 1'b1;
                            Mem_CS = 1'b0;
                            ARF_OutDSel=2'b01;

                            ARF_FunSel=2'b00;
                            ARF_RegSel=3'b010;

                        end
                        12'b0000_0000_1000: begin
                            ARF_OutCSel=2'b00;
                            MuxDSel = 1'b1;
                            ALU_FunSel = 5'b00000;
                            MuxCSel=2'b01;

                            Mem_WR = 1'b1;
                            Mem_CS = 1'b0;
                            ARF_OutDSel=2'b01;

                            ARF_FunSel=2'b00;
                            ARF_RegSel=3'b010;
                        end
                        12'b0000_0001_0000: begin
                            ARF_FunSel=2'b10;       //value->pc kısmı
                            ARF_RegSel=3'b100;
                            MuxBSel=2'b11;
                            T_Reset=1'b1;
                        end
                    endcase
                end
                6'h08: begin//SP ← SP + 1, PC ← M[SP] (16 bit)
                    case(T)
                        12'b0000_0000_0100: begin
                            ARF_RegSel = 3'b010; //select SP and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0000_1000: begin
                            ARF_OutDSel = 2'b01;//memo reads for SP
                            Mem_WR = 1'b0;//read enable
                            Mem_CS = 1'b0;

                            DR_E=1'b1;//Load DR[7:0] with clearing rest
                            DR_FunSel=2'b01;

                            ARF_RegSel = 3'b010; //select SP and increment 
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0001_0000: begin
                            ARF_OutDSel = 2'b01;//memo reads for SP
                            Mem_WR = 1'b0;//read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load DR[7:0]
                            DR_E = 1'b1;
                        end
                        12'b0000_0010_0000: begin
                            MuxBSel=2'b10;//connect DROut to ARF
                            ARF_FunSel=2'b10;//load to PC
                            ARF_RegSel=3'b100;

                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h09: begin
                    case(T)
                        12'b0000_0000_0100: begin//testten geçmesi için önce 1 azaltacağız
                            RF_ScrSel=4'b0100;
                            RF_FunSel<=3'b010;
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxASel=2'b01;
                            end
                            else begin //eğer rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10001;
                                ALU_WF=1'b1;
                                MuxASel=2'b00;
                            end                 
                        end
                        12'b0000_0000_1000: begin //1 artırdık
                            RF_ScrSel=4'b0100;
                            RF_FunSel=3'b001;
                        end
                        12'b0000_0001_0000: begin
                            RF_OutBSel=3'b101;
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                            else begin
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                RF_FunSel=3'b010;
                                MuxASel=2'b00;
                            end    
                            T_Reset=1'b1;       
                        end
                    endcase
                end

                6'h0A: begin
                    case(T)
                        12'b0000_0000_0100: begin//testten geçmesi için önce 1 azaltacağız
                            RF_ScrSel=4'b0100;
                            RF_FunSel<=3'b010;
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxASel=2'b01;
                            end
                            else begin //eğer rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10001;
                                ALU_WF=1'b1;
                                MuxASel=2'b00;
                            end                 
                        end
                        12'b0000_0000_1000: begin //1 azalttık
                            RF_ScrSel=4'b0100;
                            RF_FunSel=3'b000;
                        end
                        12'b0000_0001_0000: begin
                            RF_OutBSel=3'b101;
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                            else begin
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                RF_FunSel=3'b010;
                                MuxASel=2'b00;
                            end    
                            T_Reset=1'b1;       
                        end
                    endcase
                end

                6'h0B: begin
                    case(T)
                        12'b0000_0000_0100: begin
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    ALU_FunSel=5'b01011;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                end
                                else begin // eğer source RF içindeyse
                                    RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                    MuxDSel=1'b0;
                                    ALU_FunSel=5'b11011;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                end
                            end
                            else begin //eğer destination RF içindeyse
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                                if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    ALU_FunSel=5'b01011;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                end
                                else begin //eğer source RF içindeyse
                                    RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                    MuxDSel=1'b0;
                                    ALU_FunSel=5'b11011;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                end
                            end
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h0C: begin
                    case(T)
                        12'b0000_0000_0100: begin
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    ALU_FunSel=5'b01100;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                end
                                else begin // eğer source RF içindeyse
                                    RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                    MuxDSel=1'b0;
                                    ALU_FunSel=5'b11100;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                end
                            end
                            else begin //eğer destination RF içindeyse
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                                if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    ALU_FunSel=5'b01100;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                end
                                else begin //eğer source RF içindeyse
                                    RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                    MuxDSel=1'b0;
                                    ALU_FunSel=5'b11100;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                end
                            end
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h0D:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b01101;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                            else begin // eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11101;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                        end
                        else begin //eğer destination RF içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                            if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b01101;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                            else begin //eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11101;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                        end
                        T_Reset=1'b1;
                    end
                    endcase
                end

                6'h0E:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b01110;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                            else begin // eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11110;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                        end
                        else begin //eğer destination RF içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                            if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b01110;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                            else begin //eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11110;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                        end
                        T_Reset=1'b1;
                    end
                    endcase
                end

                6'h0F:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b01111;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                            else begin // eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11111;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                        end
                        else begin //eğer destination RF içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                            if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b01111;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                            else begin //eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11111;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                        end
                        T_Reset=1'b1;
                    end
                    endcase
                end

                6'h10:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b00010;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                            else begin // eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10010;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                            end
                        end
                        else begin //eğer destination RF içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                            if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                ALU_FunSel=5'b00010;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                            else begin //eğer source RF içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10010;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                            end
                        end
                        T_Reset=1'b1;
                    end
                    endcase
                end

                6'h11:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b10111;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; //bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10111;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10111;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin //destination rf içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b10111;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10111;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10111;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    12'b0000_0000_1000:begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b10111;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b10111;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    endcase
                end
                
                6'h12:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b11000;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; //bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b11000;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11000;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin //destination rf içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b11000;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b11000;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11000;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    12'b0000_0000_1000:begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b11000;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b11000;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    endcase
                end

                6'h13:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b11001;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; //bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b11001;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11001;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin //destination rf içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b11001;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b11001;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11001;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    12'b0000_0000_1000:begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b11001;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b11001;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    endcase
                end

                6'h14:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b11010;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; //bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b11010;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11010;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin //destination rf içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b11010;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b11010;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b11010;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    12'b0000_0000_1000:begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b11010;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b11010;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    endcase
                end

                6'h15:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b10100;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; //bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10100;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10100;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin //destination rf içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b10100;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10100;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10100;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    12'b0000_0000_1000:begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b10100;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b10100;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    endcase
                end

                6'h16:begin
                    case(T)
                        12'b0000_0000_0100: begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b10101;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; //bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10101;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10101;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin //destination rf içindeyse
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;
                                RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                            end
                            else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                ALU_FunSel=5'b10101;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[2:1];
                                MuxDSel=1'b1;
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                ALU_FunSel=5'b10101;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                            else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                MuxDSel=1'b0;
                                ALU_FunSel=5'b10101;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    12'b0000_0000_1000:begin
                        if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                            if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                            else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                            else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                            else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b10101;
                                MuxBSel=2'b00;
                                ARF_FunSel=2'b10;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                        else begin
                            if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                            if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                MuxDSel=1'b1;
                                RF_OutBSel=3'b100;
                                ALU_FunSel=5'b10101;
                                MuxASel=2'b00;
                                RF_FunSel=3'b010;
                                T_Reset=1'b1; // bu bitti
                            end
                        end
                    end
                    endcase
                end

                6'h17:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                    ARF_OutCSel=ALUSys.IROut[2:1];
                                    MuxASel=2'b01;
                                    RF_ScrSel=4'b1000;
                                    RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                                end
                                else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    RF_OutBSel={1'b0, ALUSys.IROut[2:1]};
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                    T_Reset=1'b1; //bu bitti
                                end
                                else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0)begin // ilki rf ikincisi arf içindeyse
                                    ARF_OutCSel=ALUSys.IROut[2:1];
                                    MuxDSel=1'b1;
                                    RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                    T_Reset=1'b1; // bu bitti
                                end
                                else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1)begin //ikisi de rf içindeyse
                                    RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                                    RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                    MuxDSel=1'b0;
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                    T_Reset=1'b1; // bu bitti
                                end
                            end
                            else begin //destination rf içindeyse
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                    ARF_OutCSel=ALUSys.IROut[2:1];
                                    MuxASel=2'b01;
                                    RF_ScrSel=4'b1000;
                                    RF_FunSel=3'b010;//bu diğer cycleda devam edecek
                                end
                                else if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b1) begin//ilki arf ikinci source rf içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    RF_OutBSel={1'b0,ALUSys.IROut[2:1]};
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                    T_Reset=1'b1; // bu bitti
                                end
                                else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b0) begin//ilki rf ikinci source arf içindeyse
                                    ARF_OutCSel=ALUSys.IROut[2:1];
                                    MuxDSel=1'b1;
                                    RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                    T_Reset=1'b1; // bu bitti
                                end
                                else if(ALUSys.IROut[6]==1'b1 && ALUSys.IROut[3]==1'b1) begin//ikisi de rf içindeyse
                                    RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                    RF_OutASel={1'b0,ALUSys.IROut[2:1]};
                                    MuxDSel=1'b0;
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                    T_Reset=1'b1; // bu bitti
                                end
                            end
                        end
                        12'b0000_0000_1000:begin
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin // iki source da arf içinde
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    RF_OutBSel=3'b100;
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                    T_Reset=1'b1; // bu bitti
                                end
                            end
                            else begin
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                if(ALUSys.IROut[6]==1'b0 && ALUSys.IROut[3]==1'b0) begin//iki source da arf içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxDSel=1'b1;
                                    RF_OutBSel=3'b100;
                                    ALU_FunSel=5'b10110;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                    T_Reset=1'b1; // bu bitti
                                end
                            end
                        end
                    endcase
                end
                
                6'h18: begin //DSTREG ← SREG1
                    //ARF_RegSel 100:PC 010:SP 001:AR
                    case(T)
                        12'b0000_0000_0100: begin
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxBSel=2'b01;
                                    ARF_FunSel=2'b10;
                                end
                                else begin // eğer source RF içindeyse
                                    RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                    ALU_FunSel=5'b10001;
                                    ALU_WF=1'b1;
                                    MuxBSel=2'b00;
                                    ARF_FunSel=2'b10;
                                end
                            end
                            
                            else begin //eğer destination RF içindeyse
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                    else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                    else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                    else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                if(ALUSys.IROut[6]==1'b0)begin //eğer source ARF içindeyse
                                    ARF_OutCSel=ALUSys.IROut[5:4];
                                    MuxASel=2'b01;
                                    RF_FunSel=3'b010;
                                end
                                else begin //eğer source RF içindeyse
                                    RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                                    ALU_FunSel=5'b10001;
                                    ALU_WF=1'b1;
                                    MuxASel=2'b00;
                                    RF_FunSel=3'b010;
                                end
                            end
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h19:begin //Rx[7:0] ← IMMEDIATE (8-bit) // bu kısım test2den geçti
                    case(T)
                        12'b0000_0000_0100: begin
                            MuxASel=2'b11;
                            RF_FunSel=3'b010;//load to s1
                            RF_ScrSel=4'b1000;
                        end
                        12'b0000_0000_1000: begin
                            if(ALUSys.IROut[9:8]==2'b00)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:8]==2'b01)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:8]==2'b10)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:8]==2'b11)RF_RegSel=4'b0001;

                            RF_OutBSel=3'b100;
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            MuxASel=2'b00;
                            RF_FunSel=3'b010;
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h1A:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            MuxASel=2'b11;
                            RF_FunSel=3'b010;//load to s1
                            RF_ScrSel=4'b1000;
                        end
                        12'b0000_0000_1000: begin
                            if(ALUSys.IROut[9:8]==2'b00)RF_RegSel=4'b1000;
                            else if(ALUSys.IROut[9:8]==2'b01)RF_RegSel=4'b0100;
                            else if(ALUSys.IROut[9:8]==2'b10)RF_RegSel=4'b0010;
                            else if(ALUSys.IROut[9:8]==2'b11)RF_RegSel=4'b0001;

                            RF_OutBSel=3'b100;
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            MuxASel=2'b00;
                            RF_FunSel=3'b110;
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h1B:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            Mem_CS=1'b0;
                            Mem_WR=1'b0;
                            DR_E=1'b1;
                            DR_FunSel=2'b01;
                            ARF_OutDSel=2'b11;
                            ARF_FunSel=2'b01;
                            ARF_RegSel=3'b001;
                        end
                        12'b0000_0000_1000:begin
                            Mem_CS=1'b0;
                            Mem_WR=1'b0;
                            DR_E=1'b1;
                            DR_FunSel=2'b10;
                            ARF_OutDSel=2'b11;
                            ARF_FunSel=2'b01;
                            ARF_RegSel=3'b001;
                        end
                        12'b0000_0001_0000:begin 
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                MuxBSel=2'b10;
                                ARF_FunSel=2'b10;
                            end
                            else begin //rf içindeyse
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                MuxASel=2'b10;
                                RF_FunSel=3'b010;
                            end
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h1C:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            DR_E=1'b1;
                            DR_FunSel=2'b01;
                            Mem_CS=1'b0;
                            Mem_WR=1'b0;
                            ARF_OutDSel=2'b11;
                            ARF_FunSel=2'b01;
                            ARF_RegSel=3'b001;
                        end
                        12'b0000_0000_1000:begin
                            Mem_CS=1'b0;
                            Mem_WR=1'b0;
                            DR_E=1'b1;
                            DR_FunSel=2'b10;
                            ARF_OutDSel=2'b11;
                            ARF_FunSel=2'b01;
                            ARF_RegSel=3'b001;
                        end
                        12'b0000_0001_0000:begin
                            Mem_CS=1'b0;
                            Mem_WR=1'b0;
                            DR_E=1'b1;
                            DR_FunSel=2'b10;
                            ARF_OutDSel=2'b11;
                            ARF_FunSel=2'b01;
                            ARF_RegSel=3'b001;
                        end
                        12'b0000_0010_0000:begin
                            Mem_CS=1'b0;
                            Mem_WR=1'b0;
                            DR_E=1'b1;
                            DR_FunSel=2'b10;
                            ARF_OutDSel=2'b11;
                            ARF_FunSel=2'b01;
                            ARF_RegSel=3'b001;
                        end
                        12'b0000_0100_0000:begin 
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;
                                MuxBSel=2'b10;
                                ARF_FunSel=2'b10;
                            end
                            else begin //rf içindeyse
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;
                                MuxASel=2'b10;
                                RF_FunSel=3'b010;
                            end
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h1D:begin//M[AR] ← SREG1
                    case(T)
                        12'b0000_0000_0100: begin
                            if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse s1 içinde depolayacağız
                                ARF_OutCSel=ALUSys.IROut[5:4];
                                
                                MuxASel=2'b01;
                                RF_ScrSel=4'b1000;//load s1
                                RF_FunSel=3'b010;
                                RF_RegSel=4'b0000;//protect others
                            end
                        end
                        12'b0000_0000_1000: begin
                            Mem_CS=1'b0;//write enable
                            Mem_WR=1'b1;
                            ARF_OutDSel=2'b11;//AR drives memory

                            ARF_RegSel=3'b001;//increment AR
                            ARF_FunSel=2'b01;
                            
                            if(ALUSys.IROut[6]==1'b0) begin //source S1
                                RF_OutASel=3'b100;
                            end
                            else begin //source RF
                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                            end

                            MuxDSel=1'b0;//connect RF to ALU
                            ALU_FunSel=5'b10000; //dont change data
                            MuxCSel=2'b11; //present msb 8 bit of ALU output to memory
                        end
                        12'b0000_0001_0000: begin
                            Mem_CS=1'b0;//write enable
                            Mem_WR=1'b1;
                            ARF_OutDSel=2'b11;//AR drives memory

                            if(ALUSys.IROut[6]==1'b0) begin //source S1
                                RF_OutASel=3'b100;

                                T_Reset=1'b1; //finished for 16 bits
                            end
                            else begin //source Rx
                                ARF_RegSel=3'b001;//increment AR
                                ARF_FunSel=2'b01;

                                RF_OutASel={1'b0,ALUSys.IROut[5:4]};
                            end

                            MuxDSel=1'b0;//connect RF to ALU
                            ALU_FunSel=5'b10000; //dont change data
                            MuxCSel=2'b10; //present second 8 bit of ALU output to memory
                        end
                        12'b0000_0010_0000: begin //Rx için
                            Mem_CS=1'b0;//write enable
                            Mem_WR=1'b1;
                            ARF_OutDSel=2'b11;//AR drives memory

                            ARF_RegSel=3'b001;//increment AR
                            ARF_FunSel=2'b01;
                            
                            //scratch'den gelmiyor if'e gerek yok
                            RF_OutASel={1'b0,ALUSys.IROut[5:4]};

                            MuxDSel=1'b0;//connect RF to ALU
                            ALU_FunSel=5'b10000; //dont change data
                            MuxCSel=2'b01; //present third 8 bit of ALU output to memory
                        end
                        12'b0000_0100_0000: begin //Rx için
                            Mem_CS=1'b0;//write enable
                            Mem_WR=1'b1;
                            ARF_OutDSel=2'b11;//AR drives memory
                            
                            //scratch'den gelmiyor if'e gerek yok
                            RF_OutASel={1'b0,ALUSys.IROut[5:4]};

                            MuxDSel=1'b0;//connect RF to ALU
                            ALU_FunSel=5'b10000; //dont change data
                            MuxCSel=2'b00; //present the lsb 8 bit of ALU output to memory

                            T_Reset=1'b1; //finished for 32 bits
                        end
                        // 12'b0000_0000_0100: begin
                        //     if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                        //         ARF_OutCSel=ALUSys.IROut[5:4];
                        //         ARF_OutDSel=2'b11;

                        //         Mem_CS=1'b0;//write enable
                        //         Mem_WR=1'b1;

                        //         MuxDSel=1'b1;//connect ARF to ALU

                        //         ALU_FunSel=5'b00000;
                        //         MuxCSel=2'b00;

                        //         ARF_RegSel=3'b001;
                        //         ARF_FunSel=2'b01;
                        //     end
                        //     else begin //source rf içindeyse
                        //         RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                        //         ALU_FunSel=5'b10001;

                        //         Mem_CS=1'b0;
                        //         Mem_WR=1'b1;
                        //         MuxCSel=2'b00;

                        //         ARF_OutDSel=2'b11;
                        //         ARF_RegSel=3'b001;
                        //         ARF_FunSel=2'b01;
                        //     end
                        // end
                        // 12'b0000_0000_1000: begin
                        //     if(ALUSys.IROut[6]==1'b0) begin // eğer source ARF içindeyse
                        //         ARF_OutCSel=ALUSys.IROut[5:4];
                        //         ARF_OutDSel=2'b11;

                        //         Mem_CS=1'b0;
                        //         Mem_WR=1'b1;
                        //         MuxDSel=1'b1;

                        //         ALU_FunSel=5'b00000;
                        //         MuxCSel=2'b01;

                        //         ARF_RegSel=3'b001;
                        //         ARF_FunSel=2'b01;
                        //         T_Reset=1'b1;
                        //     end
                        //     else begin //eğer source rf içindeyse
                        //         RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                        //         ALU_FunSel=5'b10001;

                        //         Mem_CS=1'b0;
                        //         Mem_WR=1'b1;
                        //         MuxCSel=2'b01;

                        //         ARF_OutDSel=2'b11;
                        //         ARF_RegSel=3'b001;
                        //         ARF_FunSel=2'b01;
                        //     end
                        // end

                        // 12'b0000_0001_0000: begin
                        //     RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                        //     ALU_FunSel=5'b10001;

                        //     Mem_CS=1'b0;
                        //     Mem_WR=1'b1;
                        //     MuxCSel=2'b10;

                        //     ARF_OutDSel=2'b11;
                        //     ARF_RegSel=3'b001;
                        //     ARF_FunSel=2'b01;
                        // end

                        // 12'b0000_0010_0000: begin
                        //     RF_OutBSel={1'b0,ALUSys.IROut[5:4]};
                        //     ALU_FunSel=5'b10001;

                        //     Mem_CS=1'b0;
                        //     Mem_WR=1'b1;
                        //     MuxCSel=2'b11;

                        //     ARF_OutDSel=2'b11;
                        //     ARF_RegSel=3'b001;
                        //     ARF_FunSel=2'b01;
                        //     T_Reset=1'b1;
                        // end
                    endcase
                end

                6'h1E:begin//Rx ← M[ADDRESS] (16-bit)
                    case(T)
                        12'b0000_0000_0100: begin
                            MuxBSel=2'b11;//load address to AR
                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b10;
                        end

                        12'b0000_0000_1000: begin
                            ARF_OutDSel = 2'b10; //memo reads for AR
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b01;//Load DR[7:0] with clearing rest
                            DR_E = 1'b1;

                            ARF_RegSel = 3'b001; //select AR and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0001_0000: begin
                            ARF_OutDSel = 2'b10; //memo reads for AR
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load DR[7:0]
                            DR_E = 1'b1;
                        end
                        12'b0000_0010_0000: begin
                            MuxASel = 2'b10; // muxA selects DROut
                            //load the correct RF register
                            if(RegSel==2'b00)RF_RegSel=4'b1000;
                            else if(RegSel==2'b01)RF_RegSel=4'b0100;
                            else if(RegSel==2'b10)RF_RegSel=4'b0010;
                            else if(RegSel==2'b11)RF_RegSel=4'b0001;
                            RF_FunSel = 3'b010; //load
                            T_Reset=1'b1;
                        end
                    endcase
                end
                //bitti bu

                6'h1F:begin//Rx ← M[ADDRESS] (32-bit)
                    case(T)
                        12'b0000_0000_0100: begin
                            MuxBSel=2'b11;//load address to AR
                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b10;
                        end

                        12'b0000_0000_1000: begin
                            ARF_OutDSel = 2'b10; //memo reads for AR
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b01;//Load DR[7:0] with clearing rest
                            DR_E = 1'b1;

                            ARF_RegSel = 3'b001; //select AR and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0001_0000: begin
                            ARF_OutDSel = 2'b10; //memo reads for AR
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load second 8 bits
                            DR_E = 1'b1;

                            ARF_RegSel = 3'b001; //select AR and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0010_0000: begin
                            ARF_OutDSel = 2'b10; //memo reads for AR
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load third 8 bits
                            DR_E = 1'b1;

                            ARF_RegSel = 3'b001; //select AR and increment
                            ARF_FunSel = 2'b01;
                        end
                        12'b0000_0100_0000: begin
                            ARF_OutDSel = 2'b10; //memo reads for AR
                            Mem_WR = 1'b0; //read enable
                            Mem_CS = 1'b0;

                            DR_FunSel = 2'b10;//(8-bit Left Shift) and Load last 8 bits
                            DR_E = 1'b1;
                        end
                        12'b0000_1000_0000: begin
                            MuxASel = 2'b10; // muxA selects DROut
                            //load the correct RF register
                            if(RegSel==2'b00)RF_RegSel=4'b1000;
                            else if(RegSel==2'b01)RF_RegSel=4'b0100;
                            else if(RegSel==2'b10)RF_RegSel=4'b0010;
                            else if(RegSel==2'b11)RF_RegSel=4'b0001;
                            RF_FunSel = 3'b010; //load
                            T_Reset=1'b1;
                        end   
                    endcase
                
                    //case(T)
                        //         12'b0000_0000_0100: begin
                        //             MuxBSel=2'b11;
                        //             ARF_RegSel=3'b001;
                        //             ARF_FunSel=2'b10;
                        //         end
                        //         12'b0000_0000_1000: begin
                        //             ARF_OutDSel=2'b11;
                        //             Mem_CS=1'b0;
                        //             Mem_WR=1'b0;
                                    
                        //             ARF_RegSel=3'b001;
                        //             ARF_FunSel=2'b01;

                        //             DR_E=1'b1;
                        //             DR_FunSel=2'b01;
                        //         end
                        //         12'b0000_0001_0000: begin
                        //             ARF_OutDSel=2'b11;
                        //             Mem_CS=1'b0;
                        //             Mem_WR=1'b0;

                        //             ARF_RegSel=3'b001;
                        //             ARF_FunSel=2'b01;

                        //             DR_E=1'b1;
                        //             DR_FunSel=2'b10;
                        //         end
                        //         12'b0000_0010_0000: begin
                        //             ARF_OutDSel=2'b11;
                        //             Mem_CS=1'b0;
                        //             Mem_WR=1'b0;

                        //             ARF_RegSel=3'b001;
                        //             ARF_FunSel=2'b01;

                        //             DR_E=1'b1;
                        //             DR_FunSel=2'b10;
                        //         end
                        //         12'b0000_0100_0000: begin
                        //             ARF_OutDSel=2'b11;
                        //             Mem_CS=1'b0;
                        //             Mem_WR=1'b0;

                        //             ARF_RegSel=3'b001;
                        //             ARF_FunSel=2'b01;

                        //             DR_E=1'b1;
                        //             DR_FunSel=2'b10;
                        //         end
                        //         12'b0000_1000_0000: begin
                        //             if(RegSel==2'b00)RF_RegSel=4'b1000;
                        //             else if(RegSel==2'b01)RF_RegSel=4'b0100;
                        //             else if(RegSel==2'b10)RF_RegSel=4'b0010;
                        //             else if(RegSel==2'b11)RF_RegSel=4'b0001;
                        //             RF_FunSel=3'b010;
                        //             MuxASel=2'b10;
                        //             T_Reset=1'b1;
                        //         end
                    //endcase
                end
                //bitti bu

                6'h20:begin // saat 00.46 güncelledim MSB LSB mantığım yanlış imiş
                    case(T)
                        12'b0000_0000_0100: begin
                            MuxBSel=2'b11;
                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b10;
                        end

                        12'b0000_0000_1000: begin
                            RF_OutBSel={1'b0,ALUSys.IROut[9:8]};
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            MuxCSel=2'b11;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;
                            
                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end

                        12'b0000_0001_0000: begin
                            RF_OutBSel={1'b0,ALUSys.IROut[9:8]};
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            MuxCSel=2'b10;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;
                            
                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end

                        12'b0000_0010_0000: begin
                            RF_OutBSel={1'b0,ALUSys.IROut[9:8]};
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            MuxCSel=2'b01;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;
                            
                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end

                        12'b0000_0100_0000: begin
                            RF_OutBSel={1'b0,ALUSys.IROut[9:8]};
                            ALU_FunSel=5'b10001;
                            ALU_WF=1'b1;
                            MuxCSel=2'b00;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h21:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            DR_E=1'b1;
                            DR_FunSel=2'b01;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_0000_1000: begin
                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            DR_E=1'b1;
                            DR_FunSel=2'b10;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h22:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            DR_E=1'b1;
                            DR_FunSel=2'b01;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_0000_1000: begin
                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            DR_E=1'b1;
                            DR_FunSel=2'b10;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_0001_0000: begin
                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            DR_E=1'b1;
                            DR_FunSel=2'b10;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_0010_0000: begin
                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            DR_E=1'b1;
                            DR_FunSel=2'b10;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h23:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            if(ALUSys.IROut[9]==1'b0)begin //eğer destination register ARF içindeyse
                                if(ALUSys.IROut[9:7]==3'b000)ARF_RegSel=3'b100;
                                else if(ALUSys.IROut[9:7]==3'b001)ARF_RegSel=3'b010;
                                else if(ALUSys.IROut[9:7]==3'b010)ARF_RegSel=3'b001;
                                else if(ALUSys.IROut[9:7]==3'b011)ARF_RegSel=3'b001;

                                ARF_FunSel=2'b10;
                                MuxBSel=2'b10;
                            end
                            else begin
                                if(ALUSys.IROut[9:7]==3'b100)RF_RegSel=4'b1000;
                                else if(ALUSys.IROut[9:7]==3'b101)RF_RegSel=4'b0100;
                                else if(ALUSys.IROut[9:7]==3'b110)RF_RegSel=4'b0010;
                                else if(ALUSys.IROut[9:7]==3'b111)RF_RegSel=4'b0001;

                                RF_FunSel=3'b010;
                                MuxASel=2'b10;
                            end
                            T_Reset=1'b1;
                        end
                    endcase
                end

                6'h24:begin
                    case(T)
                        12'b0000_0000_0100: begin
                            MuxASel=2'b11;
                            RF_FunSel=3'b010;
                            RF_ScrSel=4'b1000;
                        end
                        12'b0000_0000_1000: begin
                            RF_OutBSel=3'b100;
                            ARF_OutCSel=2'b11;
                            MuxDSel=1'b1;

                            ALU_FunSel=5'b00100;
                            MuxBSel=2'b00;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b10;
                        end
                        12'b0000_0001_0000: begin
                            RF_OutBSel={1'b0,RegSel};
                            ALU_FunSel=5'b10001;
                            MuxCSel=2'b00;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_0010_0000: begin
                            RF_OutBSel={1'b0,RegSel};
                            ALU_FunSel=5'b10001;
                            MuxCSel=2'b01;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_0100_0000: begin
                            RF_OutBSel={1'b0,RegSel};
                            ALU_FunSel=5'b10001;
                            MuxCSel=2'b10;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;
                        end
                        12'b0000_1000_0000: begin
                            RF_OutBSel={1'b0,RegSel};
                            ALU_FunSel=5'b10001;
                            MuxCSel=2'b11;

                            ARF_OutDSel=2'b11;
                            Mem_CS=1'b0;
                            Mem_WR=1'b1;

                            ARF_RegSel=3'b001;
                            ARF_FunSel=2'b01;

                            T_Reset=1'b1;
                        end
                    endcase
                end

            endcase
        end
        
        endcase
    end
endmodule
