call D:\Xilinx\Vivado\2017.4\settings64.bat

set folder=Simulation_Files
mkdir %folder%
copy "RAM.mem" "%folder%/RAM.mem"
cd "%folder%

::CPU System Simulation
call xvlog ../Register16bit.v  
call xvlog ../Register32bit.v  
call xvlog ../RegisterFile.v
call xvlog ../AddressRegisterFile.v  
call xvlog ../InstructionRegister.v
call xvlog ../DataRegister.v  
call xvlog ../ArithmeticLogicUnit.v
call xvlog ../Memory.v  
call xvlog ../ArithmeticLogicUnitSystem.v  
call xvlog ../CPUSystem.v
call xvlog ../CPUSystemSimulation.v
call xvlog ../Helper.v

call xelab -top CPUSystemSimulation -snapshot cpusyssim -debug typical
call xsim cpusyssim -R

::CPU System Simulation for Factorial
call xvlog ../Register16bit.v  
call xvlog ../Register32bit.v  
call xvlog ../RegisterFile.v
call xvlog ../AddressRegisterFile.v  
call xvlog ../InstructionRegister.v
call xvlog ../DataRegister.v  
call xvlog ../ArithmeticLogicUnit.v
call xvlog ../Memory.v  
call xvlog ../ArithmeticLogicUnitSystem.v  
call xvlog ../CPUSystem.v
call xvlog ../CPUSystemSimulation_Factorial.v
call xvlog ../Helper.v

call xelab -top CPUSystemSimulation_Factorial -snapshot cpusyssim -debug typical
call xsim cpusyssim -R

cd ..
