# digisim
Digital Logic Simulator
Description: This file contains a Digital Logic Simulator written in C++ for the G++ compiler. The simulator which 
             we will call DigiSim, has 3 main functions: Functional Simulations, Timing Simulations, and Fault 
             Vector Generation. When the simulator is run in terminal, the user will be prompted to input a netlist
             .txt file and then choose which function to perform on the netlist by typing in [y] or [n] to each 
             option. If the user respnods [y] to a functional or timing simulation, then they will also be prompted
             to input another .txt input file for the simulator. The netlist that is input by the user must follow
             the P-Silos guidelines for defining netlists. Additionally, the current iteration of this program only
             supports simple combinational gates (AND, OR, XOR, NAND, NOR, XNOR). More gates, including sequential 
             components such as DFFs will be integrated in a future version. If a timing simulation is run, the 
             output file "TimingSimOutput" will be written to the same directory the program is run from. Similarly,
             if a functional simulation is run, the output file "FunctionalSimOutput" will be written to the same 
             directory the program is run from. If Fault Vector Generation is run, the output file "FaultVectors"
             will be written to the same directory the program is run from. The simulator currently only supports
             logic bit values 0 & 1. Integrating additional logic values U, X, and Z will be done in future versions
             of the simulator. Fault Vector Generation is done using a simple trial-and-error method but can be made
	         more efficient in the future. Lastly, the Fault Vector Generation only detects single stuck-at-x faults
             but can be made to do more complex fault grading in the future. 

			The majority of this project was a learning exercise for C++ so inefficiencies will be ironed out in 
            future versions of the simulator. *Note: Fault Vector Generation will overwrite existing Functional Sim File. 

				
			For examples, see test1-4 folders in the folder. Each of these folders contain a .txt netlist and input file
            to be run with DigiSim. The outputs produced from these files is also included in the testx folder. All
            of the example fault vector generations for tests2-4 were run with a required 100% coverage on the netlist. 
            Note that test1 contains 1 undetectable fault, so a 100% fault grade will cause an infinite loop. Max
            integer fault grade for the test1 netlist is 95%. 

            E.g. when prompted for a netlist file type:
                test2/netlist.txt
            E.g. when prompted for an input file type:
                test2/input.txt


