# Digisim
## Digital Logic Simulator
### Description: 
This file contains a Digital Logic Simulator written in C++ for the G++ compiler. The simulator which 
we will call DigiSim, has 3 main functions: Functional Simulations, Timing Simulations, and Fault 
Vector Generation. When the simulator is run in terminal, the user will be prompted to input a netlist
.txt file and then choose which function to perform on the netlist by typing in [y] or [n] to each 
option. If the user respnods [y] to a functional or timing simulation, then they will also be prompted
to input another .txt input test vector file for the simulator. The netlist that is input by the user 
must follow the P-Silos guidelines for defining netlists (see tests1-5). If a timing simulation is run, the 
output file "TimingSimOutput.vcd" will be written to the same directory the program is run from. Similarly,
if a functional simulation is run, the output file "FunctionalSimOutput.vcd" will be written to the same 
directory the program is run from. If Fault Vector Generation is run, the output file "FaultVectors.txt"
will be written to the same directory the program is run from. The simulator currently only supports
logic bit values 0 & 1. Integrating additional logic values U, X, and Z will be done in future versions
of the simulator.  
\
\
Also worth noting, the automatic test pattern generator (ATPG) only works on combinatorial netlists for 
e.g. a scan chain. The ATPG is incapable of doing sequential pattern generation at the moment. In the 
example tests, only run Fault Vector Generation on tests1-4. 

### P-Silos Netlist format (.txt): (see test1-5 examples)
Combinatorial Logic: 
[Output node name] [.Gate Type] [rise time] [fall time] [input node name 1] [input node name 2] ... [input node name 8]
\
\
DFF Logic:
[DFF Identifier] [.Gate Type] [setup time] [hold time] [D input node name] [Clock node name] [Q output node name] [Qn output node name]

### Input File format (.txt): (see test1-5 examples)
[time] [Node Name] [Logic Value]
\
\
\
\
*Note: Fault Vector Generation will overwrite existing Functional Sim File. 
Please note any inefficiencies and report to hjwilson@caltech.edu

For examples, see test1-5 folders. Each of these folders contain a .txt netlist and input file
to be run with Digisim. Test5 is the only test containing sequential logic DFFs and is also significantly 
more complex than tests1-4. 
\
\
\
Compile with:  
g++ -o digisim icon_res.o digisim.cpp

When prompted for a netlist file type:  
	test5/netlist.txt
 
When prompted for an input file type:  
	test5/input.txt
 
 To view output waveform:
 	gtkwave TimingSimOutput.vcd
 or 	gtkwave FunctionalSimOutput.vcd


