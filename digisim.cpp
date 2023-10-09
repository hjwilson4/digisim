// ------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- EE 150 DigiSim -----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// Description: This file contains a Digital Logic Simulator written in C++ for the G++ compiler. The simulator which 
//              we will call DigiSim, has 3 main functions: Functional Simulations, Timing Simulations, and Fault 
//              Vector Generation. When the simulator is run in terminal, the user will be prompted to input a netlist
//              .txt file and then choose which function to perform on the netlist by typing in [y] or [n] to each 
//              option. If the user respnods [y] to a functional or timing simulation, then they will also be prompted
//              to input another .txt input file for the simulator. The netlist that is input by the user must follow
//              the P-Silos guidelines for defining netlists. Additionally, the current iteration of this program only
//              supports simple combinational gates (AND, OR, XOR, NAND, NOR, XNOR). More gates, including sequential 
//              components such as DFFs will be integrated in a future version. If a timing simulation is run, the 
//              output file "TimingSimOutput" will be written to the same directory the program is run from. Similarly,
//              if a functional simulation is run, the output file "FunctionalSimOutput" will be written to the same 
//              directory the program is run from. If Fault Vector Generation is run, the output file "FaultVectors"
//              will be written to the same directory the program is run from. The simulator currently only supports
//              logic bit values 0 & 1. Integrating additional logic values U, X, and Z will be done in future versions
//              of the simulator. Fault Vector Generation is done using a simple trial-and-error method but can be made
//				more efficient in the future. 
//
//				The majority of this project was a learning exercise for C++ so inefficiencies will be ironed out in 
//              future versions of the simulator. *Note: Fault Vector Generation will overwrite existing Functional Sim File. 
//
// Table of Contents:
// 		Line 76   :     Enumerated type logic values for the circuit. These logic values are present on all circuit nodes. 
// 		Line 86   :     class Node defines Node objects for the circuit. 
//      Line 130  :     class Component defines base level Component objects for the circuit.
//      Line 157  :     class ComboLogicGate defines a sub-base class of simple Combinatorial gates for the circuit. 
//		Line 170  :     class ANDgate defines the child class for AND gate Component objects. 
//		Line 271  :     class ORgate defines the child class for OR gate Component objects. 
//		Line 373  :     class XORgate defines the child class for XOR gate Component objects. 
//		Line 474  :     class NANDgate defines the child class for NAND gate Component objects.
//		Line 577  :     class NORgate defines the child class for NOR gate Component objects.
//		Line 680  :     class XNORgate defines the child class for XNOR gate Component objects.
//		Line 789  :     class Event defines Event objects for the Event Queue used in simulators. 
//		Line 808  :     struct LessThanTime defines the Event Queue time comparator between Events.
//		Line 820  :     class EventQueue defines Event Queue objects for the simulators. 
//		Line 876  :     class Circuit defines the Circuit object for the simulators. Describes top level connections. 
//		Line 1045 :     Circuit:Function Timing Simulation defines the process of running a timing simulation.
//		Line 1161 :     Circuit:Function Functional Simulation defines the process of running a functional simulation. 
//		Line 1394 :     class FaultVectorGenerator defines the Fault Vector Generator for generating Fault Vectors. 
//		Line 1572 :     main() function (runs program). 
//
// Revision History:
//		May 09, 2023    Hector Wilson      Started assignment 
//		May 15, 2023    Hector Wilson      Completed Node and Component general classes
//      May 20, 2023    Hector Wilson      Completed working Circuit class for constructing Circuit from netlist
//		May 23, 2023    Hector Wilson      Added Event Queue methods and classes for Timing Simulation. 
//		May 30, 2023    Hector Wilson      Has working Timing Simulation (with bugs).
//		June 01, 2023   Hector Wilson      Finished ironing out bugs in timing simulator. (enabled canceling calcations). 
//		June 07, 2023   Hector Wilson      Finished Functional Simulator, Started Fault Vector Generator
//		June 15, 2023   Hector Wilson      Completed Fault Vector Generator. 
//		June 16, 2023   Hector Wilson      Added thorough comments and most test cases to directory. 

#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string.h>
#include <vector>
#include <set>
#include <cstdio>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <time.h>
#include <filesystem>
using namespace std;

// ------------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Circuit Logic Values ------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// Logic Values allowed for the circuit 
// Currently, the simulator only supports bit values 0 and 1. 
// Future versions will incorporate X, U, and Z logic values. 
enum LogicValue { ZERO, ONE, X, U, Z};

// ------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------- Circuit Nodes ----------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// This class implements Node objects for the circuit and simulator. 
// The nodes contain 3 attributes. The name is defined from the netlist
// input by the user. CurValue is the current logic value listed on the 
// Node. The integer StuckAtOp is a control signal indicating if the 
// Node object is a stuck-at node. 
class Node {
public:
	string name;
	LogicValue CurValue;
	int StuckAtOp;
	Node(string x) {
		name = x; // set name of node. 
		CurValue = ZERO; // initialize Node to value 0. 
		StuckAtOp = 0; // disable stuck-at (i.e. allow changes). 
	}

	// This Function reads off the current value of the Node. 
	LogicValue ReadValue() { 
		return CurValue; 
	}

	// This Function makes the Node a Stuck-At-y Node, using the passed value n=y. 
	void MakeStuckAt(LogicValue y) {
		CurValue = y;
		StuckAtOp = 1; // set Stuck-At control op (wont allow future changes). 
	}

	// This Function reverts the Node to a normal (non Stuck-At) Node. 
	void UndoStuckAt() {
		StuckAtOp = 0; // reset Stuck-At control op (will allow future changes).
	}

	// This Function updates the value of the Node. If the Node is Stuck-At,
	// however, then it wont change. 
	void UpdateValue(LogicValue n) {
		// If the Node is Stuck-At dont change. Otherwise, update with passed n. 
		CurValue = (StuckAtOp == 0) ? n : CurValue; 
	}
};

// ------------------------------------------------------------------------------------------------------------------
// -------------------------------------------- Circuit Components --------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// This class implements base Component objects for the circuit and simulator. 
// The components contain the following protected attributes. An arrays of pointers 
// to input Nodes and the string output Node name. A vector of integer logic values 
// the circuit inputs. A vector of strings storing the names of all the input Nodes. 
// Additionally, there is the pointer to the output Node and the integer delay 
// associated with updating this component. 
class Component {
protected:
	Node *inputs[8];   // we have up to 8 pointers to the inputs of each circuit component. 
	string outputName; // Node name on the output of this component
	vector<int> inputValues;
	vector<string> inputNames;
public:
	Node *output;	   // 1 pointer to the output of each circuit component. 
	int delay;		   // integer delay associated with updating this component's output logic value. 
	
	// These functions will have specific definitions depending on the gate being implemented. 
	virtual void Calculate() {};
	virtual int PreCalc() { return 0; }
	virtual vector <string> ReadInputNames() { return {}; }
	virtual string ReadOutputName() { return ""; }
	virtual void RevertOutput() {};
	virtual LogicValue ReadOutput () { return Z; }
	virtual ~Component(void) {};

};

// ------------------------------------------------------------------------------------------------------------------
// This class implements a sub-base class for simple Combinatorial logic gates. These gates only
// have 2 relevant delays associated with each component, the rise time and the fall time delay. 
// At the moment, these are the only types of gates able to used in a circuit for this system. 
// These simple gates contain the following additional protected attributes: Rise and fall time 
// delays associated with each one of these gates. The current and previous output value. 
class ComboLogicGate: public Component {
protected:
	int rise_Time;
	int fall_Time;
	int outputValue = 0;
	int prevoutputValue = 0;
	LogicValue outVal = ZERO;
public: 
};

// ------------------------------------------------------------------------------------------------------------------
// AND gate is special implementation of a combinatorial logic gate. 
// Inline comments below describe all relevant class functions. 
class ANDgate: public ComboLogicGate {
public:
	// Constructor for the AND gate assigns all Node pointers and sets the rise and fall time delays. 
	ANDgate(Node* out, int x, int y, Node* in1, Node* in2, Node* in3, 
			Node* in4, Node* in5, Node* in6, Node* in7, Node* in8) {
		rise_Time = x;
		fall_Time = y;
		inputs[0] = in1; 
		inputs[1] = in2; 
		inputs[2] = in3;
		inputs[3] = in4;
		inputs[4] = in5;
		inputs[5] = in6;
		inputs[6] = in7;
		inputs[7] = in8;
		output = out;

		// Create a vector of input Node names for this component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				inputNames.push_back((inputs[i]->name));
			}
		}
	} 

	// This function is used by the component to update the output value. The function also sets 
	// the delay to the proper parameter (rise or fall time) depending on an single event lookback on 
	// the output Node. 
	void Calculate() {
		prevoutputValue = outputValue;
		int tempoutputValue = 1;
		// This loop does an AND operation on all input Nodes for the component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue & tempoutputValue;
			}
		}

		// Calculate the delay from this result
		// if old value = 0 and new value = 1 then delay is rise time. 
		// if old value = 1 and new value = 0 then delay is fall time.
		if (tempoutputValue == 0 and outputValue == 1) {
			delay = fall_Time;
		}
		else if (tempoutputValue == 1 and outputValue == 0) {
			delay = rise_Time;
		}
		else {
			delay = 0;
		}

		// and set the output
		outputValue = tempoutputValue;
		outVal = (outputValue == 0) ? ZERO : ONE;
	}

	// Special function used to determine if a change in input during calculate will 
	// effect the result. Used by simulator event handler to determine when to cancel
	// events. 
	int PreCalc() {
		int tempoutputValue = 1;
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue & tempoutputValue;
			}
		}
		return (tempoutputValue == outputValue) ? 0 : 1; // if output didn't change return 0, if it did change return 1. 
	}

	// Special function used to revert output of component to its previous value. 
	// This function is only used by the event handler in simulations when canceling
	// an ongoing operation (delay hasn't finished). 
	void RevertOutput() {
		outputValue = prevoutputValue;
	}

	// Function used to return vector of input Node names for this component. 
	vector <string> ReadInputNames() {
		return inputNames;
	}

	// Function used to return the output Node name for this component. 
	string ReadOutputName() {
		return output->name;
	}

	// Function used to return the logic value present on the output of this component. 
	LogicValue ReadOutput () { return outVal; }

	// Destructor 
	~ANDgate(void) {};

};

// ------------------------------------------------------------------------------------------------------------------
// OR gate is special implementation of a combinatorial logic gate. 
// Inline comments below describe all relevant class functions. 
class ORgate: public ComboLogicGate {
public:
	// Constructor for the OR gate assigns all Node pointers and sets the rise and fall time delays. 
	ORgate(Node* out, int x, int y, Node* in1, Node* in2, Node* in3, 
		   Node* in4, Node* in5, Node* in6, Node* in7, Node* in8) {
		rise_Time = x;
		fall_Time = y;
		inputs[0] = in1; 
		inputs[1] = in2; 
		inputs[2] = in3;
		inputs[3] = in4;
		inputs[4] = in5;
		inputs[5] = in6;
		inputs[6] = in7;
		inputs[7] = in8;
		output = out;

		// Create a vector of input Node names for this component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				inputNames.push_back((inputs[i]->name));
			}
		}
	} 

	// This function is used by the component to update the output value. The function also sets 
	// the delay to the proper parameter (rise or fall time) depending on an single event lookback on 
	// the output Node. 
	void Calculate() {
		prevoutputValue = outputValue;
		int tempoutputValue = 0;
		// This loop does an OR operation on all input Nodes for the component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue | tempoutputValue;
			}
		}

		// Calculate the delay from this result// Calculate the delay from this result
		// if old value = 0 and new value = 1 then delay is rise time. 
		// if old value = 1 and new value = 0 then delay is fall time.
		if (tempoutputValue == 0 and outputValue == 1) {
			delay = fall_Time;
		}
		else if (tempoutputValue == 1 and outputValue == 0) {
			delay = rise_Time;
		}
		else {
			delay = 0;
		}

		// and set the output
		outputValue = tempoutputValue;
		outVal = (outputValue == 0) ? ZERO : ONE;
	}

	// Special function used to determine if a change in input during calculate will 
	// effect the result. Used by simulator event handler to determine when to cancel
	// events.
	int PreCalc() {
		int tempoutputValue = 0;
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue | tempoutputValue;
			}
		}

		return (tempoutputValue == outputValue) ? 0 : 1; // if output didn't change return 0, if it did change return 1. 
	}

	// Special function used to revert output of component to its previous value. 
	// This function is only used by the event handler in simulations when canceling
	// an ongoing operation (delay hasn't finished). 
	void RevertOutput() {
		outputValue = prevoutputValue;
	}

	// Function used to return vector of input Node names for this component. 
	vector <string> ReadInputNames() {
		return inputNames;
	}

	// Function used to return the output Node name for this component.
	string ReadOutputName() {
		return output->name;
	}

	// Function used to return the logic value present on the output of this component. 
	LogicValue ReadOutput () { return outVal; }

	// Destructor
	~ORgate(void) {};

};

// ------------------------------------------------------------------------------------------------------------------
// XOR gate is special implementation of a combinatorial logic gate. 
// Inline comments below describe all relevant class functions. 
class XORgate: public ComboLogicGate {
public:
	// Constructor for the XOR gate assigns all Node pointers and sets the rise and fall time delays. 
	XORgate(Node* out, int x, int y, Node* in1, Node* in2, Node* in3, 
			Node* in4, Node* in5, Node* in6, Node* in7, Node* in8) {
		rise_Time = x;
		fall_Time = y;
		inputs[0] = in1; 
		inputs[1] = in2; 
		inputs[2] = in3;
		inputs[3] = in4;
		inputs[4] = in5;
		inputs[5] = in6;
		inputs[6] = in7;
		inputs[7] = in8;
		output = out;

		// Create a vector of input Node names for this component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				inputNames.push_back((inputs[i]->name));
			}
		}
	} 

	// This function is used by the component to update the output value. The function also sets 
	// the delay to the proper parameter (rise or fall time) depending on an single event lookback on 
	// the output Node. 
	void Calculate() {
		prevoutputValue = outputValue;
		int tempoutputValue = 0;
		// This loop does an XOR operation on all input Nodes for the component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue ^ tempoutputValue;
			}
		}

		// Calculate the delay from this result// Calculate the delay from this result
		// if old value = 0 and new value = 1 then delay is rise time. 
		// if old value = 1 and new value = 0 then delay is fall time.
		if (tempoutputValue == 0 and outputValue == 1) {
			delay = fall_Time;
		}
		else if (tempoutputValue == 1 and outputValue == 0) {
			delay = rise_Time;
		}
		else {
			delay = 0;
		}

		// and set the output
		outputValue = tempoutputValue;
		outVal = (outputValue == 0) ? ZERO : ONE;
	}

	// Special function used to determine if a change in input during calculate will 
	// effect the result. Used by simulator event handler to determine when to cancel
	// events.
	int PreCalc() {
		int tempoutputValue = 0;
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue ^ tempoutputValue;
			}
		}

		return (tempoutputValue == outputValue) ? 0 : 1; // if output didn't change return 0, if it did change return 1. 
	}

	// Special function used to revert output of component to its previous value. 
	// This function is only used by the event handler in simulations when canceling
	// an ongoing operation (delay hasn't finished). 
	void RevertOutput() {
		outputValue = prevoutputValue;
	}

	// Function used to return vector of input Node names for this component. 
	vector <string> ReadInputNames() {
		return inputNames;
	}

	// Function used to return the output Node name for this component.
	string ReadOutputName() {
		return output->name;
	}

	// Function used to return the logic value present on the output of this component. 
	LogicValue ReadOutput () { return outVal; }

	// Destructor
	~XORgate(void) {};
};

// ------------------------------------------------------------------------------------------------------------------
// NAND gate is special implementation of a combinatorial logic gate. 
// Inline comments below describe all relevant class functions. 
class NANDgate: public ComboLogicGate {
public: 
	// Constructor for the NAND gate assigns all Node pointers and sets the rise and fall time delays. 
	NANDgate(Node* out, int x, int y, Node* in1, Node* in2, Node* in3, 
			 Node* in4, Node* in5, Node* in6, Node* in7, Node* in8) {
		rise_Time = x;
		fall_Time = y;
		inputs[0] = in1; 
		inputs[1] = in2; 
		inputs[2] = in3;
		inputs[3] = in4;
		inputs[4] = in5;
		inputs[5] = in6;
		inputs[6] = in7;
		inputs[7] = in8;
		output = out;

		// Create a vector of input Node names for this component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				inputNames.push_back((inputs[i]->name));
			}
		}
	} 

	// This function is used by the component to update the output value. The function also sets 
	// the delay to the proper parameter (rise or fall time) depending on an single event lookback on 
	// the output Node. 
	void Calculate() {
		prevoutputValue = outputValue;
		int tempoutputValue = 1;
		// This loop does an AND operation on all input Nodes for the component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue & tempoutputValue;
			}
		}
		tempoutputValue = (tempoutputValue == 1) ? 0 : 1; // not the output of the AND gate for NAND

		// Calculate the delay from this result// Calculate the delay from this result
		// if old value = 0 and new value = 1 then delay is rise time. 
		// if old value = 1 and new value = 0 then delay is fall time.
		if (tempoutputValue == 0 and outputValue == 1) {
			delay = fall_Time;
		}
		else if (tempoutputValue == 1 and outputValue == 0) {
			delay = rise_Time;
		}
		else {
			delay = 0;
		}

		// and set the output
		outputValue = tempoutputValue;
		outVal = (outputValue == 0) ? ZERO : ONE;
	}

	// Special function used to determine if a change in input during calculate will 
	// effect the result. Used by simulator event handler to determine when to cancel
	// events.
	int PreCalc() {
		int tempoutputValue = 1;
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue & tempoutputValue;
			}
		}
		tempoutputValue = (tempoutputValue == 1) ? 0 : 1; // not the output of the AND gate for NAND

		return (tempoutputValue == outputValue) ? 0 : 1; // if output didn't change return 0, if it did change return 1. 
	}

	// Special function used to revert output of component to its previous value. 
	// This function is only used by the event handler in simulations when canceling
	// an ongoing operation (delay hasn't finished). 
	void RevertOutput() {
		outputValue = prevoutputValue;
	}

	// Function used to return vector of input Node names for this component. 
	vector <string> ReadInputNames() {
		return inputNames;
	}

	// Function used to return the output Node name for this component.
	string ReadOutputName() {
		return output->name;
	}

	// Function used to return the logic value present on the output of this component. 
	LogicValue ReadOutput () { return outVal; }

	// Destructor
	~NANDgate(void) {};
};

// ------------------------------------------------------------------------------------------------------------------
// NOR gate is special implementation of a combinatorial logic gate. 
// Inline comments below describe all relevant class functions. 
class NORgate: public ComboLogicGate {
public:
	// Constructor for the NOR gate assigns all Node pointers and sets the rise and fall time delays. 
	NORgate(Node* out, int x, int y, Node* in1, Node* in2, Node* in3, 
		    Node* in4, Node* in5, Node* in6, Node* in7, Node* in8) {
		rise_Time = x;
		fall_Time = y;
		inputs[0] = in1; 
		inputs[1] = in2; 
		inputs[2] = in3;
		inputs[3] = in4;
		inputs[4] = in5;
		inputs[5] = in6;
		inputs[6] = in7;
		inputs[7] = in8;
		output = out;

		// Create a vector of input Node names for this component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				inputNames.push_back((inputs[i]->name));
			}
		}
	} 

	// This function is used by the component to update the output value. The function also sets 
	// the delay to the proper parameter (rise or fall time) depending on an single event lookback on 
	// the output Node. 
	void Calculate() {
		prevoutputValue = outputValue;
		int tempoutputValue = 0;
		// This loop does an OR operation on all input Nodes for the component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue | tempoutputValue;
			}
		}
		tempoutputValue = (tempoutputValue == 1) ? 0 : 1; // not the output of the OR gate for NOR

		// Calculate the delay from this result// Calculate the delay from this result
		// if old value = 0 and new value = 1 then delay is rise time. 
		// if old value = 1 and new value = 0 then delay is fall time.
		if (tempoutputValue == 0 and outputValue == 1) {
			delay = fall_Time;
		}
		else if (tempoutputValue == 1 and outputValue == 0) {
			delay = rise_Time;
		}
		else {
			delay = 0;
		}

		// and set the output
		outputValue = tempoutputValue;
		outVal = (outputValue == 0) ? ZERO : ONE;
	}

	// Special function used to determine if a change in input during calculate will 
	// effect the result. Used by simulator event handler to determine when to cancel
	// events.
	int PreCalc() {
		int tempoutputValue = 0;
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue | tempoutputValue;
			}
		}
		tempoutputValue = (tempoutputValue == 1) ? 0 : 1; // not the output of the AND gate for NOR

		return (tempoutputValue == outputValue) ? 0 : 1; // if output didn't change return 0, if it did change return 1. 
	}

	// Special function used to revert output of component to its previous value. 
	// This function is only used by the event handler in simulations when canceling
	// an ongoing operation (delay hasn't finished). 
	void RevertOutput() {
		outputValue = prevoutputValue;
	}

	// Function used to return vector of input Node names for this component. 
	vector <string> ReadInputNames() {
		return inputNames;
	}

	// Function used to return the output Node name for this component.
	string ReadOutputName() {
		return output->name;
	}

	// Function used to return the logic value present on the output of this component. 
	LogicValue ReadOutput () { return outVal; }

	// Destructor
	~NORgate(void) {};
};

// ------------------------------------------------------------------------------------------------------------------
// XNOR gate is special implementation of a combinatorial logic gate. 
// Inline comments below describe all relevant class functions. 
class XNORgate: public ComboLogicGate {
public:
	// Constructor for the XNOR gate assigns all Node pointers and sets the rise and fall time delays. 
	XNORgate(Node* out, int x, int y, Node* in1, Node* in2, Node* in3, 
			 Node* in4, Node* in5, Node* in6, Node* in7, Node* in8) {
		rise_Time = x;
		fall_Time = y;
		inputs[0] = in1; 
		inputs[1] = in2; 
		inputs[2] = in3;
		inputs[3] = in4;
		inputs[4] = in5;
		inputs[5] = in6;
		inputs[6] = in7;
		inputs[7] = in8;
		output = out;

		// Create a vector of input Node names for this component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				inputNames.push_back((inputs[i]->name));
			}
		}
	} 

	// This function is used by the component to update the output value. The function also sets 
	// the delay to the proper parameter (rise or fall time) depending on an single event lookback on 
	// the output Node. 
	void Calculate() {
		prevoutputValue = outputValue;
		int tempoutputValue = 0;
		// This loop does an XOR operation on all input Nodes for the component. 
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue ^ tempoutputValue;
			}
		}
		tempoutputValue = (tempoutputValue == 1) ? 0 : 1; // not the output of XOR gate for XNOR

		// Calculate the delay from this result// Calculate the delay from this result
		// if old value = 0 and new value = 1 then delay is rise time. 
		// if old value = 1 and new value = 0 then delay is fall time.
		if (tempoutputValue == 0 and outputValue == 1) {
			delay = fall_Time;
		}
		else if (tempoutputValue == 1 and outputValue == 0) {
			delay = rise_Time;
		}
		else {
			delay = 0;
		}

		// and set the output
		outputValue = tempoutputValue;
		outVal = (outputValue == 0) ? ZERO : ONE;
	}

	// Special function used to determine if a change in input during calculate will 
	// effect the result. Used by simulator event handler to determine when to cancel
	// events.
	int PreCalc() {
		int tempoutputValue = 0;
		for (int i=0; i < 8; i++) {
			if (inputs[i] != NULL) {
				int nodeValue = ((inputs[i]->ReadValue()) == ZERO) ? 0 : 1;
				inputValues.push_back(nodeValue);
				tempoutputValue = nodeValue ^ tempoutputValue;
			}
		}
		tempoutputValue = (tempoutputValue == 1) ? 0 : 1; // not the output of the AND gate for XNOR

		return (tempoutputValue == outputValue) ? 0 : 1; // if output didn't change return 0, if it did change return 1. 
	}

	// Special function used to revert output of component to its previous value. 
	// This function is only used by the event handler in simulations when canceling
	// an ongoing operation (delay hasn't finished). 
	void RevertOutput() {
		outputValue = prevoutputValue;
	}

	// Function used to return vector of input Node names for this component. 
	vector <string> ReadInputNames() {
		return inputNames;
	}

	// Function used to return the output Node name for this component.
	string ReadOutputName() {
		return output->name;
	}

	// Function used to return the logic value present on the output of this component. 
	LogicValue ReadOutput () { return outVal; }

	// Destructor
	~XNORgate(void) {};
};

// ------------------------------------------------------------------------------------------------------------------
// --------------------------------------------- EVENT QUEUE OBJECTS ------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// This class implements Event objects for the event queue used by the simulators in this system. 
// Each Event object contains the following attributes: A pointer to a Component and a pointer to a Node. 
// Depending on the type of event (update Node or update Component), the opposite type pointer will be NULL. 
// Integer eventTime is the time at which the event should be executed in the circuit. The next value of the Node
// being updated by this event (only matters for Node events). And lastly, an additional indicator CN = 0 when  
// the event is a Component update and CN = 1 when the event is a Node update. 
class Event {
public:
	Component *eventComp;
	Node *eventNode;
	int eventTime;
	LogicValue nextVal;
	int CompNode; // Comp = 0, Node = 1 
	Event(Component* x, Node* y, int t, LogicValue z, int CN) {
		eventTime = t;
		eventComp = x;
		eventNode = y;
		nextVal = z;
		CompNode = CN;
	}
};

// ------------------------------------------------------------------------------------------------------------------
// This struct implements the comparator for the priority queue data structure used to define Event Queues. The
// comparator will organize the priority queue based on the value of the eventTime attribute in each Event object. 
struct LessThanTime
{
  bool operator()(const Event& lhs, const Event& rhs) const
  {
    return lhs.eventTime > rhs.eventTime;
  }
};

// ------------------------------------------------------------------------------------------------------------------
// This class implements the Event Queue for the system simulators. 
// The Event Queue is implemented as a priority queue with a custom comparator defined above LessThanTime. 
// The Event Queue has a total of 4 associated functions defined below. 
class EventQueue {
public:
	priority_queue<Event, vector<Event>, LessThanTime> PQ;

	// This function adds a passed Event to the Event Queue. 
	void Append(Event x) {
		PQ.push(x);
	}

	// This function removes the top Event from the Event Queue. 
	void Pop() {
		PQ.pop();
	}

	// This function deletes all Events in the queue pertaining to an input Node 
	// It is used when the simulator has to cancel a Node update before the delay has completed. 
	void Delete(Node* x, Component* y) {
		priority_queue<Event, vector<Event>, LessThanTime> tempPQ; // temporary Queue to be modified
		priority_queue<Event, vector<Event>, LessThanTime> newPQ; // new Event Queue being made clean
		tempPQ = PQ;

		// Only add Events to the new Event Queue if they dont have the given rotten input Node. 
		while (!tempPQ.empty()) {
			// if next Event doesnt contain passed input Node, then add it to the new Event Queue
			if (tempPQ.top().eventNode != x) {
				newPQ.push(tempPQ.top());
			}
			// if the next Event does contain passed input Node, then dont add it to the new Event Queue
			// and additionally revert the output of the passed component.
			else {
				// Revert the output of the gate if the queue contained a rotten node update
				y->RevertOutput();
			}

			tempPQ.pop();
		}

		// Store the clean Event Queue back into PQ. 
		PQ = newPQ;
	}

	// This function returns the next Event to be executed in the Event Queue. 
	Event Top() {
		return PQ.top();
	}
};

// ------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- CIRCUIT ------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------
// This very important class (VIC) implements the circuit object used by both simulator types as well as the fault 
// vector generator. A circuit objects is responsible for establishing all top level connections between Nodes, 
// Components, as well as inputs/outputs to the circuit. These top level connections are defined by the netlist 
// fed into the program at the start. A Circuit object also contains the functions which define the operation 
// of the system. That is, we define below functions for running Timing Simulations, Functional Simulations, and
// Fault Vector Generation on a Circuit object. 
class Circuit {
private:
	// Circuit objects contain the following private attributes:
	string netlist;                // user-input netlist detailing top-level circuit connections
	Component **comps = NULL;      // array of pointers to Component objects
	set<Node*> nodes;			   // set of pointers to Node objects
	set<Node*> outputnodes;        // set of pointers to output Node objects
	set<Node*> inputnodes;		   // set of pointers to input Node objects
	set<string> nodeNames;         // set of strings of input Node names
	EventQueue queue;              // the Event Queue for the Circuit
  
	int compCnt=0;				   // the number of Components in the Circuit
	int nodeCnt=0;				   // the number of Nodes in the Circuit
public:
	// The constructor for the Circuit object is passed a string netlist file. The constructor opens the file
	// and creates the Circuit passed on the netlist. 
	Circuit(string z) {
		netlist = z;

		// BEGIN READING NETLIST FILE
		string line;
		ifstream MyReadFile(z);
		while (getline (MyReadFile,line)) {
			// Allocate +1 space in Component array
			Component **tempcomps = new Component*[compCnt+1];
			// Add all previous Components in old array to new array
			for (int j=0; j<compCnt; ++j) {
				tempcomps[j] = comps[j];
			}

			// The input netlist has the following format to be read:
			// OUTPUT   .COMPONENT    (DELAYS)    INPUT1     INPUT2    ....
			stringstream linestream(line);

			string out;      // 1 output
			string compType; // component type
			string ins[8];   // up to 8 inputs
			int delay[11];   // up to 11 delay parameters 
			linestream >> out >> compType;

			Node **inputPtr = NULL;		// array of input pointers used to create Components
			inputPtr = new Node*[8];    // array should have length 8 (8 is max # of inputs for a Component)

			Node *o = new Node(out);	// Create new Node for the output of this line (each line will have a brand new output Node)
			nodes.insert(o);            // insert new Node to relevant data sets 
			nodeNames.insert(out);

			// read in the rest of the data from the line
			linestream >> delay[0] >> delay[1] >> ins[0] >> ins[1] 
					   >> ins[2] >> ins[3] >> ins[4] >> ins[5] >> ins[6] >> ins[7];
			// This for loop determines if an input Node already exists and sets up the appropriate pointers
			// to be used to create the Component object of this netlist line. 
			for (int i=0; i < 8; i++) {
				if (!ins[i].empty()) {
					// Node already exists --> use old pointer
					if (nodeNames.find(ins[i]) != nodeNames.end()) {
						for(set<Node*>::iterator k = nodes.begin();k!=nodes.end();k++)
				    	{
				        	if ((*k)->name == ins[i]) {
				        		inputPtr[i] = *k;
				        	}
				    	}
					}
					// Node doesn't already exist --> create New Node
					else {
						inputPtr[i] = new Node(ins[i]);
						nodes.insert(inputPtr[i]);
						nodeNames.insert(ins[i]);						
					}
				}
				// If the Component doesn't user up all 8 inputs then set those input pointers to NULL
				else{
					inputPtr[i] = NULL;
				}
			}

			// Determine the type of Component being made and create the Component with proper Node pointers and delay values. 
			if (compType.compare(".OR") == 0) {
				ORgate *q = new ORgate(o, delay[0], delay[1], inputPtr[0], inputPtr[1], inputPtr[2], 
				                       inputPtr[3], inputPtr[4], inputPtr[5], inputPtr[6], inputPtr[7]);
				tempcomps[compCnt] = q;
			}
			else if (compType.compare(".AND") == 0) {
				ANDgate *q = new ANDgate(o, delay[0], delay[1], inputPtr[0], inputPtr[1], inputPtr[2], 
					                     inputPtr[3], inputPtr[4], inputPtr[5], inputPtr[6], inputPtr[7]);
				tempcomps[compCnt] = q;
			}
			else if (compType.compare(".XOR") == 0) {
				XORgate *q = new XORgate(o, delay[0], delay[1], inputPtr[0], inputPtr[1], inputPtr[2], 
					                     inputPtr[3], inputPtr[4], inputPtr[5], inputPtr[6], inputPtr[7]);
				tempcomps[compCnt] = q;
			}
			else if (compType.compare(".NOR") == 0) {
				NORgate *q = new NORgate(o, delay[0], delay[1], inputPtr[0], inputPtr[1], inputPtr[2], 
					                     inputPtr[3], inputPtr[4], inputPtr[5], inputPtr[6], inputPtr[7]);
				tempcomps[compCnt] = q;
			}
			else if (compType.compare(".NAND") == 0) {
				NANDgate *q = new NANDgate(o, delay[0], delay[1], inputPtr[0], inputPtr[1], inputPtr[2], 
					                       inputPtr[3], inputPtr[4], inputPtr[5], inputPtr[6], inputPtr[7]);
				tempcomps[compCnt] = q;
			}
			else if (compType.compare(".XNOR") == 0) {
				XNORgate *q = new XNORgate(o, delay[0], delay[1], inputPtr[0], inputPtr[1], inputPtr[2], 
					                       inputPtr[3], inputPtr[4], inputPtr[5], inputPtr[6], inputPtr[7]);
				tempcomps[compCnt] = q;
			}

			// Increase the Component count by 1 after reading each line of a netlist. 
			++compCnt;
			// Delete old Component array
			delete[] comps;
			// And set it to new Component array
			comps = tempcomps;
		};
		// Call the FindIOs function to determine which nodes are inputs/outputs to the netlist. 
		FindIOs();

		// Close File. 
		MyReadFile.close();
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This Function runs to determine which nodes in the circuit are inputs and outputs (IOs). It adds these nodes
	// to seperate sets containing input node pointers and output node pointers. 
	void FindIOs() {
		// Determine which Nodes are inputs/outputs to the netlist and add them to the list of input/output node pointers. 
		// start by looping through each node. 
		for (set<string>::iterator i = nodeNames.begin(); i != nodeNames.end(); i++) {
			int input_occurances = 0;
			int output_occurances = 0;
			// loop through each component and see if the node is an output or input to the component. 
			for (int j=0; j<compCnt; j++) {
				// Check if Node is an output to this component. 
				string compOutputName = comps[j]->ReadOutputName();
				if ((*i) == compOutputName) {
					output_occurances += 1;
				}
				// Check if Node is an input to this component. 
				vector<string> compInputNames = comps[j]->ReadInputNames();
				for (vector<string>::iterator k = compInputNames.begin(); k != compInputNames.end(); k++) {
					if ((*k) == (*i)) {
						input_occurances += 1; 
					}
				}
			}
			// If the Node never appeared as an output to a component then it is an input. 
			if (output_occurances == 0) {
				for (set<Node*>::iterator k = nodes.begin(); k != nodes.end(); k++) {
					if ((*k)->name == (*i)) {
						inputnodes.insert(*k);
					}
				}
			}
			// If the Node never appeared as an input to a component then it is an output. 
			if (input_occurances == 0) {
				for (set<Node*>::iterator k = nodes.begin(); k != nodes.end(); k++) {
					if ((*k)->name == (*i)) {
						outputnodes.insert(*k);
					}
				}
			}
		}
	}

	// --------------------------------------------- TIMING SIMULATION --------------------------------------------------
	// This Function runs a Timing Simulation on the Circuit. It takes in as an argument the input file as a string.
	// The Timing Simulator uses an Event Queuing system to determine the order of Events to execute wherein Events 
	// are queued by realistic execution times based on their associated delays.  
	void TimingSimulation(string z) {
		cout << "Starting Timing Simulation" << endl;

		string line;
		int inputTime = 0;
		ifstream InputFile(z);
		ofstream OutputFile("TimingSimOutput.txt");
		OutputFile << "This file contains the Timing Simulation output:" << endl;

		// first output initial node values (all 0) to the output file at time 0 
		for (set<Node*>::iterator i = nodes.begin(); i != nodes.end(); i++) {
			OutputFile << (*i)->name << " " << 0 << " " << (*i)->CurValue << endl;
		}


		// if we have any NAND, NOR, XNOR gates start by sending these to the event queue at time 0
		for (int i=0; i < compCnt; i++) {
			comps[i]->Calculate();
			int delay = comps[i]->delay;

			if (delay != 0) {
				queue.Append(Event(NULL, comps[i]->output, delay, comps[i]->ReadOutput(), 1));
			}
		}

		// Add user defined inputs into the event queue
		while (getline (InputFile,line)) {
			stringstream linestream(line);
			string input;
			string newVals;
			LogicValue newVal;
			float time; 
			linestream >> time >> input >> newVals;
			if (newVals == "0") { 
				newVal = ZERO;
			}
			else if (newVals == "1") { 
				newVal = ONE;
			}
			else {
				newVal = Z;
			}

			// Add changes in input nodes to the event queue
			inputTime = time; 
			for (set<Node*>::iterator i = nodes.begin(); i != nodes.end(); i++) {
				if ((*i)->name == input) {
					queue.Append(Event(NULL, (*i), inputTime, newVal, 1));
				}
			}		
		}


		// start executing event queue for this circuit
		while (!(queue.PQ).empty()) {
			Event nextEvent = queue.Top();

			// if next in Event Queue is a Node then we update the node value and write the change to the output file. 
			if (nextEvent.CompNode == 1) {
				nextEvent.eventNode->UpdateValue(nextEvent.nextVal);

				// *****  Write to the output file the change. ***** 
        		OutputFile << nextEvent.eventNode->name << " " << nextEvent.eventTime << " " << nextEvent.eventNode->CurValue << endl;


        		// Additionally, if any gates use this Node as an input then we must add them to the Event Queue. 
        		for (int k=0; k < compCnt; k++) {
					vector<string>  names = (comps[k]->ReadInputNames());
					for (vector<string>::iterator s = names.begin(); s != names.end(); s++) {
						if ((*s) == nextEvent.eventNode->name) {
							// Add all components connected to the changing Node to the Event queue
							//
							// We check if this input node change is expected to change the output of any gate its connected to. 
							// If it will, then we cancel the all pending gate changes already in queue, revert the gate, and add the gate
							// change to the event queue. 
							//
							// If the input node change will not change the output of an already pending gate change event then it overrides 
							// this event and we do nothing.  
							if (comps[k]->PreCalc() == 1) {
								queue.Delete(comps[k]->output, comps[k]);
								queue.Append(Event(comps[k], NULL, nextEvent.eventTime, Z, 0));
							}
							break;
						}
					}
				}
			}
			// if next in Event Queue is a Component then calculate Component and compute delay
			else if (nextEvent.CompNode == 0) {
				nextEvent.eventComp->Calculate();
				int delay = nextEvent.eventComp->delay;

				// If the output to the gate changed (delay != 0) then we add the output node to the Event Queue
				// to update its value.
				if (delay != 0) {
					queue.Append(Event(NULL,nextEvent.eventComp->output, nextEvent.eventTime+delay, 
									   nextEvent.eventComp->ReadOutput(), 1));
				}
			}
			queue.Pop();
		}

		OutputFile.close();

		// Output final node values to the terminal
		cout << "Final Node Values:" << endl;
		for(set<Node*>::iterator k = nodes.begin();k!=nodes.end();k++)
	    {
	        cout << (*k)->name << " " << (*k)->CurValue << endl;
	    }
	}

	// -------------------------------------------- FUNCTIONAL SIMULATION --------------------------------------------------
	// This Function runs a Functional Simulation on the Circuit. It takes in as an argument the input file as a string. 
	// Like the Timing Simulation, the Functional Simulation also uses an Event Queueing system but this time, all
	// Events originating off a changing input will happen at the same time. i.e. there is 0 delay for all Component updates. 
	void FunctionalSimulation(string z) {
		string line;
		int inputTime;
		ifstream InputFile(z);
		ofstream OutputFile("FunctionalSimOutput.txt");
		OutputFile << "This file contains the Functional Simulation output:" << endl;

		// Calculate initial state: start by processing any NAND, NOR, or XNOR gates
		//
		//
		// send any NAND, NOR, XNOR gates to the event queue at time 0
		for (int i=0; i < compCnt; i++) {
			comps[i]->Calculate();
			int delay = comps[i]->delay;

			if (delay != 0) {
				queue.Append(Event(NULL, comps[i]->output, 0, comps[i]->ReadOutput(), 1));
			}
		}

		// Process all of the NAND, NOR, XNOR gates in queue. 
		while (!(queue.PQ).empty()) {
			Event nextEvent = queue.Top();

			// if next in Event Queue is a Node then we update the node value and write the change to the output file. 
			if (nextEvent.CompNode == 1) {
				nextEvent.eventNode->UpdateValue(nextEvent.nextVal);

        		// Additionally, if any gates use this Node as an input then we must add them to the Event Queue. 
        		for (int k=0; k < compCnt; k++) {
					vector<string>  names = (comps[k]->ReadInputNames());
					for (vector<string>::iterator s = names.begin(); s != names.end(); s++) {
						if ((*s) == nextEvent.eventNode->name) {
							// Add all components connected to the changing Node to the Event queue
							//
							// We check if this input node change is expected to change the output of any gate its connected to. 
							// If it will, then we cancel the all pending gate changes already in queue, revert the gate, and add the gate
							// change to the event queue. 
							//
							// If the input node change will not change the output of an already pending gate change event then it overrides 
							// this event and we do nothing.  
							if (comps[k]->PreCalc() == 1) {
								queue.Delete(comps[k]->output, comps[k]);
								queue.Append(Event(comps[k], NULL, nextEvent.eventTime, Z, 0));
							}
							break;
						}
					}
				}
			}
			// if next in Event Queue is a Component then calculate Component and compute delay
			else if (nextEvent.CompNode == 0) {
				nextEvent.eventComp->Calculate();
				int delay = nextEvent.eventComp->delay;

				// If the output to the gate changed (delay != 0) then we add the output node to the Event Queue
				// to update its value. For functional simulation, however, we do not add a delay to event time. 
				if (delay != 0) {
					queue.Append(Event(NULL,nextEvent.eventComp->output, nextEvent.eventTime, nextEvent.eventComp->ReadOutput(), 1));
				}
			}
			queue.Pop();
		}

		// Done calculating initial state: 
		// next output initial node values to the output file at time 0
		for (set<Node*>::iterator i = nodes.begin(); i != nodes.end(); i++) {
			OutputFile << (*i)->name << " " << 0 << " " << (*i)->CurValue << endl;
		}

		// Finished calculating initial state. Begin Functional Simulation:
		// Add user defined inputs into the event queue
		while (getline (InputFile,line)) {
			stringstream linestream(line);
			string input;
			string newVals;
			LogicValue newVal;
			float time; 
			linestream >> time >> input >> newVals;
			if (newVals == "0") { 
				newVal = ZERO;
			}
			else if (newVals == "1") { 
				newVal = ONE;
			}
			else {
				newVal = Z;
			}

			// Add changes in input nodes to the event queue
			inputTime = time; 
			for (set<Node*>::iterator i = nodes.begin(); i != nodes.end(); i++) {
				if ((*i)->name == input) {
					queue.Append(Event(NULL, (*i), inputTime, newVal, 1));
				}
			}		
		}


		// start executing event queue for this circuit
		while (!(queue.PQ).empty()) {
			Event nextEvent = queue.Top();

			// if next in Event Queue is a Node then we update the node value and write the change to the output file. 
			if (nextEvent.CompNode == 1) {
				nextEvent.eventNode->UpdateValue(nextEvent.nextVal);
        		OutputFile << nextEvent.eventNode->name << " " << nextEvent.eventTime << " " << nextEvent.eventNode->CurValue << endl;


        		// Additionally, if any gates use this Node as an input then we must add them to the Event Queue. 
        		for (int k=0; k < compCnt; k++) {
					vector<string>  names = (comps[k]->ReadInputNames());
					for (vector<string>::iterator s = names.begin(); s != names.end(); s++) {
						if ((*s) == nextEvent.eventNode->name) {
							// Add all components connected to the changing Node to the Event queue
							//
							// We check if this input node change is expected to change the output of any gate its connected to. 
							// If it will, then we cancel the all pending gate changes already in queue, revert the gate, and add the gate
							// change to the event queue. 
							//
							// If the input node change will not change the output of an already pending gate change event then it overrides 
							// this event and we do nothing.  
							if (comps[k]->PreCalc() == 1) {
								queue.Delete(comps[k]->output, comps[k]);
								queue.Append(Event(comps[k], NULL, nextEvent.eventTime, Z, 0));
							}
							break;
						}
					}
				}
			}
			// if next in Event Queue is a Component then calculate Component and compute delay
			else if (nextEvent.CompNode == 0) {
				nextEvent.eventComp->Calculate();
				int delay = nextEvent.eventComp->delay;

				// If the output to the gate changed (delay != 0) then we add the output node to the Event Queue
				// to update its value. For functional simulation, however, we do not add a delay to event time. 
				if (delay != 0) {
					queue.Append(Event(NULL,nextEvent.eventComp->output, nextEvent.eventTime, nextEvent.eventComp->ReadOutput(), 1));
				}
			}
			queue.Pop();
		}

		OutputFile.close();
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This Function changes the passed Node into a stuck-at-y Node where y is passed in. 
	void CreateStuckAt(string x, LogicValue y) {
		// Change passed Node to a Stuck-At-y Node. 
		// loop through all nodes in the circuit
		for(set<Node*>::iterator k = nodes.begin(); k != nodes.end(); k++) {
			// if node has same name as passed node then turn it into a stuck-at-y node
	    	if ((*k)->name == x) {
	    		(*k)->MakeStuckAt(y); 
	    	}
	    }
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This helper Function just returns the node names set from the circuit. 
	set<string> CircuitNodeNames() {
		return nodeNames;
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This helper Function returns a set of input names from the circuit. 
	set<string> CircuitInputNames() {
		set<string> names;
		for(set<Node*>::iterator k = inputnodes.begin(); k != inputnodes.end(); k++) {
	        names.insert((*k)->name);
	    }
	    return names;
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This helper function returns a vector of tuples containing input Node (names, logic value) for the Circuit. 
	vector<tuple<string,int>> CircuitInputs() {
		vector<tuple<string, int>> inputs; 
		for (set<Node*>::iterator i = inputnodes.begin(); i != inputnodes.end(); i++) {
			inputs.push_back(make_tuple((*i)->name, (*i)->CurValue));
		}
		return inputs;
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This helper function returns a vector of tuples containing input Node (names, logic value) for the Circuit. 
	vector<tuple<string,int>> CircuitOutputs() {
		vector<tuple<string, int>> outputs; 
		for (set<Node*>::iterator i = outputnodes.begin(); i != outputnodes.end(); i++) {
			outputs.push_back(make_tuple((*i)->name, (*i)->CurValue));
		}
		return outputs;
	}

	// ------------------------------------------------------------------------------------------------------------------
	// This Function deletes all the components and nodes allocated for this circuit. 
	void Delete() {
		// DELETE COMPONENTS
		for (int i = 0; i < compCnt; i++) {
			delete comps[i];
		}

		// DELETE NODES
		for (set<Node*>::iterator i = nodes.begin(); i != nodes.end(); i++) {
			delete (*i);
		}

	}

	// ------------------------------------------------------------------------------------------------------------------
	// Destructor for the Circuit
	~Circuit(void) { 
		Delete();
	}
};

// ----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------- FAULT VECTOR GENERATOR ---------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------
// This class implements the Fault Vector Generator for the system. The Fault Vector Generator is constructed with a 
// passed string netlist file. It creates the Circuit object from that netlist as well as additional faulty stuck-at 
// circuits. For each Node defined in the netlist there will be 2 possible stuck-ats and thus 2 associated faulty Circuits. 
// The Fault Vector Generator runs randomly generated test vectors on all of these Circuits and detects faults by comparing 
// the outputs of the "Good" Circuit to the outputs of the "Faulty" Circuits. A fault is detected if the outputs differ. 
// The generator operates on a trial-and-error basis. On each trial, it runs a number of test vectors on the Good and Faulty
// Circuits. The number of tests per trial is determined by the number of remaining faults to be detected. After running all 
// of the tests for a trial, the generator determines which test vector had the best coverage on the remaining faults and 
// adds it to the list of Fault Vectors. All of the faulty Circuits detected by this max coverage test vector are then 
// removed before running the next trial. This process continues until the user-input coverage (% faults detected) is satisfied. 
//. 
class FaultVectorGenerator {
private:
	set<Circuit*> faultyCircuits;
	Circuit* GoodCircuit;
public:
	// The Constructor for the FaultVectorGenerator takes a string input netlist from 
	// the user and creates circuits for the generator. One correct circuit is created 
	// along with 2*(# of nodes) faulty circuits each with a single stuck-at-1/0 node. 
	FaultVectorGenerator(string x) {
		// Create a Good (No Fault) Circuit
		Circuit *GoodCircuit = new Circuit(x);
		// Grab all node names (including inputs/outputs) for the circuit
		set<string> allNodeNames = GoodCircuit->CircuitNodeNames();

		// Next, create faulty circuit(s)
		// loop through each node in the circuit
		for (set<string>::iterator j = allNodeNames.begin(); j != allNodeNames.end(); j++) {
			// create faulty node stuck-at-0 circuit
			Circuit *sa0circuit = new Circuit(x);
			sa0circuit->CreateStuckAt((*j), ZERO);
			faultyCircuits.insert(sa0circuit);

			// create faulty node stuck-at-1 circuit 
			Circuit *sa1circuit = new Circuit(x);
			sa1circuit->CreateStuckAt((*j), ONE);
			faultyCircuits.insert(sa1circuit);
		}
	}

	// This Function takes as an integer input (0-100), the amount of coverage % requested
	// by the user and generates a set of test vectors which achieves that coverage. The 
	// generated vectors are written to the file TestVectorOutput.txt in the directory the
	// program is being run from. 
	// The program has a set bin size of 20 runs in each of which a test vector is generated and
	// run on all of the faulty circuits. After 10 runs/vectors are tested, the vector providing
	// the most coverage is selected and written to the file. The number of remaining faults is 
	// updated before repeating the same process over again. 
	void Generate(double x) {
		// required minimum test vector coverage = x
		double required_coverage = x/double(100);
		double total_coverage = 0;
		int total_faults = size(faultyCircuits);
		int vector_cnt = 0;

		// Grab input names for the circuit
		set<string> inputNames = GoodCircuit->CircuitInputNames();

		ofstream TestVectorOutput("FaultVectors.txt");
		TestVectorOutput << "This file contains a set of test vectors providing " << required_coverage*100 << 
							"% fault coverage on the given circuit: " << endl;

		while ((required_coverage - total_coverage) > 0.001) {
			// # of test vector cases to run this trial is equal to the number of the remaining faults left to 
			// detect for the circuit. This way, we only run 1 test vector per trial when there is 1 fault 
			// remaining. 
			int caseCnt = faultyCircuits.size();
			// responses to each test vector
			vector<tuple<int,set<Circuit*>,vector<tuple<string,int>>>> responses;
			// Get seed for pseudo-random number generator.
			srand(time(0));
			for (int i = 0; i<caseCnt; i++) {
				// Create test vector 
				string testVectorName = "testVector.txt";
				ofstream Vector(testVectorName);
				for (set<string>::iterator i = inputNames.begin(); i != inputNames.end(); i++) {
					// Generate random bit (0/1)
					int randombit = rand() % 2;
					// Create test value for input
					Vector << 0 << " " << (*i) << " " << randombit << endl;
				}
				Vector.close();

				// Calculate coverage of this test vector
				tuple<int,set<Circuit*>,vector<tuple<string,int>>> vector_coverage = Calculate(testVectorName);
				// Add the coverage of this test to the list of all responses
				responses.push_back(vector_coverage);

				// Convert test vector file name to array of chars so we can remove it. 
				const char* testVectorNameChars = testVectorName.c_str();
				remove (testVectorNameChars);
			}

			// returns a pointer to the largest coverage tuple in the vector of responses. 
			auto largest_coverage = max_element(responses.begin(), responses.end(),
	                                            [](const tuple<int, set<Circuit*>, vector<tuple<string,int>>> &x,
	                                            const tuple<int, set<Circuit*>, vector<tuple<string,int>>> &y) {
	                                            return get<0>(x) < get<0>(y);
	                                           });

			set<Circuit*> faults_covered = get<1>(*largest_coverage);
			total_coverage += ((double)get<0>(*largest_coverage)/(double)total_faults);

			// Only record test vector if it detected faults 
			if (get<0>(*largest_coverage) != 0) {
				cout << "Total Coverage: " << total_coverage*100 << "%" << endl;
				for (set<Circuit*>::iterator j = faults_covered.begin(); j != faults_covered.end(); j++) {
					faultyCircuits.erase(*j);
				}

				// Make header
				vector_cnt += 1;
				TestVectorOutput << "---------------" << " Test Vector #" << vector_cnt << " ---------------" << endl;

				// Write test vector under header
				vector<tuple<string,int>> testvector = get<2>(*largest_coverage);
				for (vector<tuple<string,int>>::iterator k = testvector.begin(); k != testvector.end(); k++) {
					TestVectorOutput << get<0>(*k) << " " << get<1>(*k) << endl;
				}

				TestVectorOutput << "Total Coverage = " << total_coverage << endl;
			}

		}

		TestVectorOutput.close();
	}

// -----------------------------------------------------------------------------------------------------------------------
	/*
	This function takes as an input a filename to a test vector. The Calculate function
	then runs functional simulations on the normal circuit as well as faulty circuits with 
	stuck-at-faults. The function returns with the integer coverage of the test vector
	and a set of pointers to all the detected stuck-at-fault circuits. Also returned is 
	the test vector itself -> names and values of the inputs. 
	*/
	tuple<int,set<Circuit*>,vector<tuple<string,int>>> Calculate(string x) {
		// Make set of pointers to faulty circuits. At the end we will return with the 
		// set of faulty circuits that were detected. 
		set<Circuit*> bustedCircuits;

		// Run functional simulation on the test vector.
		GoodCircuit->FunctionalSimulation(x);
		// Grab the test vector as well to return it 
		vector<tuple<string,int>> test_inputs = GoodCircuit->CircuitInputs();

		// Grab functional simulation correct outputs; 
		vector<tuple<string,int>> correctoutputs = GoodCircuit->CircuitOutputs();

		// Next, simulate faulty circuit(s) on the same test vector
		int detectedFaults = 0;
		for (set<Circuit*>::iterator i = faultyCircuits.begin(); i != faultyCircuits.end(); i++) {
			(*i)->FunctionalSimulation(x);

			vector<tuple<string,int>> faultyoutputs = (*i)->CircuitOutputs();
			vector<tuple<string,int>> reorder_faultyoutputs; 
			for (vector<tuple<string,int>>::iterator j = correctoutputs.begin(); j != correctoutputs.end(); j++) {
				for (vector<tuple<string,int>>::iterator k = faultyoutputs.begin(); k != faultyoutputs.end(); k++) {
					if (get<0>(*j) == get<0>(*k)) {
						reorder_faultyoutputs.push_back(*k);
					}
				}
			}

			if (reorder_faultyoutputs != correctoutputs) { 
				detectedFaults += 1; 
				bustedCircuits.insert((*i));
			}
		}

		// We return with the following values below:
		// 1.) the # of detected faults by this test vector
		// 2.) pointers to the faulty circuits that were detected
		// 3.) input test vector names & values
		return {detectedFaults, bustedCircuits, test_inputs};
	}

	~FaultVectorGenerator(void) { 
		// delete faulty Circuits 
		for (set<Circuit*>::iterator i = faultyCircuits.begin(); i != faultyCircuits.end(); i++) {
			delete (*i);
		}
		// delete good Circuit
		delete GoodCircuit;
	}
};


// MAIN
int main() {
	// Retrieve circuit netlist file
	string netlistFile;
	cout << "Enter netlist file: " << endl;
	cin >> netlistFile;

	/* 
	------------------------------------------------------------------------------
	Run Timing Simulation:
	will repeatedly prompt for user input until user responds with y or n. If user
	responds y then it will run a timing simulation on the user-input netlist and
	input files. The output will be written to TimingSimOutput.txt
	*/

	//  Ask user if they want to run a timing simulation. 
	string timing_response = "x";
	while (timing_response != "y" and timing_response != "n") {
		cout << "Run Timing Simulation? [y/n]: " << endl;
		cin >> timing_response;
	};
	// if user responds 'yes' then execute timing sim.
	if (timing_response == "y") {
		// Retrieve input file from user
		string inputFile;
		cout << "Enter input file: " << endl;
		cin >> inputFile;

		// Create Timing Sim Circuit
		Circuit *CircuitTestSim = new Circuit(netlistFile);
		// Run Timing Sim
		CircuitTestSim->TimingSimulation(inputFile);
		// Delete Timing Sim Circuit
		delete CircuitTestSim;
	}
	// if user responds 'no' then skip.
	else {
		cout << "Skipping Timing Simulation" << endl;

		/* 
		------------------------------------------------------------------------------
		Run Functional Simulation:
		will repeatedly prompt for user input until user responds with y or n. If user
		responds y then it will run a functional simulation on the user-input netlist and
		input files. The output will be written to FunctionalSimOutput.txt
		*/ 

		// Ask user if they want to run a functional simulation.
		string func_response = "x";
		while (func_response != "y" and func_response != "n") {
			cout << "Run Functional Simulation? [y/n]: " << endl;
			cin >> func_response;
		};
		// if user responds 'yes' then execute functional sim. 
		if (func_response == "y") {
			// Retrieve input file
			string inputFile;
			cout << "Enter input file: " << endl;
			cin >> inputFile;

			// Create Functional Sim Circuit
			Circuit  *CircuitTestFunc = new Circuit(netlistFile);
			// Run Functional Sim
			CircuitTestFunc->FunctionalSimulation(inputFile);
			// Delete Functional Sim Circuit
			delete CircuitTestFunc;
		}
		// if user responds 'no' then skip.
		else {
			cout << "Skipping Functional Simulation" << endl;

			/* 
			------------------------------------------------------------------------------
			Run Fault Vector Generation:
			will repeatedly prompt for user input until user responds with y or n. If user
			responds y then it will generate fault vectors on the user-input netlist and 
			input files. The output will be written to FaultVectors.txt
			*/
			string fault_response = "x";
			while (fault_response != "y" and fault_response != "n") {
				cout << "Run Fault Vector Generation? [y/n]: " << endl;
				cin >> fault_response;
			};
			// if user responds 'yes' then execute fault vector gen.
			if (fault_response == "y") {
				// Create FaultVectorGenerator
				FaultVectorGenerator *Generator = new FaultVectorGenerator(netlistFile);
				// Ask for coverage constraint
				double coverage_constraint = -1;
				while (coverage_constraint < 0 or coverage_constraint > 100) {
					cout << "Minimum required coverage? (Value between 0-100): " << endl;
					cin >> coverage_constraint;
				}
				// Run Fault Vector Generation
				Generator->Generate(coverage_constraint);
				// Delete Generator
				delete Generator;
			}
			// if user responds 'no' then skip.
			else {
				cout << "Skipping Fault Vector Generation" << endl;
			};
		};
	};

	return 0;
}