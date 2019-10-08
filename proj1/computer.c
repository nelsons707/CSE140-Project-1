#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include "computer.h"
#undef mips			/* gcc already has a def for mips */

unsigned int endianSwap(unsigned int);

void PrintInfo (int changedReg, int changedMem);
unsigned int Fetch (int);
void Decode (unsigned int, DecodedInstr*, RegVals*);
int Execute (DecodedInstr*, RegVals*);
int Mem(DecodedInstr*, int, int *);
void RegWrite(DecodedInstr*, int, int *);
void UpdatePC(DecodedInstr*, int);
void PrintInstruction (DecodedInstr*);

/*Globally accessible Computer variable*/
Computer mips;
RegVals rVals;

/*
 *  Return an initialized computer with the stack pointer set to the
 *  address of the end of data memory, the remaining registers initialized
 *  to zero, and the instructions read from the given file.
 *  The other arguments govern how the program interacts with the user.
 */
void InitComputer (FILE* filein, int printingRegisters, int printingMemory,
  int debugging, int interactive) {
    int k;
    unsigned int instr;

    /* Initialize registers and memory */

    for (k=0; k<32; k++) {
        mips.registers[k] = 0;
    }
    
    /* stack pointer - Initialize to highest address of data segment */
    mips.registers[29] = 0x00400000 + (MAXNUMINSTRS+MAXNUMDATA)*4;

    for (k=0; k<MAXNUMINSTRS+MAXNUMDATA; k++) {
        mips.memory[k] = 0;
    }

    k = 0;
    while (fread(&instr, 4, 1, filein)) {
	/*swap to big endian, convert to host byte order. Ignore this.*/
        mips.memory[k] = ntohl(endianSwap(instr));
        k++;
        if (k>MAXNUMINSTRS) {
            fprintf (stderr, "Program too big.\n");
            exit (1);
        }
    }

    mips.printingRegisters = printingRegisters;
    mips.printingMemory = printingMemory;
    mips.interactive = interactive;
    mips.debugging = debugging;
}

unsigned int endianSwap(unsigned int i) {
    return (i>>24)|(i>>8&0x0000ff00)|(i<<8&0x00ff0000)|(i<<24);
}

/*
 *  Run the simulation.
 */
void Simulate () {
    char s[40];  /* used for handling interactive input */
    unsigned int instr;
    int changedReg=-1, changedMem=-1, val;
    DecodedInstr d;
    
    /* Initialize the PC to the start of the code section */
    mips.pc = 0x00400000;
    while (1) {
        if (mips.interactive) {
            printf ("> ");
            fgets (s,sizeof(s),stdin);
            if (s[0] == 'q') {
                return;
            }
        }

        /* Fetch instr at mips.pc, returning it in instr */
        instr = Fetch (mips.pc);

        printf ("Executing instruction at %8.8x: %8.8x\n", mips.pc, instr);

        /* 
	 * Decode instr, putting decoded instr in d
	 * Note that we reuse the d struct for each instruction.
	 */
        Decode (instr, &d, &rVals);

        /*Print decoded instruction*/
        PrintInstruction(&d);

        /* 
	 * Perform computation needed to execute d, returning computed value 
	 * in val 
	 */
        val = Execute(&d, &rVals);

	UpdatePC(&d,val);

        /* 
	 * Perform memory load or store. Place the
	 * address of any updated memory in *changedMem, 
	 * otherwise put -1 in *changedMem. 
	 * Return any memory value that is read, otherwise return -1.
         */
        val = Mem(&d, val, &changedMem);

        /* 
	 * Write back to register. If the instruction modified a register--
	 * (including jal, which modifies $ra) --
         * put the index of the modified register in *changedReg,
         * otherwise put -1 in *changedReg.
         */
        RegWrite(&d, val, &changedReg);

        PrintInfo (changedReg, changedMem);
    }
}

/*
 *  Print relevant information about the state of the computer.
 *  changedReg is the index of the register changed by the instruction
 *  being simulated, otherwise -1.
 *  changedMem is the address of the memory location changed by the
 *  simulated instruction, otherwise -1.
 *  Previously initialized flags indicate whether to print all the
 *  registers or just the one that changed, and whether to print
 *  all the nonzero memory or just the memory location that changed.
 */
void PrintInfo ( int changedReg, int changedMem) {
    int k, addr;
    printf ("New pc = %8.8x\n", mips.pc);
    if (!mips.printingRegisters && changedReg == -1) {
        printf ("No register was updated.\n");
    } else if (!mips.printingRegisters) {
        printf ("Updated r%2.2d to %8.8x\n",
        changedReg, mips.registers[changedReg]);
    } else {
        for (k=0; k<32; k++) {
            printf ("r%2.2d: %8.8x  ", k, mips.registers[k]);
            if ((k+1)%4 == 0) {
                printf ("\n");
            }
        }
    }
    if (!mips.printingMemory && changedMem == -1) {
        printf ("No memory location was updated.\n");
    } else if (!mips.printingMemory) {
        printf ("Updated memory at address %8.8x to %8.8x\n",
        changedMem, Fetch (changedMem));
    } else {
        printf ("Nonzero memory\n");
        printf ("ADDR	  CONTENTS\n");
        for (addr = 0x00400000+4*MAXNUMINSTRS;
             addr < 0x00400000+4*(MAXNUMINSTRS+MAXNUMDATA);
             addr = addr+4) {
            if (Fetch (addr) != 0) {
                printf ("%8.8x  %8.8x\n", addr, Fetch (addr));
            }
        }
    }
}

/*
 *  Return the contents of memory at the given address. Simulates
 *  instruction fetch. 
 */
unsigned int Fetch ( int addr) {
    return mips.memory[(addr-0x00400000)/4];
}

/* Decode instr, returning decoded instruction. */
void Decode ( unsigned int instr, DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	//my code is below
	
	int i = 0;
	
	char binary[32];
	char temp_str[4];
	
	while (instr[i]) {
		switch (instr[i]){
		case '0':
			temp_str = "0000";
			strcat(binary,temp_str);
			break;
		case '1':
			temp_str = "0001";
			strcat(binary,temp_str);
			break;
		case '2':
			temp_str = "0010";
			strcat(binary,temp_str);
			break;	
		case '3':
			temp_str = "0011";
			strcat(binary,temp_str);
			break;
		case '4':
			temp_str = "0100";
			strcat(binary,temp_str);
			break;
		case '5':
			temp_str = "0101";
			strcat(binary,temp_str);
			break;
		case '6':
			temp_str = "0110";
			strcat(binary,temp_str);
			break;
		case '7':
			temp_str = "0111";
			strcat(binary,temp_str);
			break;
		case '8':
			temp_str = "1000";
			strcat(binary,temp_str);
			break;
		case '9':
			temp_str = "1001";
			strcat(binary,temp_str);
			break;
		case 'a':
			temp_str = "1010";
			strcat(binary,temp_str);
			break;
		case 'b':
			temp_str = "1011";
			strcat(binary,temp_str);
			break;
		case 'c':
			temp_str = "1100";
			strcat(binary,temp_str);
			break;
		case 'd':
			temp_str = "1101";
			strcat(binary,temp_str);
			break;
		case 'e': 
			temp_str = "1110";
			strcat(binary,temp_str);
			break;
		case 'f':
			temp_str = "1111";
			strcat(binary,temp_str);
			break;
		}
		i++;
	}
	
	//CHECK WITH DANIEL IF WE CAN DO THIS
	//grab first 6 bits of binary and put it into d, this isn't the right way to do this, maybe append an int
	d.op = binary[0];
	
	for (int j = 1; j < 6; j++){
		if (binary[j] == '0')
			d.op = d.op << 1;
		else (binary[j] == '1') {
			d.op = d.op << 1;
			d.op = d.op | 1;
		}	
	}
	
	//from there check what the opcode is, depending on what the opcode is, change InstrType
	if (d.op == 000000) {
		d.op = binary[6];
		for (i = 7; i < 11; i++) {
			if (binary[i] == '0')
				d.r.rs = d.r.rs << 1;
			else (binary[i] == '1') {
				d.r.rs = d.r.rs << 1;
				d.r.rs = d.r.rs | 1;
			}
		}
		d.r.rs = //get the next 5 bits same way we did op
		d.RRegs.rt = //Same as above
		d.Rregs.rd
	} else if ( op == 2 || op == 3) {
		d.JRegs.target = //However many bits is target in J format instructions.
	} else {
		d.DecodedInstr = IRegs		
	}
	

	//Depending on InstrType, we can fill in the values for the structs of either RRegs, IRegs, or JRegs
	//d.Rregs.rs = 2;
	if (Rregs) {
		for (int j = 6; j < 11; j++){
			binary[j];
			
			if (binary[j] = '00000') {
				d.r.rs = 0;
			} else if (binary[j] = '00010') {
				d.r.rs = 2;
			} else if (binary[j] = '00011') {
				d.r.rs = 3;
			} else if (binary[j] = '00100') {
				d.r.rs = 4;
			} else if (binary[j] = '00101') {
				d.r.rs = 5;
			} else if (binary[j] = '00110') {
				d.r.rs = 6;
			} else if (binary[j] = '00111') {
				d.r.rs = 7;
			} else if (binary[j] = '01000') {
				d.r.rs = 8;
			} else if (binary[j] = '01001') {
				d.r.rs = 9;
			} else if (binary[j] = '01010') {
				d.r.rs = 10;
			} else if (binary[j] = '01011') {
				d.r.rs = 11;
			} else if (binary[j] = '01100') {
				d.r.rs = 12;
			} else if (binary[j] = '01101') {
				d.r.rs = 13;
			} else if (binary[j] = '01111') {
				d.r.rs = 14;
			} else if (binary[j] = '10000') {
				d.r.rs = 15;
			} else if (binary[j] = '10001') {
				d.r.rs = 16;
			} else if (binary[j] = '10010') {
				d.r.rs = 17;
			} else if (binary[j] = '10011') {
				d.r.rs = 18;
			} else if (binary[j] = '10100') {
				d.r.rs = 19;
			}
		}
	}
	
}
	//implement filling in Reg vals. 
	rVals->R_rs = mips.registers[rs];
    rVals->R_rt = mips.registers[rt];
    rVals->R_rd = mips.registers[rd];

/*
 *  Print the disassembled version of the given instruction
 *  followed by a newline.
 */
void PrintInstruction ( DecodedInstr* d) {
    /* Your code goes here */
	
}

/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	if (RRegs) {
		if 
	}
  return 0;
}

/* 
 * Update the program counter based on the current instruction. For
 * instructions other than branches and jumps, for example, the PC
 * increments by 4 (which we have provided).
 */
void UpdatePC ( DecodedInstr* d, int val) {
    mips.pc+=4;
    /* Your code goes here */
}

/*
 * Perform memory load or store. Place the address of any updated memory 
 * in *changedMem, otherwise put -1 in *changedMem. Return any memory value 
 * that is read, otherwise return -1. 
 *
 * Remember that we're mapping MIPS addresses to indices in the mips.memory 
 * array. mips.memory[0] corresponds with address 0x00400000, mips.memory[1] 
 * with address 0x00400004, and so forth.
 *
 */
int Mem( DecodedInstr* d, int val, int *changedMem) {
    /* Your code goes here */
  return 0;
}

/* 
 * Write back to register. If the instruction modified a register--
 * (including jal, which modifies $ra) --
 * put the index of the modified register in *changedReg,
 * otherwise put -1 in *changedReg.
 */
void RegWrite( DecodedInstr* d, int val, int *changedReg) {
    /* Your code goes here */
}
