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
    return mips.memory[(addr-0x00400000)/4]; // this is an integer and comouter stores in 0s and 1s
}

/* Decode instr, returning decoded instruction. */
void Decode ( unsigned int instr, DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	//my code is below
	
	d.op = instr >> 26; //grab first 6 bits of binary and put it into d.op
	
	/*d.op = binary[0];
	for (int j = 1; j < 6; j++){
		if (binary[j] == '0')
			d.op = d.op << 1;
		else (binary[j] == '1') {
			d.op = d.op << 1;
			d.op = d.op | 1;
		}	
	}*/
	
	//from there check what the opcode is, depending on what the opcode is, change InstrType
	if (d.op == 0) {
		d.type = R;
		unsigned int instr1 = instr; // instr1 is the 5 bits of rs 
		instr1 = instr1 >> 21; // you still have the 11 bits that includes the opcode
		//get rid of the opcode
		instr1 = instr1 << 6;
		instr1 = instr1 >> 6;
		
		d.r.rs = instr1;
		
		unsigned int instr2 = instr; // instr2 is the 5 bits of rt
		instr2 = instr2 >> 16;
		//get rid of the opcode and rs
		instr2 = instr2 << 11;
		instr2 = instr2 >> 11;
		
		d.r.rt = instr2;
		
		unsigned int instr3 = instr; // instr3 is the 5 bits of rd
		instr3 = instr3 >> 11;
		//get rid of the opcode, rs and rt
		instr2 = instr2 << 16;
		instr2 = instr2 >> 16;
		
		d.r.rd = instr3;
		
		unsigned int instr4 = instr; // instr4 is the 5 bits of shamt
		instr4 = instr4 >> 5;
		//get rid of the opcode, rs, rt, rd
		instr4 = instr4 << 21;
		instr4 = instr4 >> 21;
		
		d.r.shamt = instr4;
		
		unsigned int instr5 = instr; // instr5 is the 5 bits of funct
		//get rid of the opcode, rs, rt, rd, shamt
		instr5 = instr5 << 26;
		instr5 = instr5 >> 26;
		
		d.r.funct = instr5;
		
		rVals.R_rs = mips.registers[d.r.rs];
		rVals.R_rt = mips.registers[d.r.rt];
		//used in ALU: rVals.R_rd = mips.registers[d.r.rd];
		
	} else if ( d.op == 2 || d.op == 3) {
		d.type = J;
		unsigned int instr1 = instr; //instr1 is the 26 bits of "target" or immediate
		instr1 = instr1 << 6;
		instr1 = instr1 >> 6;
		
		d.j.target = instr1;
		
	} else {
		d.type = I;	
		unsigned int instr1 = instr; // instr1 is the 5 bits of rs
		//get rid of opcode
		instr1 = instr1 >> 21;
		instr1 = instr1 << 6;
		instr1 = instr1 >> 6;
		
		d.i.rs = instr1;
		
		unsigned int instr2 = instr; // instr2 is the 5 bits of rt
		instr2 = instr2 >> 16;
		//get rid of the opcode and rs
		instr2 = instr2 << 11;
		instr2 = instr2 >> 11;
		
		d.i.rt = instr2;
		
		unsigned int instr3 = instr; // instr3 is the 16 bits of "addr_or_immed"
		//get rid of the opcode, rs, rt
		instr3 = instr3 << 16;
		instr3 = instr3 >> 16;
		
		d.i.addr_or_immed = instr3;
		
		rVals.R_rs = mips.registers[d.i.rs];
		rVals.R_rt = mips.registers[d.i.rt];
	}
}

/*
 *  Print the disassembled version of the given instruction
 *  followed by a newline.
 */
void PrintInstruction ( DecodedInstr* d) {
    /* Your code goes here */
	if (d.type ==  R) {
		if (d.r.funct == 32)
			print("add     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 33)
			print("addu    $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 36)
			print("and     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 8)
			print("jr      $%d\n", d.r.rs);
		if (d.r.funct == 39)
			print("nor     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 37)
			print("or      $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 42)
			print("slt     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 42)
			print("slt     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 43)
			print("sltu    $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 0)
			print("sll     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.shamt);
		if (d.r.funct == 2)
			print("srl     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.shamt);
		if (d.r.funct == 34)
			print("sub     $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
		if (d.r.funct == 35)
			print("subu    $%d, $%d, $%d\n", d.r.rd, d.r.rs, d.r.rt);
	} else if (d.type ==  J) {
		if (d.op == 2) 
			print("j       $%d, 0x%x\n", //we need to know the address of the target);
		if (d.op == 3)
			print("jal     $%d, 0x%x\n", );
	} else if (d.type ==  I) {
		if (d.op == 8)
			print("addi    $%d, $%d, %d\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 9)
			print("addiu   $%d, $%d, %d\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 12)
			print("andi    $%d, $%d, %d\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 4)
			print("beq     0x%x\n", d.i.addr_or_immed);
		if (d.op == 5)
			print("bne     0x%n\n", d.i.addr_or_immed );
		if (d.op == 36)
			print("lbu     $%d, $%d, %d\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 37)
			print("lhu     $%d, $%d, %d\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 48)
			print("11      $%d, $%d, %d\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 15)
			print("lui     $%d, 0x%x\n", d.r.rt, d.i.addr_or_immed);
		if (d.op == 35)
			print("lw      $%d, %d($%d)", d.r.rt, d.i.addr_or_immed, d.r.rs);
		if (d.op == 13)
			print("ori     $%d, $%d, 0x%x\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 10)
			print("slti    $%d, $%d, 0x%x\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 11)
			print("sltiu   $%d, $%d, 0x%x\n", d.r.rt, d.r.rs, d.i.addr_or_immed);
		if (d.op == 43)
			print("lw      $%d, %d($%d)", d.r.rt, d.i.addr_or_immed, d.r.rs);
	}
}

/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	if (d.type == R) {
		if(d.r.funct == 32 || 33) {		//add instruction or addu instruction
			rVals.R_rd = rVals.R_rs + rVals.R_rt;
			
		} else if (d.r.funct == 36) { 		//and instruction
			rVals.R_rd = rVals.R_rs & rVals.R_rt;
			
		} else if (d.r.funct == 8)  { // jr instruction
			int PC = rVals.R_rs;
			
		} else if (d.r.funct == 39) { //nor instr
			rVals.R_rd = ~(rVals.R_rs | rVals.R_rt);
			
		} else if (d.r.funct == 37) { //or instr
			rVals.R_rd = rVals.R_rs | rVals.R_rt;
			
		} else if (d.r.funct == 42){
			if (rVals.R_rs < rVals.R_rt)
				rVals.R_rd = 1;
			else 
				rVals.R_rd = 0;
			
		} else if (d.r.funct == 0) {
			rVals.R_rd = rVals.R_rt << d.r.shamt;
			
		} else if (d.r.funct == 2) {
			rVals.R_rd = 
			
		} else if (d.r.funct == 34) {
			
		} else if (d.r.funct == 35) {
			
		}
			
		
	}
	
	if (d.type == I) {
		
	}
	
	if (d.type == J) {
		
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
