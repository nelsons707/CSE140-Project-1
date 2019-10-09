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
	
	//from there check what the opcode is, depending on what the opcode is, change InstrType
	if (d.op == 0) {
		d.type = R;
		unsigned int rs = instr; // rs is the 5 bits of rs 
		rs = rs >> 21; // you still have the 11 bits that includes the opcode
		//get rid of the opcode
		rs = rs << 6;
		rs = rs >> 6;
		
		d.r.rs = rs;
		
		unsigned int rt = instr; // rt is the 5 bits of rt
		rt = rt >> 16;
		//get rid of the opcode and rs
		rt = rt << 11;
		rt = rt >> 11;
		
		d.r.rt = rt;
		
		unsigned int rd = instr; // rd is the 5 bits of rd
		rd = rd >> 11;
		//get rid of the opcode, rs and rt
		rd = rd << 16;
		rd = rd >> 16;
		
		d.r.rd = rd;
		
		unsigned int shamt = instr; // shamt is the 5 bits of shamt
		shamt = shamt >> 5;
		//get rid of the opcode, rs, rt, rd
		shamt = shamt << 21;
		shamt = shamt >> 21;
		
		d.r.shamt = shamt;
		
		unsigned int funct = instr; // funct is the 5 bits of funct
		//get rid of the opcode, rs, rt, rd, shamt
		funct = funct << 26;
		funct = funct >> 26;
		
		d.r.funct = funct;
		
		rVals.R_rs = mips.registers[d.r.rs];
		rVals.R_rt = mips.registers[d.r.rt];
		//used in ALU: rVals.R_rd = mips.registers[d.r.rd];
		
	} else if ( d.op == 2 || d.op == 3) {
		d.type = J;
		unsigned int target = instr; //target is the 26 bits of "target" or immediate
		target = target << 6;
		target = target >> 6;
		
		d.j.target = target;
		
	} else {
		d.type = I;	
		unsigned int rs = instr; // rs is the 5 bits of rs
		//get rid of opcode
		rs = rs >> 21;
		rs = rs << 6;
		rs = rs >> 6;
		
		d.i.rs = rs;
		
		unsigned int rt = instr; // rt is the 5 bits of rt
		rt = rt >> 16;
		//get rid of the opcode and rs
		rt = rt << 11;
		rt = rt >> 11;
		 
		d.i.rt = rt;
		
		unsigned int addrOrImmed = instr; // rd is the 16 bits of "addr_or_immed"
		//get rid of the opcode, rs, rt
		addrOrImmed = addOrImmed << 16;
		addrOrImmed = addrOrImmed >> 16;
		
		d.i.addr_or_immed = addrOrImmed;
		
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
		//Addu
		if(d.r.funct == 33) {		//addu instruction
			rVals.R_rd = rVals.R_rs + rVals.R_rt;
			val = rVals.R_rd;
			
		//AND	
		} else (d.r.funct == 36){
			rVals.R_rd = rVals.R_rs & rVals.R_rt;
		//JR
		} else if (d.r.funct == 8)  { 
			int PC = rVals.R_rs;
		
		//OR
		} else if (d.r.funct == 37) { 
			rVals.R_rd = rVals.R_rs | rVals.R_rt;
		
		//SLT 
		} else if (d.r.funct == 42){
			if (rVals.R_rs < rVals.R_rt)
				rVals.R_rd = 1;
			else 
				rVals.R_rd = 0;
		
		//SLL
		} else if (d.r.funct == 0) {
			rVals.R_rd = rVals.R_rt << d.r.shamt;
		
		//SRL
		} else if (d.r.funct == 2) {
			rVals.R_rd = rVals.R_rt >> d.r.shamt;
		
		//SUBU
		} else (d.r.funct == 35) {
			rVals.R_rd = rVals.R_rs - rVals.R_rs;
			
		}
			
		
	}
	
	if (d.type == I) {
		//Add Immediate Unsigned
		if (d.op == 8 || d.op == 9) {
			rVals.R_rt = rVals.R_rs + d.i.addr_or_immed;
		}
		
		//And Immediate
		if (d.op == 12) {
			rVals.R_rt = rVals.R_rs & d.i.addr_or_immed;
		}
		
		//Branch On Equal
		if (d.op == 4) {
			if (rVals.R_rs == rVals.R_rt)
				mips.pc+=4;
		}
		
		//Branch On Not Equal
		if (d.op == 5) {
			if (rVals.R_rs != rVals.R_rt)
				mips.pc+=4;
		}
		
		
		//Load Upper Immediate
//ASK DANIEL
		if (d.op == 15) {
			rVals.R_rt = d.i.addr_or_immed;	 //Supposed to be the upper 16 bits, is this it?
		}
		
		//Load word
		if (d.op == 35) {
			
		}
		
		//ORI
		if (d.op == 13) {
			rVals.R_rt = rVals.R_rs | d.i.addr_or_immed;
		}
		
		//SW
		if (d.op == 43) {
			
		}
		
		
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
	
	//WHAT IS 'INT VAL' ??
	
	if (d.op == 4){
		mips.pc = d.i.addr_or_immed;
		
	}else if (d.op == 5){
		mips.pc = d.i.addr_or_immed;
		
	}else if (d.op == 3){
		mips.register[31] = mips.pc;
		mips.pc = d.j.target;
		
	}else if (d.op == 2){
		mips.pc = d.j.target;
		
	}else if (d.op == 0 &&  d.r.funct == 8){
		mips.pc = mips.registers[d.r.rs];
	}
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
	
	int address = d.i.addr_or_immed + mips.registers[d.i.rs];
	
	int index = ; // we want the index to be the (address - the beginning of the address) / the size 
	
	if (d.op == 43){ // 43 = opcode of store word
		mips.memory[index] = mips.registers[d.i.rt];
		*changedMem = address;
		return -1;
	} else if (d.op == 35) { // 35 = opcode for load word
		return mips.memory[index];
	}	
	
	
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
	
	if (d.op == 0) 
		*changedReg = d.r.rd;
	else if (d.i.rt == 2 || d.i.rt == 3)
		*changedReg = d.i.rt;
	else
		return;
	mips.registers[*changedReg] = val;
}