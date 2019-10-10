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
 *  all the nonzero memory or just the memory location that changed->
 */
void PrintInfo ( int changedReg, int changedMem) {
    int k, addr;
    printf ("New pc = %8.8x\n", mips.pc);
    if (!mips.printingRegisters && changedReg == -1) {
        printf ("No register was updated->\n");
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
        printf ("No memory location was updated->\n");
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
	//This is an array! We get the actually memory within the address by doing this calculation. 
}

/* Decode instr, returning decoded instruction. */
void Decode ( unsigned int instr, DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	//my code is below
	
	d->op = instr >> 26; //grab first 6 bits of binary and put it into d->op
	
	//from there check what the opcode is, depending on what the opcode is, change InstrType
	if (d->op == 0) {
		d->type = R;
		unsigned int rs = instr; // rs is the 5 bits of rs 
		rs = rs >> 21; // you still have the 11 bits that includes the opcode
		//get rid of the opcode
		rs = rs << 6;
		rs = rs >> 6;
		
		d->regs.r.rs = rs;
		
		unsigned int rt = instr; // rt is the 5 bits of rt
		rt = rt >> 16;
		//get rid of the opcode and rs
		rt = rt << 11;
		rt = rt >> 11;
		
		d->regs.r.rt = rt;
		
		unsigned int rd = instr; // rd is the 5 bits of rd
		rd = rd >> 11;
		//get rid of the opcode, rs and rt
		rd = rd << 16;
		rd = rd >> 16;
		
		d->regs.r.rd = rd;
		
		unsigned int shamt = instr; // shamt is the 5 bits of shamt
		shamt = shamt >> 5;
		//get rid of the opcode, rs, rt, rd
		shamt = shamt << 21;
		shamt = shamt >> 21;
		
		d->regs.r.shamt = shamt;
		
		unsigned int funct = instr; // funct is the 5 bits of funct
		//get rid of the opcode, rs, rt, rd, shamt
		funct = funct << 26;
		funct = funct >> 26;
		
		d->regs.r.funct = funct;
		
		rVals->R_rs = mips.registers[d->regs.r.rs];
		rVals->R_rt = mips.registers[d->regs.r.rt];
		//used in ALU: rVals->R_rd = mips.registers[d->regs.r.rd];
		
	} else if ( d->op == 2 || d->op == 3) {
		d->type = J;
		unsigned int target = instr; //target is the 26 bits of "target" or immediate
		target = target << 6;
		target = target >> 6;
		
		d->regs.j.target = target;
		
	} else {
		d->type = I;	
		unsigned int rs = instr; // rs is the 5 bits of rs
		//get rid of opcode
		rs = rs >> 21;
		rs = rs << 6;
		rs = rs >> 6;
		
		d->regs.i.rs = rs;
		
		unsigned int rt = instr; // rt is the 5 bits of rt
		rt = rt >> 16;
		//get rid of the opcode and rs
		rt = rt << 11;
		rt = rt >> 11;
		 
		d->regs.i.rt = rt;
		
		unsigned int addrOrImmed = instr; // rd is the 16 bits of "addr_or_immed"
		//get rid of the opcode, rs, rt
		addrOrImmed = addrOrImmed << 16;
		addrOrImmed = addrOrImmed >> 16;
		
		d->regs.i.addr_or_immed = addrOrImmed;
		
		rVals->R_rs = mips.registers[d->regs.i.rs];
		rVals->R_rt = mips.registers[d->regs.i.rt];
	}
}

/*
 *  Print the disassembled version of the given instruction
 *  followed by a newline.
 */
void PrintInstruction ( DecodedInstr* d) {
    /* Your code goes here */
	if (d->type ==  R) {
		if (d->regs.r.funct == 32)
			printf("add     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 33)
			printf("addu    $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 36)
			printf("and     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 8)
			printf("jr      $%d\n", d->regs.r.rs);
		if (d->regs.r.funct == 39)
			printf("nor     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 37)
			printf("or      $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 42)
			printf("slt     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 42)
			printf("slt     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 43)
			printf("sltu    $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 0)
			printf("sll     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.shamt);
		if (d->regs.r.funct == 2)
			printf("srl     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.shamt);
		if (d->regs.r.funct == 34)
			printf("sub     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 35)
			printf("subu    $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
	} else if (d->type ==  J) {
		if (d->op == 2) 
			printf("j       $%d, 0x%x\n", );//we need to know the address of the target
		if (d->op == 3)
			printf("jal     $%d, 0x%x\n", );
	} else if (d->type ==  I) {
		if (d->op == 8)
			printf("addi    $%d, $%d, %d\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 9)
			printf("addiu   $%d, $%d, %d\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 12)
			printf("andi    $%d, $%d, %d\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 4)
			printf("beq     0x%x\n", d->regs.i.addr_or_immed);
		if (d->op == 5)
			printf("bne     0x%n\n", d->regs.i.addr_or_immed );
		if (d->op == 36)
			printf("lbu     $%d, $%d, %d\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 37)
			printf("lhu     $%d, $%d, %d\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 48)
			printf("11      $%d, $%d, %d\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 15)
			printf("lui     $%d, 0x%x\n", d->regs.r.rt, d->regs.i.addr_or_immed);
		if (d->op == 35)
			printf("lw      $%d, %d($%d)", d->regs.r.rt, d->regs.i.addr_or_immed, d->regs.r.rs);
		if (d->op == 13)
			printf("ori     $%d, $%d, 0x%x\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 10)
			printf("slti    $%d, $%d, 0x%x\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 11)
			printf("sltiu   $%d, $%d, 0x%x\n", d->regs.r.rt, d->regs.r.rs, d->regs.i.addr_or_immed);
		if (d->op == 43)
			printf("lw      $%d, %d($%d)", d->regs.r.rt, d->regs.i.addr_or_immed, d->regs.r.rs);
	}
}

/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	int val = 0;
	if (d->type == R) {
		//Addu
		if(d->regs.r.funct == 33) {		//addu instruction
			rVals->R_rd = rVals->R_rs + rVals->R_rt;
			val = rVals->R_rd;
			
		//AND	
		} else if(d->regs.r.funct == 36) {
			rVals->R_rd = rVals->R_rs & rVals->R_rt;
			val = rVals->R_rd;
			
		//JR 
	//Double check this
		} else if (d->regs.r.funct == 8) { 
			val = rVals->R_rs;
		
		//OR
		} else if (d->regs.r.funct == 37) { 
			rVals->R_rd = rVals->R_rs | rVals->R_rt;
			val = rVals->R_rd;
		
		//SLT 
		} else if (d->regs.r.funct == 42) {
			if (rVals->R_rs < rVals->R_rt){
				rVals->R_rd = 1;
				val = rVals->R_rd;
				
			}else {
				rVals->R_rd = 0;
				val = rVals->R_rd;
			}
		
		//SLL
		} else if (d->regs.r.funct == 0) {
			rVals->R_rd = rVals->R_rt << d->regs.r.shamt;
			val = rVals->R_rd;
		
		//SRL
		} else if (d->regs.r.funct == 2) {
			rVals->R_rd = rVals->R_rt >> d->regs.r.shamt;
			val = rVals->R_rd;
		
		//SUBU
		} else if (d->regs.r.funct == 35) {
			rVals->R_rd = rVals->R_rs - rVals->R_rs;
			val = rVals->R_rd;
		}
			
		
	}
	
	if (d->type == I) {
		//Add Immediate Unsigned
		if (d->op == 9) {
			rVals->R_rt = rVals->R_rs + d->regs.i.addr_or_immed;
			val = rVals->R_rt;
		}
		
		//And Immediate
		else if (d->op == 12) {
			rVals->R_rt = rVals->R_rs & d->regs.i.addr_or_immed;
			val = rVals->R_rt;
		}
		
		//Branch On Equal
		else if (d->op == 4) {
			if (rVals->R_rs == rVals->R_rt)
				val = mips.pc + d->regs.i.addr_or_immed;
		}
		
		//Branch On Not Equal
		else if (d->op == 5) {
			if (rVals->R_rs != rVals->R_rt)
				val = mips.pc + d->regs.i.addr_or_immed;
		}
		
		
		//Load Upper Immediate
//ASK DANIEL
		if (d->op == 15) {
			rVals->R_rt = d->regs.i.addr_or_immed;	 //Supposed to be the upper 16 bits, is this it?
			val = rVals->R_rt;
		}
		
		//Load word
		if (d->op == 35) {
			val = mips.registers[d->regs.i.rs] + d->regs.i.addr_or_immed;
		}
		
		//ORI
		else if (d->op == 13) {
			rVals->R_rt = rVals->R_rs | d->regs.i.addr_or_immed;
			val = rVals->R_rt;
		}
		
		//SW
		else (d->op == 43) {
			//This needs to go into Execute
			val = d->regs.i.addr_or_immed + mips.registers[d->regs.i.rs];
		}
		
		
	}
	
	if (d->type == J) {
		//JUMP
		if (d->op == 2) {
			
		}
		
		//JUMP REGISTER
		if (d->op == 3) {
			val = rVals->R_rs;
		}
		
		
	}
	
  return val;
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
	
	//BRANCH ON EQUAL
	if (d->op == 4){
		mips.pc = val;
	
	//BRANCH ON NOT EQUAL
	}else if (d->op == 5){
		mips.pc = val;
	
	//JUMP AND LINK
	}else if (d->op == 3){
		mips.register[31] = mips.pc;
		mips.pc = d->regs.j.target;
	
	//JUMP
	}else if (d->op == 2){
		mips.pc = val;
	
	//JUMP REGISTER
	}else if (d->op == 0 &&  d->regs.r.funct == 8){
		mips.pc = mips.registers[d->regs.r.rs];
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
	
	//int index = ; // we want the index to be the (address - the beginning of the address) / the size (4) 
	
	if (d->op == 43){ // 43 = opcode of store word
		mips.memory[index] = mips.registers[d->regs.i.rt];
		*changedMem = address;
		return -1;
	} else (d->op == 35) { // 35 = opcode for load word
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
	
	//R type instructions except for JR 
	if (d->op == 0 && d->regs.r.funct != 8) 
		*changedReg = d->regs.r.rd;
	
	else if (d->op == 9 || d->op == 12 || d->op == 16 || d->op == 35 || d->op == 13 || d->op == 43) 
		*changedReg = d->regs.r.rt;
	
	//Do the ones for SW and LW
	else
		return;
	mips.registers[*changedReg] = val;
}