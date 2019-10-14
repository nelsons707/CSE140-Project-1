/*
CSE 140: Project 1
Partners: Malia Bowman and Nelson Swasono
*/

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

		//printf("Before the print\n"); //checking if the program executed to here

        printf ("Executing instruction at %8.8x: %8.8x\n", mips.pc, instr);

        /* 
	 * Decode instr, putting decoded instr in d
	 * Note that we reuse the d struct for each instruction.
	 */
		
        Decode(instr, &d, &rVals);

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

        PrintInfo(changedReg, changedMem);
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
	
	d->op = instr >> 26; 
	
	//from there check what the opcode is, depending on what the opcode is, change InstrType
	if (d->op == 0x0) {
		d->type = R;
		unsigned int rs = instr; // rs is the 5 bits of rs 
		rs = rs << 6; // you still have the 11 bits that includes the opcode
		//get rid of the opcode
		rs = rs >> 27;
		
		d->regs.r.rs = rs;
		
		
		unsigned int rt = instr; // rt is the 5 bits of rt
		//rt = rt >> 16;
		//get rid of the opcode and rs
		rt = rt << 11;
		rt = rt >> 27;
		
		d->regs.r.rt = rt;
		
		unsigned int rd = instr; // rd is the 5 bits of rd
		//get rid of the opcode, rs and rt
		rd = rd << 16;
		rd = rd >> 27;
		
		d->regs.r.rd = rd;
		
		unsigned int shamt = instr; // shamt is the 5 bits of shamt
		//get rid of the opcode, rs, rt, rd
		shamt = shamt << 21;
		shamt = shamt >> 27;
		
		d->regs.r.shamt = shamt;
		
		unsigned int funct = instr; // funct is the 5 bits of funct
		//get rid of the opcode, rs, rt, rd, shamt
		funct = funct << 26;
		funct = funct >> 26;
		
		d->regs.r.funct = funct;
		
		//NOTE: use this form in execute (ALU) so we know exactly is being used in the execution!
		rVals->R_rs = mips.registers[d->regs.r.rs];
		rVals->R_rt = mips.registers[d->regs.r.rt];
		rVals->R_rd = mips.registers[d->regs.r.rd]; // Went to Daniel's office hours and he helped us with this part. 
		
		
	} else if ( d->op == 0x2 || d->op == 0x3) {
		d->type = J;
		unsigned int target = instr; //target is the 26 bits of "target" or immediate
		target = target << 6;
		target = target >> 6;
		//target = target << 2;
		
		d->regs.j.target = target;
		
		
	} else if (d->op == 9 || d->op == 12 || d->op ==4 || d->op == 5 || d->op == 15 || d->op == 35 || d->op == 13 || d->op == 43){
		d->type = I;	
		unsigned int rs = instr; // rs is the 5 bits of rs
		//get rid of opcode
		rs = rs << 6;
		rs = rs >> 27;
		//rs = rs & 00000011111; // this worked too but we understand the shifting bit manipulation better than this method
		
		d->regs.i.rs = rs;
	
		
		unsigned int rt = instr; // rt is the 5 bits of rt
		//get rid of the opcode and rs
		//rt = rt >> 16;
		//rt = rt & 0000000000011111;
		rt = rt << 11;
		//rt = rt >> 11;
		rt = rt >> 27;
		 
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
		if (d->regs.r.funct == 33)
			printf("addu    $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 0x24)
			printf("and     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 8)
			printf("jr      $31\n");
		if (d->regs.r.funct == 37)
			printf("or      $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 42)
			printf("slt     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
		if (d->regs.r.funct == 0)
			printf("sll     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.shamt);
		if (d->regs.r.funct == 2)
			printf("srl     $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.shamt);
		if (d->regs.r.funct == 35)
			printf("subu    $%d, $%d, $%d\n", d->regs.r.rd, d->regs.r.rs, d->regs.r.rt);
	} else if (d->type ==  J) {
		if (d->op == 2) 
			printf("j       0x%x\n", d->regs.j.target << 2);//we need to know the address of the target, if this print does not match multiply target by 4
		if (d->op == 3)
			printf("jal     0x%x\n", d->regs.j.target << 2);
	} else if (d->type ==  I) {
		if (d->op == 9)
			printf("addiu   $%d, $%d, %d\n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);
		if (d->op == 12)
			printf("andi    $%d, $%d, %d\n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);
		if (d->op == 4)
			printf("beq     $%d, $%d, 0x%x\n", d->regs.i.rs, d->regs.i.rt, (d->regs.i.addr_or_immed)*4 + 4 + mips.pc);  // check if this is right might have to mult by 4
		if (d->op == 5)
			printf("bne     $%d, $%d, 0x%x\n", d->regs.i.rs, d->regs.i.rt, (d->regs.i.addr_or_immed) * 4 + 4 + mips.pc);
		if (d->op == 15)
			printf("lui     $%d, 0x%x\n", d->regs.i.rt, d->regs.i.addr_or_immed);
		if (d->op == 35)
			printf("lw      $%d, %d($%d)", d->regs.i.rt, d->regs.i.addr_or_immed, d->regs.r.rs);
		if (d->op == 13)
			printf("ori     $%d, $%d, 0x%x\n", d->regs.i.rt, d->regs.i.rs, d->regs.i.addr_or_immed);
		if (d->op == 43)
			printf("sw      $%d, %d($%d)", d->regs.i.rt, d->regs.i.addr_or_immed, d->regs.i.rs);
	}
	else 
		exit(0);
	
}

/* Perform computation needed to execute d, returning computed value */
int Execute ( DecodedInstr* d, RegVals* rVals) {
    /* Your code goes here */
	
	if (d->op == 0) {
		//Addu
		if(d->regs.r.funct == 33) {		//addu instruction
			return (rVals->R_rs + rVals->R_rt);
			
		//AND	
		} else if(d->regs.r.funct == 36) {
			return (rVals->R_rs & rVals->R_rt);						
		//JR 
	//Double check this
		} else if (d->regs.r.funct == 8) { 
			return (rVals->R_rs);
		
		//OR
		} else if (d->regs.r.funct == 37) { 
			return (rVals->R_rs | rVals->R_rt);
		
		//SLT 
		} else if (d->regs.r.funct == 42) {
			if(rVals->R_rs < rVals->R_rt){
                return 1;
            }else{
                return 0;
            }
		
		//SLL
		} else if (d->regs.r.funct == 0) {
			return (rVals->R_rt << d->regs.r.shamt);
		
		//SRL
		} else if (d->regs.r.funct == 2) {
			return (rVals->R_rt >> d->regs.r.shamt);
		
		//SUBU
		} else if (d->regs.r.funct == 35) {
			return (rVals->R_rs - rVals->R_rt);
		}
			
		
	}
	
	//if (d->type == I) {
		//Add Immediate Unsigned
		else if (d->op == 9) {
			return (rVals->R_rs + d->regs.i.addr_or_immed);
		}
		
		//And Immediate
		else if (d->op == 12) {
			return (rVals->R_rs & d->regs.i.addr_or_immed);
		}
		
		//Branch On Equal
		else if(d->op == 0x4){
            if(rVals->R_rs == rVals->R_rt){
                return (d->regs.i.addr_or_immed);
            }else{
                return 0;
            }
			
		//Branch On Not Equal
        }else if(d->op == 0x5){
            if (rVals->R_rs != rVals->R_rt){
                return (d->regs.i.addr_or_immed);
            }else{
                return 0;
            }
		
		//Load Upper Immediate
		}else if (d->op == 15) {
			 return (rVals->R_rs << 16);	 //Supposed to be the upper 16 bits, is this it? 
		}
		
		//Load word
		else if (d->op == 35) {
			 return (rVals->R_rs + (d->regs.i.addr_or_immed << 2));
		}
		
		//ORI
		else if (d->op == 13) {
			 return (rVals->R_rs | (d->regs.i.addr_or_immed << 2));
		}
		
		//SW
		else if(d->op == 43) {
			 return (rVals->R_rs);
		}
	//}
	
	//if (d->type == J) {
		//JUMP
		else if (d->op == 2) {
			 return (d->regs.j.target << 2);
		} 

		//JUMP and LINK
		else if (d->op == 3) {
			return mips.pc + 4;
			//return (d->regs.j.target << 2);
			//mips.registers[31] = 8 + mips.pc;
			//mips.pc = d->regs.j.target;
		    //val = mips.pc;
		}
		else {
			exit(0);
		}
	//} 
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
	
	/*Lines 487 to 507 was our original implementation which did NOT work so we had to go back to the drawing board!
	//BRANCH ON EQUAL
	if (d->op == 4){
		mips.pc = val;
	
	//BRANCH ON NOT EQUAL
	}else if (d->op == 5){
		mips.pc = val;
	
	//JUMP AND LINK
	}else if (d->op == 3){
		mips.registers[31] = mips.pc;
		mips.pc = d->regs.j.target;
	
	//JUMP
	}else if (d->op == 2){
		mips.pc = val;
	
	//JUMP REGISTER
	}else if (d->op == 0 &&  d->regs.r.funct == 8){
		mips.pc = mips.registers[d->regs.r.rs];
	}
	*/ 
	
	//Let's try to do implement this stage like the others, by looking at the type (i.e. R, J, or I type)
	//Pay attention to the MIPS sheet! that's above was not working. 
	if (d->type == R) {
		//JUMP REGISTER
        if(d->regs.r.funct == 8){
			mips.pc = val; // val is the int returned from execute, remember!
		}
    } else if(d->op == 0x5 || d->op == 0x4){
        mips.pc += val;
    } else if(d->op == 0x2 || d->op == 0x3){
        mips.pc = val + 4;
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
	
	int begaddress = 0x00400000;
	//int index = (val - begaddress) >> 2 ; // we want the index to be the (address - the beginning of the address) / the size (4) 
	
	if ( (val + begaddress) > 0x00401000 && (val + begaddress) < 0x00403fff){
		printf("Memory Access Exception at %x: address %x\n", val, val + begaddress);
	}
	if(d->op == 35){ // lw
        *changedMem = -1;
        return mips.memory[d->regs.i.addr_or_immed];
    }else if(d->op == 43){ //sw
        *changedMem = val;
        mips.memory[d->regs.i.addr_or_immed] = val;
        return -1;
    }else{
        *changedMem = -1;
        return val;
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
	
	// lines 572 to 582 did not work! Again, let's implement by looking at the type (i.e. R, J, or I type)
	/*R type instructions except for JR 
	if (d->op == 0 && d->regs.r.funct != 8) 
		*changedReg = d->regs.r.rd;
	
	else if (d->op == 9 || d->op == 12 || d->op == 16 || d->op == 35 || d->op == 13 || d->op == 43) 
		*changedReg = d->regs.r.rt;
	
	//Do the ones for SW and LW
	
	mips.registers[*changedReg] = val;
	*/
	
	//new implementation below
	
	if (d->type == R) {
		//JUMP REGISTER
        if(d->regs.r.funct == 8){
            //mips.registers[31] = mips.pc;
            //*changedReg = 31;
			 *changedReg = -1;
			 
        } else {
            mips.registers[d->regs.r.rd] = val;
            *changedReg = d->regs.r.rd;
        }
		
    } else if (d->type == J) {
		//JUMP AND LINK
		if (d->op == 3) {
            mips.registers[31] = val; // Remember, this is from the MIPS sheet!
			*changedReg = 31;  
        } else {
        *changedReg = -1;
		}
		
	} else if (d->type == I) {
		//BRANCH NOT EQUAL and SW
        if (d->op == 43 || d->op == 5) {
            *changedReg = -1;
        } else {
            mips.registers[d->regs.i.rt] = val;
			*changedReg = d->regs.i.rt;
        }
    }

}
