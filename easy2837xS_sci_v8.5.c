/***************************************************************
    easy2837xS_sci.c
	2015.01.01 : First release based TI header files v130
	2015.12.15 : 8 Byte writing with auto address increasement
				 block data (max. 512 word) transfer function
				 SciaRegs.SCIRXBUF.all -> ScibRegs.SCIRXBUF.bit.RXDT
	by Daewoong Chung (Á¤´ë¿õ)


****************************************************************/
#include "F2837xS_device.h"
#include "F28x_Project.h"
#include "easy2837xS_sci_v8.5.h"
#include "RingBuff.h"

/////////////////////////////////////////////////////////////////
// NOTICE : Don't uncomment ! This is for internal use only
/////////////////////////////////////////////////////////////////
//#define FLASH_API_WRAPPER

/////////////////////////////////////////////////////////////////
// NOTICE :
// When you run the code with flash, uncomment below and run the ISRs fast on the RAM
/////////////////////////////////////////////////////////////////
//#pragma CODE_SECTION(easy_RXINT_ISR, "ramfuncs");
//#pragma CODE_SECTION(easy_TXINT_ISR, "ramfuncs");

/////////////////////////////////////////////////////////////////
// NOTICE : Please change below CPU_CLK, LSP_CLK, BAUDRATE
//          according to your system
//          Don't change any other variables and source
/////////////////////////////////////////////////////////////////
#define CPU_CLK		200000000L			// 200MHz
//#define CPU_CLK		190000000L			// 190MHz
//#define CPU_CLK		150000000L			// 150MHz
//#define CPU_CLK		120000000L			// 120MHz
//#define CPU_CLK		100000000L			// 100MHz

#define	LSP_CLK		(CPU_CLK/4)				// default at reset

//#define	BAUDRATE	9600L
//#define	BAUDRATE	19200L	
//#define	BAUDRATE	38400L
//#define	BAUDRATE	57600L
//#define	BAUDRATE	86400L
#define	BAUDRATE	115200L
//#define	BAUDRATE	153600L
//#define	BAUDRATE	192000L
//#define	BAUDRATE	230400L
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

// Bit codes for Test Bit instruction (BIT)
#ifdef BIT0
#else
#define BIT0		(1<<0)
#define BIT1		(1<<1)
#define BIT2		(1<<2)
#define BIT3		(1<<3)
#define BIT4		(1<<4)
#define BIT5		(1<<5)
#define BIT6		(1<<6)
#define BIT7		(1<<7)
#endif

#define	TX_START	{	if(SciaRegs.SCICTL2.bit.TXRDY) {	\
								SciaRegs.SCICTL2.all = BIT1 | BIT0;	\
								SciaRegs.SCITXBUF.all = ExtractRing();}	\
						}
#define	TX_STOP	{SciaRegs.SCICTL2.all = BIT1;}	// enable only Rx int

// easyDSP commands & states
#define STAT_INIT		0
#define STAT_ADDR		1
#define STAT_DATA2B		2
#define STAT_DATA4B		3
#define STAT_WRITE		4
#define STAT_DATA8B		5
#define STAT_CONFIRM	6
#define STAT_DATA_BLOCK	7				// v8.5

#define CMD_ADDR			0xE7
#define	CMD_READ2B			0xDB
#define CMD_READ4B			0xC3
#define CMD_READ8B			0x8B
#define CMD_READ16B			0x28
#define	CMD_DATA2B			0xBD
#define	CMD_DATA4B			0x99
#define	CMD_DATA8B			0x64
#define CMD_DATA_BLOCK		0x55		// v8.5
#define	CMD_WRITE			0x7E
#define	CMD_FB_READ			0x0D
#define	CMD_FB_WRITE_OK		0x0D
#define CMD_FB_WRITE_NG		0x3C
#define CMD_CHANGE_CPU		0X5A
#define CMD_CONFIRM			0xA5

// for internal use
unsigned int ezDSP_Version_SCI = 850;
unsigned int ezDSP_uRead16BPossible = 1, ezDSP_uRead8BPossible = 1;
float ezDSP_fFloat = 0;

void easyDSP_SCI_Init()
{
	int i;

	// SCI-A to CPU1
	// This register must be configured prior to enabling the peripheral clocks.
//	EALLOW;
//	DevCfgRegs.CPUSEL5.bit.SCI_A = 0;
//	EDIS;

	// SCI-A CLOCK ENABLE
	EALLOW;
	CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
	EDIS;

	GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
	GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
//	GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PUSHPULL);
//	GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);

	EALLOW;
	GpioCtrlRegs.GPCPUD.bit.GPIO85 = 0;    // Enable pull-up for GPIO85 (SCIRXDA)
	GpioCtrlRegs.GPCPUD.bit.GPIO84 = 0;	   // Enable pull-up for GPIO84 (SCITXDA)
//	GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = 3;  // Asynch(no qualification) input GPIO85 (SCIRXDA)
    EDIS;

	// SCI Registers Settings : no action = no use
	//SciaRegs.SCIFFTX.all = 0xa000;		// FIFO reset
	//SciaRegs.SCIFFCT.all = 0x4000;		// Clear ABD
 	
	SciaRegs.SCICCR.all = 0x7;						// 1 stop & no parity & 8bit char, no loopback, idle-line
	SciaRegs.SCICTL1.all = BIT6 | BIT1 | BIT0;		// enable RX-ERR interrupt, TX and RX operation

	i = (int)(((float)LSP_CLK/(BAUDRATE*8.) - 1) + 0.5);
	SciaRegs.SCIHBAUD.all = i >> 8;
	SciaRegs.SCILBAUD.all = i & 0xFF;

	SciaRegs.SCICTL2.all = 0x3;			// enable RX/BK INT, TX INT
	SciaRegs.SCICTL1.all = BIT6 | BIT5 | BIT1 | BIT0;	// Relinquish SCI from Reset

    // Reassign ISR for easyDSP. Don't use SCIRXINTA_ISR & SCITXINTA_ISR found in F28x_DefaultIsr.c.
	EALLOW;
	PieVectTable.SCIA_RX_INT = &easy_RXINT_ISR;
	PieVectTable.SCIA_TX_INT = &easy_TXINT_ISR;
	EDIS;

	// Enable interrupts required
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;		// Enable the PIE block
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;		// Enable SCI-A RX/TX INT in the PIE: Group 9 interrupt 1,2
	PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
	IER |= M_INT9;							// Enable CPU INT9 for SCI-A  (M_INT9 = 0x100)
	EINT;									// Enable Global interrupt INTM

	// init ring buffer
	ResetRing();
}

// error counter
unsigned int ezDSP_uBRKDTCount = 0, ezDSP_uFECount = 0, ezDSP_uOECount = 0, ezDSP_uPECount = 0;
unsigned int ezDSP_uWrongISRCount = 0;

// for easyDSP
unsigned char ezDSP_ucRx = 0;
extern unsigned int ezDSP_uState = STAT_INIT, ezDSP_uData = 0, ezDSP_uChksum = 0;
unsigned long ezDSP_ulData = 0, ezDSP_ulAddr = 0;
unsigned int ezDSP_uAddrRdCnt = 0, ezDSP_uDataRdCnt = 0;
unsigned long long ezDSP_ullData = 0;
#ifdef FLASH_API_WRAPPER
unsigned int ezDSP_uBlockTransfer = 1, ezDSP_uBlockSize = 0, ezDSP_uBlockIndex = 0, ezDSP_uChkSumCalculated = 0; // v8.5
#endif

interrupt void easy_RXINT_ISR()
{
	///////////////////////////////////////////////////////////////////////////
	// interrupt nesting to allow another interrupts before the completion of this ISR
	// Please kindly use your own code for this purpose if necessary
	///////////////////////////////////////////////////////////////////////////
	//EINT;
	//IER |= M_INT9;
	//PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
	///////////////////////////////////////////////////////////////////////////

	Uint16 uIndex;

	// check RX Error
	if(SciaRegs.SCIRXST.bit.RXERROR) {
		if(SciaRegs.SCIRXST.bit.BRKDT)	ezDSP_uBRKDTCount++;	// Break Down
		if(SciaRegs.SCIRXST.bit.FE) 	ezDSP_uFECount++;		// FE
		if(SciaRegs.SCIRXST.bit.OE) 	ezDSP_uOECount++;		// OE
		if(SciaRegs.SCIRXST.bit.PE)		ezDSP_uPECount++;		// PE

		// 'Break down' stops further Rx operation.
		// software reset is necessary to clear its status bit and proceed further rx operation
		SciaRegs.SCICTL1.bit.SWRESET = 0;
		SciaRegs.SCICTL1.bit.SWRESET = 1;

		PieCtrlRegs.PIEACK.all |= 0x100;
		return;
	}

	// Receive Char
	if(SciaRegs.SCIRXST.bit.RXRDY) {
		ezDSP_ucRx = SciaRegs.SCIRXBUF.bit.SAR;

		// loop back for test
		//SciaRegs.SCITXBUF.all = ezDSP_ucRx;
		//PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;	
		//return;
	}
	else {
		ezDSP_uWrongISRCount++;
		PieCtrlRegs.PIEACK.all |= 0x100;
		return;
	}

	////////////////////////////////////////////
	// Parsing by state
	////////////////////////////////////////////
	
	if(ezDSP_uState == STAT_INIT) {
		if(ezDSP_ucRx == CMD_ADDR) {
			ezDSP_uState = STAT_ADDR;
			ezDSP_uAddrRdCnt = 0;
		}
		else if(ezDSP_ucRx == CMD_READ2B) {
			ezDSP_ulAddr++;	// auto increment
			ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;
			
			AddRing(ezDSP_uData >> 8);	// MSB
			AddRing(ezDSP_uData);		// LSB
			AddRing(CMD_FB_READ);
			
			ezDSP_uState = STAT_INIT;
			TX_START;
		}
		else if(ezDSP_ucRx == CMD_READ16B) {
			ezDSP_ulAddr += 8;
			for(uIndex = 0; uIndex < 8; uIndex++) {
				// Since this is not for variable, addresss is increased sequentially
				ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + uIndex);
				AddRing(ezDSP_uData >> 8);		// MSB
				AddRing(ezDSP_uData);			// LSB
			}
			AddRing(CMD_FB_READ);
	
			ezDSP_uState = STAT_INIT;
			TX_START;
		}
		else if(ezDSP_ucRx == CMD_DATA2B) {
			ezDSP_ulAddr++;	// auto increment

			ezDSP_uState = STAT_DATA2B;
			ezDSP_uDataRdCnt = 0;
		}
		else if(ezDSP_ucRx == CMD_DATA4B) {
			ezDSP_ulAddr += 2;	// auto increment

			ezDSP_uState = STAT_DATA4B;
			ezDSP_uDataRdCnt = 0;
		}
		// v8.5
		else if(ezDSP_ucRx == CMD_DATA8B) {
			ezDSP_ulAddr += 4;	// auto increment

			ezDSP_uState = STAT_DATA8B;
			ezDSP_uDataRdCnt = 0;
		}

	}
	else if(ezDSP_uState == STAT_ADDR) {
		ezDSP_uAddrRdCnt++;
		if(ezDSP_uAddrRdCnt == 1) {
			ezDSP_ulAddr = ezDSP_ucRx; 			// MSB
			ezDSP_ulAddr <<= 16; 				// MSB
		}
		else if(ezDSP_uAddrRdCnt == 2)
			ezDSP_ulAddr |= (ezDSP_ucRx << 8);

		else if(ezDSP_uAddrRdCnt == 3)
			ezDSP_ulAddr |= ezDSP_ucRx;			// LSB

		else if(ezDSP_uAddrRdCnt == 4) {
			if(ezDSP_ucRx == CMD_READ2B) {
				ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;
				
				AddRing(ezDSP_uData >> 8);	// MSB
				AddRing(ezDSP_uData);		// LSB
				AddRing(CMD_FB_READ);
				
				ezDSP_uState = STAT_INIT;
				TX_START;
			}
			else if(ezDSP_ucRx == CMD_READ4B) {
				ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + 1);
				AddRing(ezDSP_uData >> 8);	// MSB
				AddRing(ezDSP_uData);		

				ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;
				AddRing(ezDSP_uData >> 8);	
				AddRing(ezDSP_uData);		// LSB

				AddRing(CMD_FB_READ);
				
				ezDSP_uState = STAT_INIT;
				TX_START;
			}
			else if(ezDSP_ucRx == CMD_READ8B) {
				ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + 3);
				AddRing(ezDSP_uData >> 8);	// MSB
				AddRing(ezDSP_uData);		

				ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + 2);
				AddRing(ezDSP_uData >> 8);	
				AddRing(ezDSP_uData);

				ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + 1);
				AddRing(ezDSP_uData >> 8);
				AddRing(ezDSP_uData);		

				ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;
				AddRing(ezDSP_uData >> 8);	
				AddRing(ezDSP_uData);		// LSB

				AddRing(CMD_FB_READ);
				
				ezDSP_uState = STAT_INIT;
				TX_START;
			}
			else if(ezDSP_ucRx == CMD_READ16B) {
				for(uIndex = 0; uIndex < 8; uIndex++) {
					// Since this is not for variable, addresss is increased sequentially
					ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + uIndex);
					AddRing(ezDSP_uData >> 8);		// MSB
					AddRing(ezDSP_uData);			// LSB
				}
				AddRing(CMD_FB_READ);
				
				ezDSP_uState = STAT_INIT;
				TX_START;
			}
			else if(ezDSP_ucRx == CMD_DATA2B) {
				ezDSP_uState = STAT_DATA2B;
				ezDSP_uDataRdCnt = 0;
			}
			else if(ezDSP_ucRx == CMD_DATA4B) {
				ezDSP_uState = STAT_DATA4B;
				ezDSP_uDataRdCnt = 0;
			}
			else if(ezDSP_ucRx == CMD_DATA8B) {
				ezDSP_uState = STAT_DATA8B;
				ezDSP_uDataRdCnt = 0;
			}
			// v8.5
			else if(ezDSP_ucRx == CMD_DATA_BLOCK) {
				ezDSP_uState = STAT_DATA_BLOCK;
				ezDSP_uDataRdCnt = 0;
			}
			else ezDSP_uState = STAT_INIT;
		}
		else 
			ezDSP_uState = STAT_INIT;
	}
	
	else if(ezDSP_uState == STAT_DATA2B) {
		ezDSP_uDataRdCnt++;
		if(ezDSP_uDataRdCnt == 1)
			ezDSP_uData = ezDSP_ucRx << 8; 		// MSB
		else if(ezDSP_uDataRdCnt == 2)
			ezDSP_uData |= ezDSP_ucRx; 			// LSB
		else if(ezDSP_uDataRdCnt == 3) 		
			ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
		else if(ezDSP_uDataRdCnt == 4)
			ezDSP_uChksum |= ezDSP_ucRx;		// LSB
		else if(ezDSP_uDataRdCnt == 5) {
			if(ezDSP_ucRx == CMD_WRITE) {
				if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_uData) & 0xFFFF)) {
					*(unsigned int*)ezDSP_ulAddr = ezDSP_uData;
					AddRing(CMD_FB_WRITE_OK);
					ezDSP_uState = STAT_INIT;				
				}
				else {
					AddRing(CMD_FB_WRITE_NG);
					ezDSP_uState = STAT_INIT;
				}
				TX_START;
			}
			else
				ezDSP_uState = STAT_INIT;
		}
		else
			ezDSP_uState = STAT_INIT;
	}
	else if(ezDSP_uState == STAT_DATA4B) {
		ezDSP_uDataRdCnt++;
		if(ezDSP_uDataRdCnt == 1) {
			ezDSP_ulData = ezDSP_ucRx; 		// MSB
			ezDSP_ulData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 2) {
			ezDSP_ulData |= ezDSP_ucRx;
			ezDSP_ulData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 3) {
			ezDSP_ulData |= ezDSP_ucRx;
			ezDSP_ulData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 4) {
			ezDSP_ulData |= ezDSP_ucRx;
		}
		else if(ezDSP_uDataRdCnt == 5) 		
			ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
		else if(ezDSP_uDataRdCnt == 6)
			ezDSP_uChksum |= ezDSP_ucRx;		// LSB
		else if(ezDSP_uDataRdCnt == 7) {
			if(ezDSP_ucRx == CMD_WRITE) {
				if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_ulData) & 0xFFFF)) {
					*(unsigned long*)ezDSP_ulAddr = ezDSP_ulData;
					AddRing(CMD_FB_WRITE_OK);
					ezDSP_uState = STAT_INIT;				
				}
				else {
					AddRing(CMD_FB_WRITE_NG);
					ezDSP_uState = STAT_INIT;
				}
				TX_START;
			}
			else
				ezDSP_uState = STAT_INIT;
		}
		else
			ezDSP_uState = STAT_INIT;
	}
	// new
	else if(ezDSP_uState == STAT_DATA8B) {
		ezDSP_uDataRdCnt++;
		if(ezDSP_uDataRdCnt == 1) {
			ezDSP_ullData = ezDSP_ucRx; 		// MSB
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 2) {
			ezDSP_ullData |= ezDSP_ucRx;
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 3) {
			ezDSP_ullData |= ezDSP_ucRx;
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 4) {
			ezDSP_ullData |= ezDSP_ucRx;
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 5) {
			ezDSP_ullData |= ezDSP_ucRx;
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 6) {
			ezDSP_ullData |= ezDSP_ucRx;
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 7) {
			ezDSP_ullData |= ezDSP_ucRx;
			ezDSP_ullData <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 8) {
			ezDSP_ullData |= ezDSP_ucRx;
		}
		else if(ezDSP_uDataRdCnt == 9) 		
			ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
		else if(ezDSP_uDataRdCnt == 10)
			ezDSP_uChksum |= ezDSP_ucRx;		// LSB
		else if(ezDSP_uDataRdCnt == 11) {
			if(ezDSP_ucRx == CMD_WRITE) {
				if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_ullData) & 0xFFFF)) {
					*(unsigned long long*)ezDSP_ulAddr = ezDSP_ullData;
					AddRing(CMD_FB_WRITE_OK);
					ezDSP_uState = STAT_INIT;				
				}
				else {
					AddRing(CMD_FB_WRITE_NG);
					ezDSP_uState = STAT_INIT;
				}
				TX_START;
			}
			else
				ezDSP_uState = STAT_INIT;
		}
		else
			ezDSP_uState = STAT_INIT;
	}
#ifdef FLASH_API_WRAPPER
	// v8.5
	else if(ezDSP_uState == STAT_DATA_BLOCK) {
		ezDSP_uDataRdCnt++;
		// block size
		if(ezDSP_uDataRdCnt == 1) {
			ezDSP_uBlockSize = ezDSP_ucRx; 		// MSB. size in word
			ezDSP_uBlockSize <<= 8;
		}
		else if(ezDSP_uDataRdCnt == 2) {
			ezDSP_uBlockSize |= ezDSP_ucRx;		// LSB
			ezDSP_uChkSumCalculated = ezDSP_uBlockSize + ezDSP_ulAddr;
			ezDSP_uBlockIndex = 0;
		}
		// data transfer : ezDSP_uBlockSize = in words
		else if((ezDSP_uDataRdCnt >= 3) && (ezDSP_uDataRdCnt < (3 + ezDSP_uBlockSize*2))) {
			if(ezDSP_uBlockIndex == 0) {
				ezDSP_uData =  ezDSP_ucRx;
				ezDSP_uBlockIndex = 1;
			}
			else if(ezDSP_uBlockIndex == 1 ) {
				ezDSP_uData <<= 8;
				ezDSP_uData |=  ezDSP_ucRx;
				ezDSP_uBlockIndex = 0;

				// transfer
				*(unsigned int*)ezDSP_ulAddr = ezDSP_uData;
				ezDSP_uChkSumCalculated += ezDSP_uData;
				ezDSP_ulAddr++;
			}
		}
		else if(ezDSP_uDataRdCnt == (3 + ezDSP_uBlockSize*2 + 0)) 	ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
		else if(ezDSP_uDataRdCnt == (3 + ezDSP_uBlockSize*2 + 1))	ezDSP_uChksum |= ezDSP_ucRx;		// LSB
		else if(ezDSP_uDataRdCnt == (3 + ezDSP_uBlockSize*2 + 2)) {
			if(ezDSP_ucRx == CMD_WRITE) {
				if(ezDSP_uChksum == ezDSP_uChkSumCalculated)	AddRing(CMD_FB_WRITE_OK);
				else 											AddRing(CMD_FB_WRITE_NG);
				ezDSP_uState = STAT_INIT;
				TX_START;
			}
			else
				ezDSP_uState = STAT_INIT;
		}
		else
			ezDSP_uState = STAT_INIT;
	}
#endif
	else {
		ezDSP_uState = STAT_INIT;
	}

	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all |= 0x100;
}
                                              
interrupt void easy_TXINT_ISR(void)
{
	///////////////////////////////////////////////////////////////////////////
	// interrupt nesting to allow another interrupts before the completion of this ISR
	// Please kindly use your own code for this purpose if necessary
	///////////////////////////////////////////////////////////////////////////
	//EINT;
	//IER |= M_INT9;
	//PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
	///////////////////////////////////////////////////////////////////////////

	// buffer => sio
	if(!IsRingEmpty()) {
		if(SciaRegs.SCICTL2.bit.TXRDY)	// check TXRDY
			SciaRegs.SCITXBUF.all = ExtractRing();
		else 
			ezDSP_uWrongISRCount++;
	}
	else {
		TX_STOP;
	}

	PieCtrlRegs.PIEACK.all |= 0x100;
}
