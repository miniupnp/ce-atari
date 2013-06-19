#include "stm32f10x.h"                       // STM32F103 definitions
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "misc.h"

#include "defs.h"
#include "timers.h"

void init_hw_sw(void);

void fillMfmTimesForDMA(void);
BYTE getNextMFMword(void);

void getMfmWriteTimes(void);

void addAttention(BYTE atn);
void processHostCommand(WORD hostWord);

void spi_init(void);

void dma_mfmRead_init(void);
void dma_mfmWrite_init(void);
void dma_spi_init(void);

/*
TODO:
 - test TIM3 input capture bez DMA, potom s DMA
 - test DMA SPI
 - test TIM1, TIM2 po presune!
 - pomeranie kolko trvaju jednotlive casti kodu
*/

// 1 sector with all headers, gaps, data and crc: 614 B of data needed to stream -> 4912 bits to stream -> with 4 bits encoded to 1 byte: 1228 of encoded data
// Each sector takes around 22ms to stream out, so you have this time to ask for new one and get it in... the SPI transfer of 1228 B should take 0,8 ms.
// inBuffer size of 2048 WORDs can contain 3.3 encoded sectors

// gap byte 0x4e == 666446 us = 222112 times = 10'10'10'01'01'10 = 0xa96
// first gap (60 * 0x4e) takes 1920 us (1.9 ms)

#define INBUFFER_SIZE					2048
WORD inBuffer[INBUFFER_SIZE];
WORD inIndexGet;

TAtnBuffer atnBuffer[2];							// two buffers for ATNs 
TAtnBuffer *atnNow;

TWriteBuffer wrBuffer[2];							// two buffers for written sectors
TWriteBuffer *wrNow;

BYTE side, track, sector;

WORD mfmReadStreamBuffer[16];							// 16 words - 16 mfm times. Half of buffer is 8 times - at least 32 us (8 * 4us),
BYTE stream4e;												// the count of 0x4e bytes we should stream (used for the 1st gap)

WORD mfmWriteStreamBuffer[16];
WORD lastMfmWriteTC;

// cycle measure: t1 = TIM3->CNT;	t2 = TIM3->CNT;	dt = t2 - t1; -- subtrack 0x12 because that's how much measuring takes
WORD t1, t2, dt; 


// commands sent from device to host
#define ATN_FW_VERSION					0x01								// followed by string with FW version (length: 4 WORDs - cmd, v[0], v[1], 0)
#define ATN_SEND_NEXT_SECTOR    0x02               	// sent: 2, side, track #, current sector #, 0, 0, 0, 0 (length: 4 WORDs)
#define ATN_SECTOR_WRITTEN      0x03               	// sent: 3, side (highest bit) + track #, current sector #

// commands sent from host to device
#define CMD_WRITE_PROTECT_OFF		0x10
#define CMD_WRITE_PROTECT_ON		0x20
#define CMD_DISK_CHANGE_OFF			0x30
#define CMD_DISK_CHANGE_ON			0x40
#define CMD_CURRENT_SECTOR			0x50								// followed by sector #
#define CMD_GET_FW_VERSION			0x60
#define CMD_MARK_READ						0xF000							// this is not sent from host, but just a mark that this WORD has been read and you shouldn't continue to read further

#define MFM_4US     1
#define MFM_6US     2
#define MFM_8US     3

// set GPIOB as --- CNF1:0 -- 00 (push-pull output), MODE1:0 -- 11 (output 50 Mhz)
#define 	FloppyOut_Enable()			{GPIOB->CRH &= ~(0x00000fff); GPIOB->CRH |=  (0x00000333); FloppyIndex_Enable();	FloppyMFMread_Enable();		}
// set GPIOB as --- CNF1:0 -- 01 (floating input), MODE1:0 -- 00 (input)
#define 	FloppyOut_Disable()			{GPIOB->CRH &= ~(0x00000fff); GPIOB->CRH |=  (0x00000444); FloppyIndex_Disable();	FloppyMFMread_Disable();	}

// enable / disable TIM1 CH1 output on GPIOA_8
#define		FloppyMFMread_Enable()		{ GPIOA->CRH &= ~(0x0000000f); GPIOA->CRH |= (0x0000000b); };
#define		FloppyMFMread_Disable()		{ GPIOA->CRH &= ~(0x0000000f); GPIOA->CRH |= (0x00000004); };

// enable / disable TIM2 CH4 output on GPIOB_11
#define		FloppyIndex_Enable()			{ GPIOB->CRH &= ~(0x0000f000); GPIOB->CRH |= (0x0000b000); };
#define		FloppyIndex_Disable()			{ GPIOB->CRH &= ~(0x0000f000); GPIOB->CRH |= (0x00004000); };

//--------------
// circular buffer 
// watch out, these macros take 0.73 us for _add, and 0.83 us for _get operation!

#define		atnBuffer_add(X)				{ if(atnNow->count < 16)	{		atnNow->buffer[atnNow->count]	= X;	atnNow->count++;	}	}
#define		wrBuffer_add(X)					{ if(wrNow->count  < 550)	{		wrNow->buffer[wrNow->count]		= X;	wrNow->count++;		}	}

#define		inBuffer_get(X)					{ X = inBuffer[inIndexGet];								inIndexGet++;		inIndexGet = inIndexGet	& 0x7ff;	}
#define		inBuffer_get_noMove(X)	{ X = inBuffer[inIndexGet];																																	}
#define		inBuffer_markAndmove()	{ inBuffer[inIndexGet] = CMD_MARK_READ;		inIndexGet++;		inIndexGet = inIndexGet	& 0x7ff;	}
//--------------

WORD version[2] = {0xf013, 0x0527};				// this means: Franz, 2013-05-27

int main (void) 
{
	BYTE prevSide = 0, outputsActive = 0;
	WORD prevWGate = WGATE, WGate;
	
	init_hw_sw();																	// init GPIO pins, timers, DMA, global variables

	// init floppy signals
	GPIOB->BSRR = (WR_PROTECT | DISK_CHANGE);			// not write protected, not changing disk (ready!)
	GPIOB->BRR = TRACK0;													// TRACK 0 signal to L		
	GPIOB->BRR = ATN;															// ATTENTION bit low - nothing to read
	
	// whole loop without INDEX and MOTOR_ENABLE checking: 2.8 us (other bits), 3.7 us (when fetching new mfm byte)
	while(1) {
		WORD ints;
		WORD inputs = GPIOB->IDR;								// read floppy inputs

//		if((inputs & (MOTOR_ENABLE | DRIVE_SELECT)) != 0) {		// motor not enabled, drive not selected? do nothing
		if(0) {
			if(outputsActive == 1) {							// if we got outputs active
				FloppyOut_Disable();								// all pins as inputs, drive not selected
				outputsActive = 0;
			}

			// with the floppy disabled, drop the data from inBuffer, but process the host commands
			while(1) {														// empty the buffer
				WORD wval;
				inBuffer_get_noMove(wval);					// get byte from host buffer

				if(wval == CMD_MARK_READ) {					// we've hit the already read part of buffer? (underflow) quit loop
					break;
				}
				
				inBuffer_markAndmove();							// mark this word as read and move to the next one
				
				if((wval & 0x0f00) == 0) {					// lower nibble == 0? it's a command from host, process it; otherwise drop it
					processHostCommand(wval);
				}
			}

			continue;
		}

		if(outputsActive == 0) {								// if the outputs are inactive
			FloppyOut_Enable();										// set these as outputs
			outputsActive = 1;
		}
		
		WGate = inputs & WGATE;												// get current WGATE value
		if(prevWGate != WGate) {											// if WGate changed
			
			if(WGate == 0) {														// on falling edge of WGATE
				WORD wval;

				lastMfmWriteTC = TIM3->CNT;								// set lastMfmWriteTC to current TC value on WRITE start
				wrNow->readyToSend = FALSE;								// mark this buffer as not ready to be sent yet
				
				wval = (ATN_SECTOR_WRITTEN << 8) | side;	// 1st byte: ATN code, 2nd byte: side 
				wrBuffer_add(wval);
	
				wval = (track << 8) | sector;							// 3rd byte: track #, 4th byte: current sector #
				wrBuffer_add(wval);									
			} else {																		// on rising edge
				wrBuffer_add(0);													// last word: 0
				
				wrNow->readyToSend = TRUE;								// mark this buffer as ready to be sent
			}			
			
			prevWGate = WGate;													// store WGATE value
		}
		
		if(WGate == 0) {																				// when write gate is low, the data is written to floppy
			if((DMA1->ISR & (DMA1_IT_TC6 | DMA1_IT_HT6)) != 0) {	// MFM write stream: TC or HT interrupt? we've streamed half of circular buffer!
				getMfmWriteTimes();
			}
		}

		if((DMA1->ISR & (DMA1_IT_TC5 | DMA1_IT_HT5)) != 0) {		// MFM read stream: TC or HT interrupt? we've streamed half of circular buffer!
			fillMfmTimesForDMA();																	// fill the circular DMA buffer with mfm times
		}

		if(DMA1_Channel3->CNDTR == 0) {													// SPI TX: nothing to transfer?
			BYTE sending = FALSE;
			
			if(atnNow->count > 0) {																// the current ATN buffer holds some ATNs to send?
				sending = TRUE;
				
				DMA1_Channel3->CCR		&= 0xfffffffe;								// disable DMA1 Channel3 transfer
				DMA1_Channel3->CMAR		= (uint32_t) atnNow->buffer;	// from this buffer located in memory
				DMA1_Channel3->CNDTR	= atnNow->count;							// this much data
				DMA1_Channel3->CCR		|= 1;													// enable DMA1 Channel3 transfer
				
				atnNow					= atnNow->next;											// and now we will select the next buffer as current
				atnNow->count		= 0;
			}

			if(sending == FALSE && wrNow->readyToSend == TRUE) {		// not sending any ATN right now? and current write buffer has something?
				sending = TRUE;

				DMA1_Channel3->CCR		&= 0xfffffffe;								// disable DMA1 Channel3 transfer
				DMA1_Channel3->CMAR		= (uint32_t) wrNow->buffer;		// from this buffer located in memory
				DMA1_Channel3->CNDTR	= wrNow->count;								// this much data
				DMA1_Channel3->CCR		|= 1;													// enable DMA1 Channel3 transfer
				
				wrNow->readyToSend	= FALSE;												// mark the current buffer as not ready to send (so we won't send this one again)
				
				wrNow								= wrNow->next;									// and now we will select the next buffer as current
				wrNow->readyToSend	= FALSE;												// the next buffer is not ready to send (yet)
				wrNow->count				= 0;														
			}
			
			if(sending == TRUE) {
				GPIOB->BSRR = ATN;																	// ATTENTION bit high - got something to read
			} else {
				DMA1_Channel3->CCR		&= 0xfffffffe;								// disable DMA1 Channel3 transfer
				GPIOB->BRR = ATN;																		// ATTENTION bit low  - nothing to read
			}				 
		}

		// check for STEP pulse - should we go to a different track?
		ints = EXTI->PR;										// Pending register (EXTI_PR)
		if(ints & STEP) {										// if falling edge of STEP signal was found
			EXTI->PR = STEP;									// clear that int

			if(inputs & DIR) {								// direction is High? track--
				if(track > 0) {
					track--;

					addAttention(ATN_SEND_NEXT_SECTOR);
				}
			} else {											// direction is Low? track++
				if(track < 82) {
					track++;

					addAttention(ATN_SEND_NEXT_SECTOR);
				}
			}

			if(track == 0) {									// if track is 0
				GPIOB->BRR = TRACK0;						// TRACK 0 signal to L			
			} else {													// if track is not 0
				GPIOB->BSRR = TRACK0;						// TRACK 0 signal is H
			}
		}

		//------------
		// update SIDE var
		side = (inputs & SIDE1) ? 0 : 1;					// get the current SIDE
		if(prevSide != side) {										// side changed?
			addAttention(ATN_SEND_NEXT_SECTOR);			// we need another sector, this time from the right side!
			prevSide = side;
		}

		//------------
		// check INDEX pulse as needed -- duration when filling with marks: 69.8 us
// 	if((TIM2->SR & 0x0001) != 0) {		// overflow of TIM1 occured?
		if(0) {
			static BYTE broken4e[16] = {11,7,7,11,11,11,11,7,7,11,11,11,11,7,7,11};
			int i;
			TIM2->SR = 0xfffe;							// clear UIF flag
	
			stream4e = 60;									// start of the track -- we should stream 60* 0x4e
			sector = 1;
			
			for(i=0; i<16; i++) {						// copy in the 'broken' 0x4e sequence to start streaming 0x4e immediately 
				mfmReadStreamBuffer[i] = broken4e[i];
			}
			
			addAttention(ATN_SEND_NEXT_SECTOR);
		}
	}
}

void init_hw_sw(void)
{
	WORD i;
	
	RCC->AHBENR		|= (1 <<  0);																						// enable DMA1
	RCC->APB1ENR	|= (1 <<  1) | (1 <<  0);																// enable TIM3, TIM2
  RCC->APB2ENR	|= (1 << 12) | (1 << 11) | (1 << 3) | (1 << 2);     		// Enable SPI1, TIM1, GPIOA and GPIOB clock
	
	// set FLOATING INPUTs for GPIOB_0 ... 6
	GPIOB->CRL &= ~(0x0fffffff);						// remove bits from GPIOB
	GPIOB->CRL |=   0x04444444;							// set GPIOB as --- CNF1:0 -- 01 (floating input), MODE1:0 -- 00 (input), PxODR -- don't care

	FloppyOut_Disable();
	
	// SPI -- enable atlernate function for PA4, PA5, PA6, PA7
	GPIOA->CRL &= ~(0xffff0000);						// remove bits from GPIOA
	GPIOA->CRL |=   0xbbbb0000;							// set GPIOA as --- CNF1:0 -- 10 (push-pull), MODE1:0 -- 11, PxODR -- don't care

	// ATTENTION -- set GPIOB_15 (ATTENTION) as --- CNF1:0 -- 00 (push-pull output), MODE1:0 -- 11 (output 50 Mhz)
	GPIOB->CRH &= ~(0xf0000000); 
	GPIOB->CRH |=  (0x30000000);

	RCC->APB2ENR |= (1 << 0);									// enable AFIO
	
	AFIO->EXTICR[0] = 0x1000;									// EXTI3 -- source input: GPIOB_3
	EXTI->IMR			= STEP;											// EXTI3 -- 1 means: Interrupt from line 3 not masked
	EXTI->EMR			= STEP;											// EXTI3 -- 1 means: Event     form line 3 not masked
	EXTI->FTSR 		= STEP;											// Falling trigger selection register - STEP pulse
	
	//----------
	AFIO->MAPR |= 0x02000000;									// SWJ_CFG[2:0] (Bits 26:24) -- 010: JTAG-DP Disabled and SW-DP Enabled
	AFIO->MAPR |= 0x00000300;									// TIM2_REMAP -- Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)
	AFIO->MAPR |= 0x00000800;									// TIM3_REMAP -- Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)
	//----------
	timerSetup_index();
	
	timerSetup_mfmWrite();
	dma_mfmWrite_init();
	//--------------
	// DMA + SPI initialization
	for(i=0; i<INBUFFER_SIZE; i++) {							// fill the inBuffer with CMD_MARK_READ, which will tell that every byte is empty (already read)
		inBuffer[i] = CMD_MARK_READ;
	}
	
	spi_init();																		// init SPI interface
	dma_spi_init();

	// init both ATN buffers - mark them as empty, not sending, use first as current
	atnBuffer[0].count		= 0;			
	atnBuffer[0].next			= &atnBuffer[1];
	
	atnBuffer[1].count		= 0;
	atnBuffer[1].next			= &atnBuffer[0];

	atnNow = &atnBuffer[0];
	
	// now init both writeBuffers
	wrBuffer[0].count				= 0;
	wrBuffer[0].readyToSend	= FALSE;
	wrBuffer[0].next				= &wrBuffer[1];
	
	wrBuffer[1].count				= 0;
	wrBuffer[1].readyToSend	= FALSE;
	wrBuffer[1].next				= &wrBuffer[0];

	wrNow = &wrBuffer[0];
	//--------------
	// configure MFM read stream by TIM2 CH4 and DMA in circular mode
	// WARNING!!! Never let mfmReadStreamBuffer[] contain a 0! With 0 the timer update never comes and streaming stops!
	for(i=0; i<16; i++) {
		mfmReadStreamBuffer[i] = 7;				// by default -- all pulses 4 us
	}
	
	timerSetup_mfmRead();								// init MFM READ stream timer
	dma_mfmRead_init();									// and init the DMA which will feed it from circular buffer
	//--------------
	
	// init circular buffer for data incomming via SPI
	inIndexGet		= 0;
	
	// init track and side vars for the floppy position
	side					= 0;
	track 				= 0;
	sector				= 1;
	
	stream4e			= 0;			// mark that we shouldn't stream any 0x4e (yet)
	
	lastMfmWriteTC = 0;
}

void getMfmWriteTimes(void)
{
	WORD ind1, ind2, i, tmp, val, wval;
	
	// check for half transfer or transfer complete IF
	if((DMA1->ISR & DMA1_IT_HT6) != 0) {				// HTIF6 -- Half Transfer IF 6
		DMA1->IFCR = DMA1_IT_HT6;									// clear HTIF6 flag
		ind1 = 7;
		ind2 = 0;
	} else if((DMA1->ISR & DMA1_IT_TC6) != 0) {	// TCIF6 -- Transfer Complete IF 6
		DMA1->IFCR = DMA1_IT_TC6;									// clear TCIF6 flag
		ind1 = 15;
		ind2 = 8;
	}

	tmp = mfmWriteStreamBuffer[ind1];						// store the last one, we will move it to lastMfmWriteTC later
	
	for(i=0; i<7; i++) {												// create differences: this = this - previous; but only for 7 values
		mfmWriteStreamBuffer[ind1] = mfmWriteStreamBuffer[ind1] - mfmWriteStreamBuffer[ind1-1];
		ind1--;
	}

	mfmWriteStreamBuffer[ind1] = mfmWriteStreamBuffer[ind1] - lastMfmWriteTC;			// buffer[0] = buffer[0] - previousBuffer[15];  or buffer[8] = buffer[8] - previousBuffer[7];
	lastMfmWriteTC = tmp;																													// store current last item as last
	
	wval = 0;																		// init to zero
	
	for(i=0; i<8; i++) {												// now convert timer counter times to MFM times / codes
		tmp = mfmWriteStreamBuffer[ind2];
		ind2++;
		
		if(tmp < 200) {						// too short? skip it
			continue;
		} else if(tmp < 360) {		// 4 us?
			val = MFM_4US;
		} else if(tmp < 504) {		// 6 us?
			val = MFM_6US;
		} else if(tmp < 650) {		// 8 us?
			val = MFM_8US;
		} else {									// too long?
			continue;
		}
		
		wval = wval << 2;					// shift and add to the WORD 
		wval = wval | val;
	}
	
	wrBuffer_add(wval);					// add to write buffer
}

void fillMfmTimesForDMA(void)
{
	WORD ind = 0xff;

	// code to ARR value:      ??, 4us, 6us, 8us
	static WORD mfmTimes[4] = { 7,   7,  11,  15};
	BYTE time, i;
	WORD times8;

	// check for half transfer or transfer complete IF
	if((DMA1->ISR & DMA1_IT_HT5) != 0) {				// HTIF5 -- Half Transfer IF 5
		DMA1->IFCR = DMA1_IT_HT5;									// clear HTIF5 flag
		ind = 0;
	} else if((DMA1->ISR & DMA1_IT_TC5) != 0) {	// TCIF5 -- Transfer Complete IF 5
		DMA1->IFCR = DMA1_IT_TC5;									// clear TCIF5 flag
		ind = 8;
	}
	
	if(ind == 0xff) {														// no IF found? this shouldn't happen! did you come here without a reason?
		return;
	}
	
	times8 = getNextMFMword();									// get next word
	
	for(i=0; i<8; i++) {													// convert all 8 codes to ARR values
		time		= times8 >> 14;											// get bits 15,14 (and then 13,12 ... 1,0) 
		time		= mfmTimes[time];										// convert to ARR value
		times8	= times8 << 2;											// shift 2 bits higher so we would get lower bits next time

		mfmReadStreamBuffer[ind] = time;								// store and move to next one
		ind++;
	}
}

BYTE getNextMFMword(void)
{
	static WORD gap[3] = {0xa96a, 0x96a9, 0x6a96};
	static BYTE gapIndex = 0;
	WORD wval;

	if(stream4e == 0) {
		while(1) {														// go through inBuffer to process commands and to find some data
			inBuffer_get_noMove(wval);					// get WORD from buffer, but don't move
		
			if(wval == CMD_MARK_READ) {					// we've hit the already read part of buffer? (underflow) quit loop
				break;
			}
			
			inBuffer_markAndmove();							// mark this word as read
			
			// lower nibble == 0? it's a command from host - if we should turn on/off the write protect or disk change
			if((wval & 0x0f00) == 0) {					// it's a command?
				processHostCommand(wval);
			} else {														// not a command? return it
				gapIndex = 0;		
				return wval;
			}
		}
	}

	//---------
	// if we got here, we have no data to stream
	wval = gap[gapIndex];					// stream this GAP byte
	gapIndex++;

	if(gapIndex > 2) {						// we got only 3 gap WORD times, go back to 0
		gapIndex = 0;
		
		if(stream4e > 0) {					// if we streamed 3 gap elements, we've streamed 4* 0x4e
			if(stream4e < 4) {				// for 0...3 resize to 0
				stream4e = 0;
			} else {									// for higher numbers subtract 4
				stream4e -= 4;
			}
		}		
	}

	return wval;
}

void processHostCommand(WORD hostWord)
{
	BYTE hi, lo;
	hi = hostWord >> 8;												// get upper byte
	lo = hostWord  & 0xff;										// get lower byte
	
	switch(hi) {
		case CMD_WRITE_PROTECT_OFF:		GPIOB->BSRR	= WR_PROTECT;		break;			// WR PROTECT to 1
		case CMD_WRITE_PROTECT_ON:		GPIOB->BRR	= WR_PROTECT;		break;			// WR PROTECT to 0
		case CMD_DISK_CHANGE_OFF:			GPIOB->BSRR	= DISK_CHANGE;	break;			// DISK_CHANGE to 1
		case CMD_DISK_CHANGE_ON:			GPIOB->BRR	= DISK_CHANGE;	break;			// DISK_CHANGE to 0

		case CMD_GET_FW_VERSION:								// send FW version string
		{
			atnBuffer_add(ATN_FW_VERSION);				// command		(this adds 0 and ATN_FW_VERSION)
			atnBuffer_add(version[0]);
			atnBuffer_add(version[1]);
			atnBuffer_add(0);											// terminating zero(s) (0, 0)
			break;
		}
		
		case CMD_CURRENT_SECTOR:								// get the next byte, which is sector #, and store it in sector variable
			sector = lo;
			addAttention(ATN_SEND_NEXT_SECTOR);		// also ask for the next sector to this one
			break;			
	}
}

void addAttention(BYTE atn)
{
	WORD wval;
	
	wval = (atn << 8) | side;							// 1st byte: ATN code, 2nd byte: side 
	atnBuffer_add(wval);
	
	wval = (track << 8) | sector;					// 3rd byte: track #, 4th byte: current sector #
	atnBuffer_add(wval);									

	atnBuffer_add(0);											
	atnBuffer_add(0);											// last word: 0
}

void spi_init(void)
{
	SPI_InitTypeDef spiStruct;

	SPI_Cmd(SPI1, DISABLE);
	
	SPI_StructInit(&spiStruct);
	spiStruct.SPI_DataSize = SPI_DataSize_16b;		// use 16b data size to lower the MCU load
	
  SPI_Init(SPI1, &spiStruct);
	SPI1->CR2 |= (1 << 7) | (1 << 6) | SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx;		// enable TXEIE, RXNEIE, TXDMAEN, RXDMAEN
	
	SPI_Cmd(SPI1, ENABLE);
}

void dma_mfmRead_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	// DMA1 channel5 configuration -- TIM1_UP (update)
  DMA_DeInit(DMA1_Channel5);
	
  DMA_InitStructure.DMA_MemoryBaseAddr			= (uint32_t) mfmReadStreamBuffer;				// from this buffer located in memory
  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t) &(TIM1->DMAR);					// to this peripheral address
  DMA_InitStructure.DMA_DIR									= DMA_DIR_PeripheralDST;						// dir: from mem to periph
  DMA_InitStructure.DMA_BufferSize					= 16;																// 16 datas to transfer
  DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;				// PINC = 0 -- don't icrement, always write to DMAR register
  DMA_InitStructure.DMA_MemoryInc						= DMA_MemoryInc_Enable;							// MINC = 1 -- increment in memory -- go though buffer
  DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;	// each data item: 16 bits 
  DMA_InitStructure.DMA_MemoryDataSize			= DMA_MemoryDataSize_HalfWord;			// each data item: 16 bits 
  DMA_InitStructure.DMA_Mode								= DMA_Mode_Circular;								// circular mode
  DMA_InitStructure.DMA_Priority						= DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M									= DMA_M2M_Disable;									// M2M disabled, because we move to peripheral
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

  // Enable DMA1 Channel5 Transfer Complete interrupt
  DMA_ITConfig(DMA1_Channel5, DMA_IT_HT, ENABLE);			// interrupt on Half Transfer (HT)
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);			// interrupt on Transfer Complete (TC)

  // Enable DMA1 Channel5 transfer
  DMA_Cmd(DMA1_Channel5, ENABLE);
}

void dma_mfmWrite_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	// DMA1 channel6 configuration -- TIM3_CH1 (capture)
  DMA_DeInit(DMA1_Channel6);
	
  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t) &(TIM3->DMAR);					// from this peripheral address
  DMA_InitStructure.DMA_MemoryBaseAddr			= (uint32_t) mfmWriteStreamBuffer;	// to this buffer located in memory
  DMA_InitStructure.DMA_DIR									= DMA_DIR_PeripheralSRC;						// dir: from periph to mem
  DMA_InitStructure.DMA_BufferSize					= 16;																// 16 datas to transfer
  DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;				// PINC = 0 -- don't icrement, always read from DMAR register
  DMA_InitStructure.DMA_MemoryInc						= DMA_MemoryInc_Enable;							// MINC = 1 -- increment in memory -- go though buffer
  DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;	// each data item: 16 bits 
  DMA_InitStructure.DMA_MemoryDataSize			= DMA_MemoryDataSize_HalfWord;			// each data item: 16 bits 
  DMA_InitStructure.DMA_Mode								= DMA_Mode_Circular;								// circular mode
  DMA_InitStructure.DMA_Priority						= DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M									= DMA_M2M_Disable;									// M2M disabled, because we move to peripheral
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);

  // Enable DMA1 Channel6 Transfer Complete interrupt
  DMA_ITConfig(DMA1_Channel6, DMA_IT_HT, ENABLE);			// interrupt on Half Transfer (HT)
  DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);			// interrupt on Transfer Complete (TC)

  // Enable DMA1 Channel6 transfer
  DMA_Cmd(DMA1_Channel6, ENABLE);
}

void dma_spi_init(void)
{
	// SPI1_TX -- DMA1_CH3
	DMA_InitTypeDef DMA_InitStructure;
	
	// DMA1 channel3 configuration -- SPI1 TX
  DMA_DeInit(DMA1_Channel3);
	
  DMA_InitStructure.DMA_MemoryBaseAddr			= (uint32_t) &atnBuffer[0];					// from this buffer located in memory
  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t) &(SPI1->DR);						// to this peripheral address
  DMA_InitStructure.DMA_DIR									= DMA_DIR_PeripheralDST;						// dir: from mem to periph
  DMA_InitStructure.DMA_BufferSize					= 0;																// nothing to transfer now
  DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;				// PINC = 0 -- don't icrement, always write to SPI1->DR register
  DMA_InitStructure.DMA_MemoryInc						= DMA_MemoryInc_Enable;							// MINC = 1 -- increment in memory -- go though buffer
  DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;	// each data item: 16 bits 
  DMA_InitStructure.DMA_MemoryDataSize			= DMA_MemoryDataSize_HalfWord;			// each data item: 16 bits 
  DMA_InitStructure.DMA_Mode								= DMA_Mode_Normal;									// normal mode
  DMA_InitStructure.DMA_Priority						= DMA_Priority_High;
  DMA_InitStructure.DMA_M2M									= DMA_M2M_Disable;									// M2M disabled, because we move to peripheral
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);

  // Enable interrupts
//  DMA_ITConfig(DMA1_Channel3, DMA_IT_HT, ENABLE);			// interrupt on Half Transfer (HT)
  DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);			// interrupt on Transfer Complete (TC)

  // Enable DMA1 Channel3 transfer
//  DMA_Cmd(DMA1_Channel3, ENABLE);
	
	//----------------
	// SPI1_RX -- DMA1_CH2
	// DMA1 channel2 configuration -- SPI1 RX
  DMA_DeInit(DMA1_Channel2);
	
  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t) &(SPI1->DR);						// from this peripheral address
  DMA_InitStructure.DMA_MemoryBaseAddr			= (uint32_t) inBuffer;							// to this buffer located in memory
  DMA_InitStructure.DMA_DIR									= DMA_DIR_PeripheralSRC;						// dir: from periph to mem
  DMA_InitStructure.DMA_BufferSize					= INBUFFER_SIZE;										// 2048 datas to transfer
  DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;				// PINC = 0 -- don't icrement, always write to SPI1->DR register
  DMA_InitStructure.DMA_MemoryInc						= DMA_MemoryInc_Enable;							// MINC = 1 -- increment in memory -- go though buffer
  DMA_InitStructure.DMA_PeripheralDataSize	= DMA_PeripheralDataSize_HalfWord;	// each data item: 16 bits 
  DMA_InitStructure.DMA_MemoryDataSize			= DMA_MemoryDataSize_HalfWord;			// each data item: 16 bits 
  DMA_InitStructure.DMA_Mode								= DMA_Mode_Circular;								// circular mode
  DMA_InitStructure.DMA_Priority						= DMA_Priority_High;
  DMA_InitStructure.DMA_M2M									= DMA_M2M_Disable;									// M2M disabled, because we move from peripheral
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  // Enable DMA1 Channel2 Transfer Complete interrupt
  DMA_ITConfig(DMA1_Channel2, DMA_IT_HT, ENABLE);			// interrupt on Half Transfer (HT)
  DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);			// interrupt on Transfer Complete (TC)

  // Enable DMA1 Channel2 transfer
  DMA_Cmd(DMA1_Channel2, ENABLE);
}