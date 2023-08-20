 /*
 * This is the main file of the STM8S library for DM63x LED driver chips
 * (c) 2016 Dmitry Reznkov, dmitry@ultiblink.com
 * This is free software under GNU General Public License.
 * You can use it any way you wish, but NO WARRANTY is provided.
 *
 * Special thanks: Alex Leone and his excellent Arduino TLC5940 library
 * that served as an inspiration for this one.
*/

#include <stdint.h>
//#include <stdio.h>
#include <stdlib.h>


// config defines from DMconfig.h

#define DM256PWM *pixel[k] // 8-bit resolution mode
                           // saves SRAM at the cost of a little speed
                           // possible values:
						   // *pixel[k] // square the value, slower but good with fade-outs
						   // <<8 // shift 8 bits (multiply by 256) - a bit faster but not as fancy
						   
#define DMCHAIN 3 // remap the LED driver outputs to be consistent with their physical locations
                    // when used in chain. Namely, if you put three DM chips on a breadboard, you'll want
                    // their outputs to be numbered 0-23 on the one side and 24-47 on the other.
					// DMCHAIN allows to do this without filling SRAM with look-up tables
					// (at the cost of speed and some program memory).
					// The value is the number of DM chips per single unit (1 in the aforementioned
					// case; 3 if used with my UltiBlink board).


// *********************************************************
// ************* MAIN PROGRAM STARTS AT LINE 443 ***********
// *********************************************************


// A bunch of addresses to avoid using HALs and stuff

//GPIO registers
#define _LATPORT_(mem_offset) (*(volatile uint8_t *)(LATport + (mem_offset)))

#define _ODR  0x00
#define _IDR  0x01
#define _DDR  0x02
#define _CR1  0x03
#define _CR2  0x04

#define PC_DDR *(volatile uint8_t *)0x00500C
#define PC_CR1 *(volatile uint8_t *)0x00500D
#define PC_CR2 *(volatile uint8_t *)0x00500E
#define PC_ODR *(volatile uint8_t *)0x00500A

//SPI registers

#define SPI_CR1		*(volatile uint8_t *)0x005200
#define SPI_CR2		*(volatile uint8_t *)0x005201
#define SPI_ICR		*(volatile uint8_t *)0x005202
#define SPI_SR		*(volatile uint8_t *)0x005203
#define SPI_DR		*(volatile uint8_t *)0x005204
#define SPI_CRCPR	*(volatile uint8_t *)0x005205
#define SPI_RXCRCR	*(volatile uint8_t *)0x005206
#define SPI_TXCRCR	*(volatile uint8_t *)0x005207

//SPI CR1 bits
#define CPHA        0x0001
#define CPOL        0x0002
#define MSTR        0x0004
#define BR_0        0x0008
#define BR_1        0x0010
#define BR_2        0x0020
#define SPE         0x0040
#define LSB_FIRST   0x0080

//SPI_CR2 bits
#define SSI         0x0001
#define SSM         0x0002
#define RX_ONLY     0x0004
//reserved bit		0x0008
#define CRC_NEXT    0x0010
#define CRC_EN      0x0020
#define BIDIOE      0x0040
#define BIDIMODE    0x0080

//SPI_SR bits
#define BSY         0x0080
#define TXE         0x0002

#define CLK_PCKENR1 *(volatile uint8_t *)0x0050C7


//LAT manipulations; _ODR offset is omitted for speed as it's 0x00

/*** Get LAT low (just in case) */
#define LAT_low() _LATPORT_(_ODR) &= ~(1<<LATpin) 
//#define LAT_low() PC_ODR &= ~(1<<4);

/*** LAT pulse - high, then low */
#define LAT_pulse() _LATPORT_(_ODR) |= (1<<LATpin); _LATPORT_(_ODR) &= ~(1<<LATpin)
//#define LAT_pulse() PC_ODR |=(1<<4); PC_ODR &= ~(1<<4)

/*** Get LAT high */
#define LAT_high() _LATPORT_(_ODR) |= (1<<LATpin);
//#define LAT_high() PC_ODR |=(1<<4);

volatile uint8_t* pixel;  //actual bytes sent to the driver

//DMLEDTABLE* ledTable; // led lookup table, type defined by DMLEDTABLE in config   
 
   //void dm_shift(uint8_t value);
   uint16_t getChainValue (uint16_t pixNum);
   //uint8_t LATbit, LATport;
   uint16_t pinNum, byteNum;
   //volatile uint8_t *LATreg, *LATout;
   uint8_t DMtype,  DMnum,  LATpin, SCKpin;
   uint32_t SPIreg, LATport, LATmode0, LATmode1, LATcnf0, SCKport, SCKmode, SCK_BSRR, SCK_BRR;
	 
void sendAll(void);
void clearAll(void);

void dm_shift(uint8_t value)
{
    SPI_DR = value; //send a byte
    while (!((SPI_SR) & (TXE))); //wait until it's sent
}

void DMdriverInit(uint8_t Driver, uint8_t Number, uint16_t LatchPort, uint8_t LatchPin)
{
	DMnum = Number; // number of DM chips in chain
	DMtype = Driver; // 16 or 12 bits (32 or 24 bytes per chip)
	LATpin = LatchPin; // latch pin
	LATport = LatchPort; // latch port
	pinNum = DMnum * 16; // number of outputs (single color LEDs)
#ifdef DM256PWM
	byteNum = pinNum; // if 8bit resolution was defined, 16 bytes per chip used
#else
	byteNum = DMnum * DMtype;
#endif

  pixel = malloc (byteNum*(sizeof(*pixel)));  //init the table of actual pixels
	//ledTable = 0; //table;
	
//SPI:
// MOSI PC6
// MISO PC7
// SCK  PC5
// NSS PA3

 
	// SPI setup
	// SPI: PC6(MOSI) and PC5(SCK) push pull
	PC_DDR |= (1<<6) | (1<<5); // output
	PC_CR1 |= (1<<6) | (1<<5); // push pull
	PC_CR2 |= (1<<6) | (1<<5); // fast
	
	// LAT pin: normal push-pull output
	_LATPORT_(_DDR) |= (1<<LATpin); // output
	_LATPORT_(_CR1) |= (1<<LATpin); // push pull
	_LATPORT_(_CR2) |= (1<<LATpin); //&= ~(1<<LATpin); // slow
	
	//setup SPI
	SPI_CR1 &= ~(BR_0 | BR_1 | BR_2); // clk/2
	SPI_CR1 &= ~CPOL; //low SCK when idle if 0
	SPI_CR1 &= ~CPHA; //rising edge if 0
	SPI_CR1 &= ~LSB_FIRST; //MSB first
  SPI_CR2 |= SSM | SSI; //enable software control of SS, SS high
	SPI_CR1 |= MSTR; //SPI master
	SPI_CR1 |= SPE; //enable SPI
	
	clearAll();
	sendAll(); // found that this is useful to have a clear start
	
}

#ifdef DM256PWM
void setPoint(uint16_t pixNum, uint16_t pixVal)
{
	#ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
	#endif
	
//	if (ledTable) pixNum = ledTable[pixNum];

	pixel[pixNum] = pixVal;
}

uint16_t getPoint(uint16_t pixNum)
{
	#ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
	#endif
	
//	if (ledTable) pixNum = ledTable[pixNum];

	return pixel[pixNum];
}

void sendAll()
{
	uint16_t k = byteNum;
	uint16_t pixVal;
	uint16_t pixVal1;
	uint16_t pixVal2;
	LAT_low();
	//cli();
	
	if (DMtype==32) do
	{   k--;
		pixVal = pixel[k] DM256PWM; // either square or <<8
		dm_shift(pixVal>>8);
		dm_shift(pixVal & 0xFF);
	} while (k);
	
	if (DMtype==24) do
	{   k--;
		pixVal1 = (pixel[k] DM256PWM)>>4; 
		k--;
		pixVal2 = (pixel[k] DM256PWM)>>4; 
		// shift MSB 8 bits, first value
		dm_shift(pixVal1>>4);
		// make second byte from LSB 4 bits of the first and MSB 4 bits of the second
		pixVal1 = (uint8_t)(pixVal1<<4) | (pixVal2 >> 8);
		dm_shift(pixVal1 & 0xFF);
		// shift LSB 8 bits of the second value
		dm_shift(pixVal2 & 0xFF);
	} while (k); 
	
	while ((SPI_SR) & (BSY)); // finish transmission
	
	LAT_pulse();
}

#else  // end of DM256 stuff

void setPoint(uint16_t pixNum, uint16_t pixVal)
{
   #ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
   #endif
   
 //  if (ledTable) pixNum = ledTable[pixNum];

  if (DMtype==24)  // 12-bit data shift
{
	
	uint16_t place = ((pixNum * 3) >> 1);
	if (pixNum & 1) 
	{ // starts clean
                      // 8 upper bits 
        pixel[place+1] = pixVal >> 4;
                               // 4 lower bits  | keep last 4 bits intact
        pixel[place] = ((uint8_t)(pixVal << 4)) | (pixel[place] & 0x0F);
   }
   else
   { // starts in the middle
                     // keep first 4 bits intact | 4 top bits 
        pixel[place+1] = (pixel[place+1] & 0xF0) | (pixVal >> 8);
		             // 8 lower bits of value
        pixel[place] = pixVal & 0xFF;
    } 
}
   else             // 16-bit data shift
{
	//uint16_t index = (pinNum-1) - pixNum;
	uint16_t place = (pixNum << 1);
	pixel[place+1] = pixVal >> 8;  // 8 top bits
	pixel[place] = pixVal & 0xFF; // 8 lower bits
}
}

uint16_t getPoint(uint16_t pixNum)
{
   #ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
   #endif
   
 //  if (ledTable) pixNum = ledTable[pixNum];
	
  if (DMtype==24)  // 12-bit data 
{ 
	uint16_t place = ((pixNum * 3) >> 1);
	if (pixNum & 1) 
    { // starts clean
                 // 8 upper bits  | 4 lower bits
        return ((pixel[place+1]<<4) |  ((pixel[place] & 0xF0)>>4));
   
    }
    else
	{ // starts in the middle
                      // top 4 bits intact | lower 8 bits 
        return (((pixel[place+1] & 0xF)<<8)  | (pixel[place]));
        
    }
}
   else             // 16-bit data 
{
	uint16_t place = (pixNum << 1);
	return (pixel[place+1]<<8 | pixel[place]);
}
}

void sendAll()
{
	uint16_t k = byteNum;
	
	LAT_low();
		
	do
	{   k--;
	    dm_shift(pixel[k]);
	} while (k);
	
	while (SPI_SR & BSY); // finish transmission
	
	LAT_pulse();
}
#endif

void setRGBpoint(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue)
{
	LED *= 3;
		
	setPoint(LED, red);
	setPoint(LED+1, green);
	setPoint(LED+2, blue);
	
}

void setRGBmax(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue, uint16_t max)
{
	if (max)
	{
		uint32_t valsum = (red + green + blue);
		if (valsum > max)
			{
				red = ((uint32_t)red * max) / valsum;
				green = ((uint32_t)green * max) / valsum;
				blue = ((uint32_t)blue * max) / valsum;
			}
	}
	
	LED *= 3;
	
	setPoint(LED, red);
	setPoint(LED+1, green);
	setPoint(LED+2, blue);
	
}

void setRGBled(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue)
{
	setPoint(LED, red);
	setPoint(LED+1, green);
	setPoint(LED+2, blue);
}


void clearAll()
{
	uint16_t k;
	for (k=0; k<byteNum; k++)
	{pixel[k]=0;}
}

void setGlobalBrightness(uint8_t bri)
{

	//clear GS data to avoid flickering
	uint16_t k;
	//uint8_t count;
	for (k=0; k<pinNum*2; k++) dm_shift(0);
	
	SPI_CR1 &= ~SPE; //disable SPI
	PC_ODR = (1<<5); //SCK high
	
	/*** 4 LAT pulses to turn on GBC input mode */
	LAT_pulse(); LAT_pulse(); LAT_pulse(); LAT_pulse();
	
	PC_ODR &= ~(1<<5); //SCK low
	SPI_CR1 |= SPE; //enable SPI
	
	/*** shift GBC data to the drivers */
	/*** (each DM gets one byte; 7 upper bits control GB data, LSB should be 0 (o/s flag) */
	for (k=0; k<DMnum; k++) {dm_shift(bri<<1);}
	
	while (SPI_SR & BSY); //finish the transmission
	LAT_pulse();
	
	sendAll(); //restore GS data

}

void setGBCbyDriver(uint8_t *bri)
{
	uint16_t k;
	
	//clear GS data to avoid flickering
	for (k=0; k<pinNum*2; k++) dm_shift(0);
	
	SPI_CR1 &= ~SPE; //disable SPI
	PC_ODR = (1<<5); //SCK high
	
	/*** 4 LAT pulses to turn on GBC input mode */
	LAT_pulse(); LAT_pulse(); LAT_pulse(); LAT_pulse();
	
	PC_ODR &= ~(1<<5); //SCK low
	SPI_CR1 |= SPE; //enable SPI
	
	/*** shift GBC data to the drivers */
	/*** (each DM gets one byte; 7 upper bits control GB data, LSB should be 0 (o/s flag) */
	for (k=0; k<DMnum; k++)
	{dm_shift((bri[DMnum-k-1]<<1));}
	
	while (SPI_SR & BSY); //finish the transmission
	LAT_pulse();
	
	sendAll(); //restore GS data

}

uint16_t getChainValue (uint16_t pixNum)
{
	uint8_t numSeg = DMnum / 3;
	uint8_t allStops = numSeg*2 - 1;
	uint16_t DMstep = 8 * 3;
	uint16_t curSeg = pixNum / DMstep;
	
	if (pixNum < (pinNum>>1)) pixNum = pixNum + DMstep * curSeg;
	else pixNum = (allStops - curSeg) * (DMstep<<1) + DMstep + pixNum%DMstep;
	return pixNum;
}

void turnOff()
{
	free (pixel); // free the allocated array memory
}

//void deallocLedTable()
//{
//	free (ledTable); // free the ledtable memory
//}

// ------------ main program below --------
#define DM631 24
#define DM632 32
#define DM633 24
#define DM634 32

#define __PORTA 0x005000
#define __PORTB 0x005005
#define __PORTC 0x00500A
#define __PORTD 0x00500F
#define __PORTE 0x005014
#define __PORTF 0x005019

// below must be set to NOT 0x18 to get 16MHz
#define CLK_CKDIVR *(volatile uint8_t *)0x0050C6

#define TAIL 30 // the length of the 'LED tail'
#define STEP 3// speed of animation; more = faster
#define DMNUMBER 3 // number of DM LED drivers in your chain

int curpos;
int numLeds = 16*DMNUMBER;
uint8_t stepLeds = 255/TAIL;


int main(){
uint16_t count;
uint16_t k;

//set speed to 16MHz
CLK_CKDIVR &= ~0x18;

// set up the object. Change the values according to your setup
DMdriverInit (DM634, DMNUMBER, __PORTD, 2);

setGlobalBrightness(10);

while (1) {

 clearAll();
 count = 0;
 do {
  setPoint((curpos+1)%numLeds, count);
  setPoint(curpos, 255);
  for (k = 1; k<TAIL; k++)
  {
    setPoint((curpos+numLeds-k)%numLeds, stepLeds*(TAIL-k)+(255-count)/TAIL);
  }
  sendAll();
  count+=STEP;
 } while (count<256);

 curpos++;
 if (curpos>(DMNUMBER*16-1)) curpos = 0;
 }
}
