// Pin assignment for camera is hardcode defined

#ifndef NANEYEC_H
#define NANEYEC_H 1
#endif

#include <stdio.h>
#include <stdlib.h>
//#include <ctype.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/console/console.h>
#include <string.h>

// Copied from the Zephyr subsys/usb/console sample. We need this so that we can
// access the zephyr_console instance, and use it to create a "dev" so we can
// read characters from the USB CDC UART. Works very nicely.
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/uart.h>
// #include <zephyr/sys/printk.h>

//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//	     "Console device is not ACM CDC UART device");

// From Zephyr subsys/usb/console sample, gives access to the USB CDC
// device, so we can read characters sent by the host over the USB virtual
// serial port.
//const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

//
// THESE few items allow us to have TWO separate USB virtual serial ports. Taken
// from the USB CDC Composite sample program. We declare TWO ports here, with
// the understanding that the first one is already used for the Zephyr "console"
// (i.e., printk()).
//
// We use the second COM port for sending raw, binary image data.
//
const int SCLK_pin = 8; // Port 0, Pin
const int SDAT_pin = 9;

// SPI CS pin very helpful for debugging with scope, although the camera itself
// does not use it.
const int CS_pin = 4;  // ItsyBitsy pin A5, right next to SPI clock.

// Macros to work with GPIO ports and pins
#define _OUTCLR(pp) if((pp)&0x20) NRF_P1->OUTCLR=1<<(pp&0x1f); else NRF_P0->OUTCLR=1<<(pp&0x1f);

// We will leave SPI clock "idle high", but active on rising edge, so that
// there is always an initial falling edge before the rising edge.

volatile int spi_rxdelay = 2;
void update_spi_rxdelay() {NRF_SPIM4->IFTIMING.RXDELAY = spi_rxdelay;}

void InitSPI()
{
	// // Set clock line as output, low initially.
	// NRF_P0->OUTCLR = 1 << SCLK_pin;
	// NRF_P0->OUTSET = 1 << CS_pin;

	NRF_P0->OUTCLR = 1 << SCLK_pin;
	NRF_P0->OUTSET = 1 << CS_pin;

	// Set clock line as output, HIGH initially.
	//NRF_P0->OUTSET = 1 << SCLK_pin;

	// High drive, to get cleaner edges. The docs are confusing at first, but
	// the chip allows to specify high or standard current drive SEPARATELY for
	// driving a 1 or a 0. This setting drives it with high current for both.
	// NRF_P0->PIN_CNF[SCLK_pin] = 1 | (GPIO_PIN_CNF_DRIVE_H0H1 << 8);
	NRF_P0->PIN_CNF[SCLK_pin] = 1 | (GPIO_PIN_CNF_DRIVE_H0H1 << 8);

	// Make sure data pin is an input, for now. Set to output only when writing.
	//NRF_P0->PIN_CNF[SDAT_pin] = 0;

	// Pull-up on data input pin, so we don't read zeros when they are not there.
	// NRF_P0->PIN_CNF[SDAT_pin] = 0x0C;
	NRF_P0->PIN_CNF[SDAT_pin] = 0x0C;

	NRF_SPIM4->ENABLE = 0;            // Disable
	NRF_SPIM4->CONFIG = 0x00;      // Clock rising edge, Idle LOW
	//NRF_SPIM4->CONFIG = 0x02;      // Clock falling edge, Idle LOW
	//NRF_SPIM4->CONFIG = 0x04;      // Clock falling edge, Idle HIGH
	//NRF_SPIM4->CONFIG = 0x06;      // Clock rising edge, Idle HIGH.

	// Set a small delay for reading, since the camera seems to start
	// transitioning its bits too soon after the rising clock edge.
	NRF_SPIM4->IFTIMING.RXDELAY = 2;

	// NRF_SPIM4->FREQUENCY = 0x10000000;   // For initial testing, just use 1Mbps
	// NRF_SPIM4->FREQUENCY = 0x40000000;   // 4Mbps
	// NRF_SPIM4->FREQUENCY = 0x80000000;   // 8Mbps
	// NRF_SPIM4->FREQUENCY = 0x0A000000;   // 16Mbps only on SPIM4, apparently.
	// CAREFUL, data can be a bit unreliable at this highest speed.
	NRF_SPIM4->FREQUENCY = 0x14000000;   // 32Mbps only on SPIM4, apparently.

	NRF_SPIM4->PSEL.SCK =  SCLK_pin;   // Clock is P0.13
	NRF_SPIM4->PSEL.MOSI =  SDAT_pin;  // MOSI and MISO both use same pin (ItsyBitsy MOSI)
	NRF_SPIM4->PSEL.MISO =  SDAT_pin;  // MOSI and MISO both use same pin (ItsyBitsy MOSI)

	// CS pin helpful for debugging.
	NRF_SPIM4->PSEL.CSN =  CS_pin;

	NRF_SPIM4->ENABLE = 0x07;
}


// Manually toggle the SPI clock line N times. We want to be sure to drive the
// data line to 0, so we drive the data line to 0 as a GPIO.
void spi_GenClocks(int N)
{
	// // Make sure data line stays at zero, so we don't clock in a '1'
	// // inadvertently. Make sure the pin was not left in MOSI mode.
	// NRF_P0->OUTCLR = 1 << SDAT_pin;
	// NRF_P0->PIN_CNF[SDAT_pin] = 1;
	NRF_P0->OUTCLR = 1 << SDAT_pin;
	NRF_P0->PIN_CNF[SDAT_pin] = 1;
	NRF_SPIM4->PSEL.MOSI = 0xffffffff;

	//NRF_P0->OUTSET = 1 << SCLK_pin;  // Clock is "idle high".
	// NRF_P0->OUTCLR = 1 << SCLK_pin;  // Clock is "idle low".
	NRF_P0->OUTCLR = 1 << SCLK_pin;  // Clock is "idle low".

	// Temporarily disable SPI clock function, so we
	// can manually toggle the line.
	NRF_SPIM4->PSEL.SCK = 0xFFFFFFFF;

	for(int i=0; i < N; ++i) {
		// NRF_P0->OUTSET = 1 << SCLK_pin;
		NRF_P0->OUTSET = 1 << SCLK_pin;
		k_busy_wait(1);

		// NRF_P0->OUTCLR = 1 << SCLK_pin;
		NRF_P0->OUTCLR = 1 << SCLK_pin;
		k_busy_wait(1);
		//NRF_P0->OUTSET = 1 << SCLK_pin;
		//k_busy_wait(1);
	}

	NRF_SPIM4->PSEL.SCK =  SCLK_pin;
	//NRF_P0->PIN_CNF[SDAT_pin] = 0;
	// NRF_P0->PIN_CNF[SDAT_pin] = 0x0C; // pull-up on data pin.
	NRF_P0->PIN_CNF[SDAT_pin] = 0x0C; // pull-up on data pin.
}

volatile int dummy_byte = 0;

uint8_t tx_byte, rx_byte;

void spi_WriteByte(uint8_t val)
{
	NRF_SPIM4->PSEL.MOSI =  SDAT_pin;  // No need to set PIN_CNF[] to output - this does it.

	tx_byte = val;
	NRF_SPIM4->TXD.PTR = (uint32_t) &tx_byte;
	NRF_SPIM4->TXD.MAXCNT = 1;

	NRF_SPIM4->RXD.PTR = (uint32_t) &rx_byte;
	NRF_SPIM4->RXD.MAXCNT = 1;

	NRF_SPIM4->EVENTS_ENDTX = 0;
	NRF_SPIM4->TASKS_START = 1;

	while(!NRF_SPIM4->EVENTS_ENDTX)
	    ;

	NRF_SPIM4->PSEL.MOSI = 0xffffffff;     // Disable data output.
}


void spi_WriteBuffer(uint8_t *buf, int N)
{
	NRF_SPIM4->PSEL.MOSI =  SDAT_pin;  // No need to set PIN_CNF[] to output - this does it.

	NRF_SPIM4->TXD.PTR = (uint32_t) buf;
	NRF_SPIM4->TXD.MAXCNT = N;

	NRF_SPIM4->RXD.PTR = (uint32_t) &rx_byte;
	NRF_SPIM4->RXD.MAXCNT = 1;

	NRF_SPIM4->EVENTS_END = 0;
	NRF_SPIM4->TASKS_START = 1;

	while(!NRF_SPIM4->EVENTS_END)
	    ;

	NRF_SPIM4->PSEL.MOSI = 0xffffffff;     // Disable data output.
}

// Optionally pass a sleep value in uSec to sleep while waiting for DMA to
// finish. This will allow other Zephyr threads to run.
void spi_ReadBuffer_sleep(uint8_t *buf, int N, int sleep_usec)
{
	NRF_SPIM4->TXD.PTR = (uint32_t) tx_byte;
	NRF_SPIM4->TXD.MAXCNT = 1;

	NRF_SPIM4->RXD.PTR = (uint32_t) buf;
	NRF_SPIM4->RXD.MAXCNT = N;

	NRF_SPIM4->EVENTS_END = 0;
	NRF_SPIM4->TASKS_START = 1;

	while(!NRF_SPIM4->EVENTS_END) {
		if(sleep_usec) {
			k_usleep(sleep_usec);
		}
	}
}

// Read w/out sleeping.
void spi_ReadBuffer(uint8_t *buf, int N) {spi_ReadBuffer_sleep(buf, N, 0);}


// Try to find the end-of-frame "SYNC", which is 8 all-zero pixel periods, which
// which we should see as 12 all-zero 8-bit bytes.
int Nzeros, Nzeros_inarow, FoundZerosAt, CurByteNumber;
void InitSYNCDetect()
{
	Nzeros = 0;
	Nzeros_inarow = 0;
	FoundZerosAt = 0;
	CurByteNumber = 0;
}

void CheckSync(uint8_t byteval)
{
	++CurByteNumber;

	if(byteval == 0) {
		++Nzeros;
		++Nzeros_inarow;
	} else {
		Nzeros_inarow = 0;
	}
	
	// Only store this ONCE per frame, to avoid confusion.
	if(Nzeros_inarow >= 10 && FoundZerosAt == 0) {
		FoundZerosAt = CurByteNumber;
		Nzeros_inarow = 0;
	}
}


uint8_t spi_ReadByte()
{
	tx_byte = 0;
	NRF_SPIM4->TXD.PTR = (uint32_t) &tx_byte;
	NRF_SPIM4->TXD.MAXCNT = 1;

	NRF_SPIM4->RXD.PTR = (uint32_t) &rx_byte;
	NRF_SPIM4->RXD.MAXCNT = 1;

	NRF_SPIM4->EVENTS_END = 0;
	NRF_SPIM4->TASKS_START = 1;

	while(!NRF_SPIM4->EVENTS_END)
	    ;
	//k_busy_wait(1);

	CheckSync(rx_byte);
	return rx_byte;
}


// A "pixel period" is 12 bits, but our SPI can only send in 8-bit blocks.

// There are only two config registers, each are 16 bits, and must be written
// over two "pixel periods" in a particular format. This struct and union makes
// it simple.

// This is 24 bits total, i.e., two "pixel periods", and we send it via three
// 8-bit SPI words. N.B.!!! The bit fields are LSBit FIRST!!! And the BYTES in
// the union are LSByte first (i.e., bytes[2] is MSB, and contains the "code"
// field).
//
struct __attribute__((packed)) NAN_REG {
	unsigned int zero : 1;
	unsigned int data : 16;
	unsigned int addr : 3;
	unsigned int code : 4;
};

// Hmm, BE CAREFUL, since (as I've read) writing to one member of a union and
// reading from another may result in UNDEFINED BEHAVIOR in C++ (but maybe not
// in C). So if we ever change to C++, BEWARE!!
typedef union {
	struct NAN_REG reg;
	uint8_t bytes[3];
} NAN_REG_union;


// Write one of the config registers. 24 bits, 3 bytes.
uint8_t regbuf[3];
void NanEye_WriteReg(int regnum, int val)
{
	NAN_REG_union U;

	U.reg.code = 0x9;
	U.reg.addr = regnum;
	U.reg.data = val;
	U.reg.zero = 0;

//	for(int i=2; i>=0; --i)
//	    spi_WriteByte(U.bytes[i]);  // write bytes[2], then [1], then [0].

	regbuf[0] = U.bytes[2];
	regbuf[1] = U.bytes[1];
	regbuf[2] = U.bytes[0];
	spi_WriteBuffer(regbuf, 3);
}

// Read and discard, will view on scope initially.
void spi_ReadBytes(int N)
{
	while(N-- > 0) {
		spi_ReadByte();
	}
}


// Keep some globals here for setting the various camera parameters in the
// config registers. These are just the register fields, from top to bottom,
// for config reg 0 and config reg 1.
struct CFGREGS {
	int rowrst, vrst, adcgain, dark, lvds;  // Config reg 0
	int bias, cdsgain, mclk, vref, cvc, highspd;
};

// LVDS should be 3, to give full drive for the camera data line.
// volatile struct CFGREGS camcfg =
//   {128, 2, 0, 2, 3,
//    0, 0, 2, 2, 1, 0};

// #Rows in Reset reduced to 20, to brighten image.
// volatile struct CFGREGS camcfg =
//   {20, 2, 0, 0, 2,
//    0, 0, 1, 1, 1, 0};

// #Rows in Reset set to 0, to eliminate dark band at bottom of image.
// Turned on high-speed clock flag.
volatile struct CFGREGS camcfg =
  {0, 2, 0, 0, 3,
   1, 0, 3, 3, 1, 1};


// This needs to get written for initial startup, and for each frame.
// void NanEye_WriteConfig()
// {
// 	//NRF_P0->OUTSET = 1 << SCOPE_TRIG;

// 	// Write the two config registers. Set for single-ended mode.
// 	NanEye_WriteReg(0, (0x80<<8) | (2<<6) | (0<<4) | (3<<2) | 0);

// 	// Rows Delay=0, 
// 	NanEye_WriteReg(1, (0<<11) | (0<<9) | (0<<8) | (2<<6) | (2<<4) | (1<<2));
// 	k_busy_wait(10);
// }

// This needs to get written for initial startup, and for each frame.
void NanEye_WriteConfig()
{
	// Write the two config registers. Set for single-ended mode.
//	NanEye_WriteReg(0, (0x80<<8) | (2<<6) | (0<<4) | (3<<2) | 0);

	// Rows Delay=0, single-ended output mode.
//	NanEye_WriteReg(1, (0<<11) | (0<<9) | (0<<8) | (2<<6) | (2<<4) | (1<<2));

	// Make sure all values are within range.
	camcfg.rowrst &= 0xff;
	camcfg.vrst &= 0x3;
	camcfg.adcgain &= 0x3;
	camcfg.dark &= 0x3;
	camcfg.lvds &= 0x3;
	
	camcfg.bias &= 0x1;
	camcfg.cdsgain &= 0x1;
	camcfg.mclk &= 0x3;
	camcfg.vref &= 0x3;
	camcfg.cvc &= 0x3;
	camcfg.highspd &= 0x1;

	// Write the two config registers. Set for single-ended mode.
	NanEye_WriteReg(0, (camcfg.rowrst<<8) | (camcfg.vrst<<6) | (camcfg.adcgain<<4)
	 | (camcfg.dark<<2) | camcfg.lvds);

	// Rows Delay=0, single-ended output mode.
	NanEye_WriteReg(1, (0<<11) | (camcfg.bias<<10) | (camcfg.cdsgain<<9) |
	(0<<8) | (camcfg.mclk<<6) | (camcfg.vref<<4) | (camcfg.cvc<<2) | camcfg.highspd);
	k_busy_wait(10);
}


// Enable IDLE mode.
void NanEye_GoIDLE()
{
	// Write the two config registers. Set for single-ended mode.
	NanEye_WriteReg(0, (0x80<<8) | (2<<6) | (0<<4) | (3<<2) | 0);

	// Rows Delay=0, single-ended output mode.
	// ENABLE IDLE MODE!!  (1<<1)
	NanEye_WriteReg(1, (0<<11) | (0<<9) | (0<<8) | (2<<6) | (2<<4) | (1<<2) | (1<<1));
	k_busy_wait(10);
}


//
// Following guidelines from AMS app note AN000611, which details how to
// communicate via a single-ended communication link with the camera.
//
// This should be run just once after the camera has powered up.
void NanEye_InitialConfig()
{
	spi_GenClocks(1);  // Single clock pulse, required.

	// NRF_P0->OUTSET = 1 << SCOPE_TRIG;
	// NRF_P0->OUTSET = 1 << SCOPE_TRIG;

	NanEye_WriteConfig();

	spi_GenClocks(10); // Ignore some bytes.
	spi_GenClocks(12);

	InitSYNCDetect();

	spi_ReadBytes(492); // Initial pre-sync mode
	spi_ReadBytes(984); // SYNC mode
	spi_ReadBytes(984); // DELAY mode
	spi_ReadBytes(320*492); // READOUT first image, discard
	spi_ReadBytes(12);  // end of frame, discard

	printk("Initial config: #zeros %d, FoundZerosAt %d\n",
	    Nzeros, FoundZerosAt);


	// NRF_P0->OUTCLR = 1 << SCOPE_TRIG;
	// NRF_P0->OUTCLR = 1 << SCOPE_TRIG;
}


// void spi_WriteZeros(int N)
// {
// 	while(N-- > 0) {
// 		spi_WriteByte(0);
// 	}
// }

// struct __attribute__((packed)) NAN_PIX {
// 	unsigned int stopbit : 1;
// 	unsigned int data : 10;
// 	unsigned int startbit : 1;
// };

// Hmm, BE CAREFUL, since (as I've read) writing to one member of a union and
// reading from another may result in UNDEFINED BEHAVIOR in C++ (but maybe not
// in C). So if we ever change to C++, BEWARE!!
// typedef union {
// 	struct NAN_PIX pix[2];
// 	uint8_t bytes[3];
// } NAN_2PIX_union;

// Pass a pointer to a 2PIX union.
// void Read2Pix(NAN_2PIX_union *pixpt)
// {
// 	pixpt->bytes[2] = spi_ReadByte();
// 	pixpt->bytes[1] = spi_ReadByte();
// 	pixpt->bytes[0] = spi_ReadByte();
// }

// NAN_2PIX_union pix2[20];

uint8_t camera_rwbuf[2000];

uint8_t ImageBuf[320][492];

// This is called repeatedly, to read video frames.
void NanEye_ReadFrame_OLD()
{
	InitSYNCDetect(); // init SYNC detect counters.

	NanEye_WriteConfig();

	spi_WriteBuffer(camera_rwbuf, 966);
	spi_WriteBuffer(camera_rwbuf, 4*492);

	// NRF_P0->OUTSET = 1 << SCOPE_TRIG;
	// NRF_P0->OUTSET = 1 << SCOPE_TRIG;

	// Read whole lines at once via DMA.
	for(int i=0; i < 320; ++i) {
		spi_ReadBuffer(&ImageBuf[i][0], 492);
	}
	// NRF_P0->OUTCLR = 1 << SCOPE_TRIG;
	// NRF_P0->OUTCLR = 1 << SCOPE_TRIG;

	spi_ReadBytes(12); // End of frame, read and discard.
}

// This is called repeatedly, to read video frames. We have shuffled things
// around from the suggestions in the App Note, so that the SYNC period is first
// (rather than Interface/Write phase) so that we can call this function
// immediately after ReSYNC().
void NanEye_ReadFrame()
{
	int key = irq_lock();

	// NRF_P0->OUTSET = 1 << SCOPE_TRIG;
	// NRF_P0->OUTSET = 1 << SCOPE_TRIG;

	// Read whole lines at once via DMA.
	// for(int i=0; i < 320; ++i) {
	// 	//spi_ReadBuffer(&ImageBuf[i][0], 492);
	// 	spi_ReadBuffer_sleep(&ImageBuf[i][0], 492, 200);
	// }
	for(int i=0; i < 4; ++i) {
		//spi_ReadBuffer(&ImageBuf[i][0], 492);
		spi_ReadBuffer_sleep(&ImageBuf[i*80][0], 492*80, 200);
	}
	// spi_ReadBuffer_sleep(&ImageBuf[0][0], 492*320, 200);
	// NRF_P0->OUTCLR = 1 << SCOPE_TRIG;
	// NRF_P0->OUTCLR = 1 << SCOPE_TRIG;

	irq_unlock(key);

	spi_ReadBytes(12); // End of frame, should be all zeros - read and discard.

	NanEye_WriteConfig();
	spi_WriteBuffer(camera_rwbuf, 966);

	spi_ReadBuffer(camera_rwbuf, 4*492); // SYNC (2 lines) and Delay (we set to 2 lines of delay).
	// printk("%01x, %01x, %01x, %01x, %01x, %01x, %01x, %01x\n", 
	// camera_rwbuf[0],camera_rwbuf[1], camera_rwbuf[2], camera_rwbuf[3], camera_rwbuf[4],camera_rwbuf[5], camera_rwbuf[6], camera_rwbuf[7]);
	// printk("%01x, %01x, %01x, %01x, %01x, %01x, %01x, %01x\n", 
	// camera_rwbuf[8],camera_rwbuf[9], camera_rwbuf[10], camera_rwbuf[11], camera_rwbuf[12],camera_rwbuf[13], camera_rwbuf[14], camera_rwbuf[15]);
}

// Count the longest run of consecutive zeros in the array.
int ConsecutiveZeros(uint8_t *Bytes, int N)
{
	int NCZ=0, NZ=0;

	for(int i=0; i<N; ++i) {
		if(Bytes[i] == 0) ++NZ; else {NCZ=MAX(NCZ,NZ); NZ=0;}
	}

	NCZ = MAX(NCZ,NZ);
	return NCZ;
}

// Count the longest run of consecutive ones in the array.
int ConsecutiveOnes(uint8_t *Bytes, int N)
{
	int NCZ=0, NZ=0;

	for(int i=0; i<N; ++i) {
		if(Bytes[i] == 0xff) ++NZ; else {NCZ=MAX(NCZ,NZ); NZ=0;}
	}

	NCZ = MAX(NCZ,NZ);
	return NCZ;
}

// Get back in sync with camera data stream.
//
// Probably easiest way is to look for the 8 PP of all 0's at the very end of
// the Readout phase.
//
// MUST make sure we enable Pull-Up on the input line, so that we are sure to
// get 0's only for those 8 PP's (otherwise, the data line may "float low"
// during the "Interface/Write" phase)./
//
void ReSYNC_OLD()
{
	// Read 12 bytes at a time to try to find the 12-bytes-worth of 0's. N.B. we
	// MIGHT be shifted by a few bits off, not a whole byte!! MUST correct for
	// that with some extra clock pulses.

	// Read 12 bytes at a time, looking for zeros.
	int i;

	// NRF_P0->OUTSET = 1 << SCOPE_2_TRIG;
	for(i=0; i < 640*492/12; ++i) {
		spi_ReadBuffer(&ImageBuf[0][0], 12);
		//if(ConsecutiveZeros(&ImageBuf[0][0], 12) >= 5) goto FOUND_ZEROS;
		if(ConsecutiveOnes(&ImageBuf[0][0], 12) >= 5) goto FOUND_ZEROS;
	}
	// NRF_P0->OUTCLR = 1 << SCOPE_2_TRIG;
	printk("Could not find zeros...\n");
	return;

FOUND_ZEROS:
	// NRF_P0->OUTCLR = 1 << SCOPE_2_TRIG;
	//printk("Found zeros at count %d\n", i*12);
	printk("Found ONES at count %d\n", i*12);
}

void ReSYNC()
{
	int i;
	const int BUF_SIZE = 100;

	//NRF_P0->OUTSET = 1 << SCOPE_2_TRIG;

	// Look for consecutive zeros.
	for(i=0; i < 640*492 / BUF_SIZE; ++i) {
		spi_ReadBuffer(&ImageBuf[0][0], BUF_SIZE);
		if(ConsecutiveZeros(&ImageBuf[0][0], BUF_SIZE) >= 5) goto FOUND_ZEROS;
	}
	
	// NRF_P0->OUTCLR = 1 << SCOPE_2_TRIG;
	printk("Could not find zeros...\n");
	return;

FOUND_ZEROS:
	// Now look for the 0xFF's due to reading during INTERFACE phase, with GPIO
	// pulled high. They should come RIGHT after the 0's.
	spi_ReadBuffer(&ImageBuf[0][0], BUF_SIZE);
	if(ConsecutiveOnes(&ImageBuf[0][0], BUF_SIZE) < 10) {
		// NRF_P0->OUTCLR = 1 << SCOPE_2_TRIG;
		printk("Could not find ones following the zeros...\n");
		return;
	}
	
	
	// NRF_P0->OUTSET = 1 << SCOPE_2_TRIG;

	//NanEye_GoIDLE(); // Go into IDLE mode.
	NanEye_WriteConfig();

	// Now we need to look for the 0x555 training pulses, but they could be off
	// by some fraction of a byte, so must look carefully. Since the Interface
	// mode lasts for 648 PP, which is 972 bytes, we will loop here and read up
	// to 10 100-byte buffers before giving up.
	int b=0;
	for(i=0; i<10; ++i) {
		spi_ReadBuffer(&ImageBuf[0][0], 100);
		// Now scan, looking for first byte that is not 0xff.
		for(b=0; b<100; ++b) {
			if(ImageBuf[0][b] != 0xff) goto FOUND_SYNC;
		}
	}
	printk("Could not find SYNC/TRAINING pulses.\n");
	return;

FOUND_SYNC:
	// NRF_P0->OUTCLR = 1 << SCOPE_2_TRIG;
	printk("Found sync at %d. Bytes: 0x%02x 0x%02x 0x%02x\n",
	i*100+b, ImageBuf[0][b-1], ImageBuf[0][b], ImageBuf[0][b+1]);

	// There are 8 possibilities for the first non 0xFF byte, and
	// we need to make clock correctsion for 7 of them.
	switch(ImageBuf[0][b]) {
		case 0x55: break;  // Perfect! Do nothing.
		case (uint8_t)(0xff55>>1): spi_GenClocks(1); break;
		case (uint8_t)(0xff55>>2): spi_GenClocks(2); break;
		case (uint8_t)(0xff55>>3): spi_GenClocks(3); break;
		case (uint8_t)(0xff55>>4): spi_GenClocks(4); break;
		case (uint8_t)(0xff55>>5): spi_GenClocks(5); break;
		case (uint8_t)(0xff55>>6): spi_GenClocks(6); break;
		case (uint8_t)(0xff55>>7): spi_GenClocks(7); break;
	}
	
	// Found from inspection on the scope that after all of the other work in
	// this function, we were consistently 4 clock cycles off. It is a bit of a
	// mystery, but with these 4 extra clocks, everything is perfect.

	spi_GenClocks(4);  // Not quite sure why this is necessary...
	// spi_GenClocks(8);  // Not quite sure why this is necessary...


	// The bits should now be aligned correctly. We just need to keep reading
	// the correct number of SYNC bytes. Total number of training bytes is
	// 4*492, so we subtract from that the number of bytes we have seen
	// (including any "corrected" bits).
	int NMoreSync = 4*492;
	NMoreSync -= 100-b;
	spi_ReadBuffer(&ImageBuf[0][0], NMoreSync);

/*******
	//spi_WriteBuffer(writebuf, 966);  // Fill out remainder of Interface mode.
	spi_ReadBuffer(&ImageBuf[0][0], 966);

	NRF_P0->OUTCLR = 1 << SCOPE_2_TRIG;
	spi_ReadBuffer(&ImageBuf[0][0], 100);

	//printk("Found zeros at count %d\n", i*12);
	printk("Found ZEROS at count %d\n", i*BUF_SIZE);
*******/
}

// Check for button press "event" - i.e., just respond TRUE only ONCE per button
// press, then wait for button to be released to reset the mechanism.
int NPress = 0;
int ButtonPress()
{
	static int Press = 0;
	bool ButtonIsDown = (NRF_P0->IN & (1<<29)) == 0;

	if(ButtonIsDown) {
		if(Press == 0) {
			Press = 1;
			++NPress;
			return 1;
		}
	} else {
		// Button was released.
		if(Press == 1) {
			Press = -2;
		} else if(Press < 0) {
			++Press;    // Must wait a few times to be sure button was released.
		}
	}
	return 0;
}

// The pixels are sent as 10-bit data preceeded by a '1' START bit, and followed
// by a '0' STOP bit, for 12 total bits per PP (pixel period).
//
// So, three bytes containing 2 PP's would be (with vertial bars at byte
// boundaries):
//
//   '1' d9 d8 d7 d6 d5 d4 d3 | d2 d1 d0 '0' '1' d9 d8 d7 | d6 d5 d4 d3 d2 d1 d0 '0'
//
//

// For quick debugging, just decode one pixel. The Bytes pointer MUST point to
// and "even numbered" pixel, which means that the byte offset MUST BE A
// MULTIPLE OF THREE (since every three bytes is two pixels), in order to decode
// correctly.
uint16_t DecodeEvenPix(uint8_t *Bytes)
{
	// Top 7 bits from first byte. Bottom 3 bits from second byte
	uint16_t p0 = (uint16_t)(Bytes[0] & 0x7f) << 3u;
	p0 |= (uint16_t)Bytes[1] >> 5u;
	return p0;
}

// Bytes pointer STILL must point to "multiple of three" offset, and we just
// decode the second pixel.
uint16_t DecodeOddPix(uint8_t *Bytes)
{
	// Top 3 bits from second byte. Bottom 7 bits from third byte.
	uint16_t p1 = (uint16_t)(Bytes[1] & 0x07) << 7u;
	p1 |= (uint16_t)Bytes[2] >> 1u;
	return p1;
}


void NanEye_WriteFrame()
{
	for(int r=0; r<320; ++r) {
		printk("%d", r);

		// Decode two pixels (across three bytes) at a time.
		for(int b=0; b<492; b+=3) {
			
			// Top 7 bits from first byte. Bottom 3 bits from second byte
			uint16_t p0 = (uint16_t)(ImageBuf[r][b] & 0x7f) << 3u;
			p0 |= (uint16_t)ImageBuf[r][b+1] >> 5u;

			// Top 3 bits from second byte. Bottom 7 bits from third byte.
			uint16_t p1 = (uint16_t)(ImageBuf[r][b+1] & 0x07) << 7u;
			p1 |= (uint16_t)ImageBuf[r][b+2] >> 1u;

			printk(",%d,%d", p0, p1);
		}
		printk("\n");
	}
}

// struct __attribute__((packed)) Bool_Byte {
// 	bool x0 : 1;
// 	bool x1 : 1;
// 	bool x2 : 1;
// 	bool x3 : 1;
// 	bool x4 : 1;
// 	bool x5 : 1;
// 	bool x6 : 1;
// 	bool x7 : 1;
// };

uint16_t p0, p1;
uint16_t Image_threshold = 64;


////////////////////////////////////////////////
// Unfortunately we don't have enough RAM
// Consider downsample

// struct Bool_Byte Image_binary[320][41];
// void Thresholding_ImageBuf() {
// 	for(int r=0; r<320; ++r) {
// 		// Compare 8 pixels (across 12 bytes) to threshold (8*1 bit) and save in 1 Bool_Byte (1 byte).
// 		for(int b=0; b<41; ++b) {
// 			p0 = (uint16_t)(ImageBuf[r][12*b] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][12*b+1] >> 5u;
// 			p1 = (uint16_t)(ImageBuf[r][12*b+1] & 0x07) << 7u;
// 			p1 |= (uint16_t)ImageBuf[r][12*b+2] >> 1u;
// 			Image_binary[r][b].x0 = p0 >= Image_threshold;
// 			Image_binary[r][b].x1 = p1 >= Image_threshold;

// 			p0 = (uint16_t)(ImageBuf[r][12*b+3] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][12*b+4] >> 5u;
// 			p1 = (uint16_t)(ImageBuf[r][12*b+4] & 0x07) << 7u;
// 			p1 |= (uint16_t)ImageBuf[r][12*b+5] >> 1u;
// 			Image_binary[r][b].x2 = p0 >= Image_threshold;
// 			Image_binary[r][b].x3 = p1 >= Image_threshold;

// 			p0 = (uint16_t)(ImageBuf[r][12*b+6] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][12*b+7] >> 5u;
// 			p1 = (uint16_t)(ImageBuf[r][12*b+7] & 0x07) << 7u;
// 			p1 |= (uint16_t)ImageBuf[r][12*b+8] >> 1u;
// 			Image_binary[r][b].x4 = p0 >= Image_threshold;
// 			Image_binary[r][b].x5 = p1 >= Image_threshold;

// 			p0 = (uint16_t)(ImageBuf[r][12*b+9] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][12*b+10] >> 5u;
// 			p1 = (uint16_t)(ImageBuf[r][12*b+10] & 0x07) << 7u;
// 			p1 |= (uint16_t)ImageBuf[r][12*b+11] >> 1u;
// 			Image_binary[r][b].x6 = p0 >= Image_threshold;
// 			Image_binary[r][b].x7 = p1 >= Image_threshold;
// 		}
// 	}
// }

// void Thresholding_ImageBuf() {
// 	for(int r=0; r<320; r+=2) {
// 		// Compare 8 pixels (across 12 bytes) to threshold (8*1 bit) and save in 1 Bool_Byte (1 byte).
// 		for(int b=0; b<20; ++b) {
// 			p0 = (uint16_t)(ImageBuf[r][24*b] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+1] >> 5u;
// 			Image_binary[r][b] = p0 >= Image_threshold;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+3] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+4] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 1;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+6] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+7] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 2;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+9] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+10] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 3;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+12] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+13] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 4;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+15] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+16] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 5;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+18] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+19] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 6;

// 			p0 = (uint16_t)(ImageBuf[r][24*b+21] & 0x7f) << 3u;
// 			p0 |= (uint16_t)ImageBuf[r][24*b+22] >> 5u;
// 			Image_binary[r][b] |= (p0 >= Image_threshold) << 7;
// 		}
// 	}
// }

#define frame_block_size (492*16)
uint8_t Image_binary[frame_block_size];


void ROI_ImageBuf() {
	for(int r=0; r<16; r++) {
		for(int b=0; b<492; ++b) {
			Image_binary[r*492+b] = ImageBuf[r][b];
		}
	}
}

///////////////////////////////// frame_block_circular_buffer


#define frame_block_circular_buf_num 10
#define frame_block_circular_buf_size (frame_block_circular_buf_num*frame_block_size)

typedef struct {
    char *buf; // actuall capacity = N-1
    int write_idx; // frame idx to write into circular buf
    int read_idx; // frame idx to read out from circular buf
} frame_block_circular_buf;

bool frame_block_is_empty(frame_block_circular_buf *buf) {
    if (buf->read_idx == buf->write_idx) {
        return true;
    }
    else {
        return false;
    }
}

bool frame_block_is_full(frame_block_circular_buf *buf) {
    if ((buf->write_idx + 1) % frame_block_circular_buf_num == buf->read_idx) {
        return true;
    }
    else {
        return false;
    }
}

int frame_block_buf_length(frame_block_circular_buf *buf) {
    // NRF_GPIOTE->INTENCLR |= 0x00000080;
    int temp = (buf->write_idx - buf->read_idx + frame_block_circular_buf_num) % frame_block_circular_buf_num;
    // NRF_GPIOTE->INTENSET |= 0x00000080;
    return temp;
}

int frame_block_write_in_buf(frame_block_circular_buf *buf, uint8_t *value) {
	// for writing one Image_binary
    if (1 + frame_block_buf_length(buf) > frame_block_circular_buf_num-1) {
        printk("circular_buf out of range, buf_length=%d \n", frame_block_buf_length(buf));
        return -1;
    }
    
	for (int i = 0;i < frame_block_size;i++) {
        buf->buf[buf->write_idx*frame_block_size+i] = value[i];
    }
	buf->write_idx = (buf->write_idx + 1) % frame_block_circular_buf_num;
    return 0;
}

int frame_block_read_out_buf(frame_block_circular_buf *buf, uint8_t *value, uint16_t *buf_l) {
	// for reading out multiple Image_binary
    uint16_t L = (uint16_t)frame_block_buf_length(buf);
	buf_l[0] = L;
    for (int i=0;i<L;i++) {
		for (int j = 0;j < frame_block_size;j++) {
        	value[i*frame_block_size+j] = buf->buf[(buf->read_idx)*frame_block_size+j];
    	}
        buf->read_idx = (buf->read_idx + 1) % frame_block_circular_buf_num;
    }
    return 0;
}