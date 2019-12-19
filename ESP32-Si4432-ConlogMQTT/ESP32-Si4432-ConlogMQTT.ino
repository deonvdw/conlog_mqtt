#include <SPI.h>
#include <WiFi.h>
// Must override MQTT_MAX_PACKET_SIZE in the include file below, can't override it here.
#include <PubSubClient.h>

#define WIFI_SSID "xxx"
#define WIFI_PASSWORD "xxx"
#define MQTT_SERVER "x.x.x.x"

// Radio config
#define MAX_RX_LEN  64               // Max packet length is 128 DECODED bytes
#define RX_FIFO_THRESHOLD   4        // To match FEC block size
unsigned char rxbuffer[MAX_RX_LEN];  // Saved copy of fully decoded message
int rxlength= 0;

int rxdone= 0;
int rxfail= 0;
int RSSI;

// WIFI and MQTT client globals
WiFiClient    wificlient;
PubSubClient  psclient(wificlient);

// SPI
static const int spiClk = 1000000; // 1 MHz
SPIClass * vspi = NULL;   //uninitalised pointers to SPI objects

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// Based on Texas intruments Design Note DN507

// Look-up source state index.
// 1st array index is Destination state, 2nd array index is- Each of two possible source states
const unsigned char aTrellisSourceStateLut[8][2]=
{
 {0, 4}, // State {0,4} -> State 0
 {0, 4}, // State {0,4} -> State 1
 {1, 5}, // State {1,5} -> State 2
 {1, 5}, // State {1,5} -> State 3
 {2, 6}, // State {2,6} -> State 4
 {2, 6}, // State {2,6} -> State 5
 {3, 7}, // State {3,7} -> State 6
 {3, 7}, // State {3,7} -> State 7
};

// Look-up expected output given:
// Destination state (1st array index), Each of two possible source states (2nd array index)
const unsigned char aTrellisTransitionOutput[8][2]=
{
 {0, 3}, // State {0,4} -> State 0 produces {"00", "11"}
 {3, 0}, // State {0,4} -> State 1 produces {"11", "00"}
 {1, 2}, // State {1,5} -> State 2 produces {"01", "10"}
 {2, 1}, // State {1,5} -> State 3 produces {"10", "01"}
 {3, 0}, // State {2,6} -> State 4 produces {"11", "00"}
 {0, 3}, // State {2,6} -> State 5 produces {"00", "11"}
 {2, 1}, // State {3,7} -> State 6 produces {"10", "01"}
 {1, 2}, // State {3,7} -> State 7 produces {"01", "10"}
};

// Look-up input bit at encoder for given Destination State
const unsigned char aTrellisTransitionInput[8]=
{
 0, 1, 0, 1, 0, 1, 0, 1,
};

// Calculate the Hamming weight of byte - returns number of bits set
unsigned char hammWeight(unsigned char a)
{
 a= ((a & 0xAA) >> 1) + (a & 0x55);
 a= ((a & 0xCC) >> 2) + (a & 0x33);
 a= ((a & 0xF0) >> 4) + (a & 0x0F);
 return a;
}

// Returns minimum value from two parameters
unsigned char min(unsigned char a, unsigned char b)
{
 return (a <= b ? a : b);
}

// De-interleaves and decodes a given input buffer. Params:
//
// pDecData - Pointer to where to put decoded data (NULL when initializing at start of packet)
// pInData - Pointer to received data
// nRemBytes - of remaining (decoded) bytes to decode
//
// Returns the number of bytes of decoded data stored at pDecData
int fecDecode(unsigned char *pDecData, unsigned char* pInData, int nRemBytes)
{
 // Two sets of buffers (last, current) for each destination state for holding:
 static unsigned char nCost[2][8]; // Accumulated path cost
 static unsigned long aPath[2][8]; // Encoder input data (32b window)

 // Indices of (last, current) buffer for each iteration
 static unsigned char iLastBuf;
 static unsigned char iCurrBuf;

 // Number of bits in each path buffer
 static unsigned char nPathBits;

 // Variables for de-interleaving data
 unsigned char aDeintData[4];
 signed char iOut;
 signed char iIn;

 // Variables used to hold # Viterbi iterations to run, # bytes output,
 // minimum cost for any destination state, bit index of input symbol
 unsigned char nIterations;
 int nOutputBytes= 0;
 unsigned char nMinCost;
 signed char BitIndex= 8 - 2;

 // Initialize variables at start of packet (and return without doing any more)
 if (pDecData==NULL)
 {
  unsigned char n;
  memset (nCost,0,sizeof(nCost));
  for (n=1;n<8;n++)
   nCost[0][n]= 100;
  iLastBuf= 0;
  iCurrBuf= 1;
  nPathBits= 0;
  return 0;
 }

 // De-interleave received data (and change pInData to point to de-interleaved data)
 for (iOut = 0; iOut < 4; iOut++)
 {
  unsigned char dataByte= 0;
  for (iIn=3; iIn >= 0; iIn--)
   dataByte= (dataByte<<2) | ((pInData[iIn]>>(2*iOut)) & 0x03);
  aDeintData[iOut]= dataByte;
 }
 pInData= aDeintData;


 // Process up to 4 bytes of de-interleaved input data, processing one encoder symbol (2b) at a time
 for (nIterations = 16; nIterations > 0; nIterations--)
 {
  unsigned char iDestState;
  unsigned char symbol = ((*pInData)>>BitIndex) & 0x03;
 
  // Find minimum cost so that we can normalize costs (only last iteration used)
  nMinCost = 0xFF;
 
  // Get 2b input symbol (MSB first) and do one iteration of Viterbi decoding
  if ((BitIndex -= 2) < 0)
  {
   BitIndex= 6;
   pInData++; // Update pointer to the next byte of received data
  }
 
  // For each destination state in the trellis, calculate hamming costs for both possible paths into state and
  // select the one with lowest cost.
  for (iDestState = 0; iDestState < 8; iDestState++)
  {
   unsigned char nCost0;
   unsigned char nCost1;
   unsigned char iSrcState0;
   unsigned char iSrcState1;
   unsigned char nInputBit;
  
   nInputBit= aTrellisTransitionInput[iDestState];
  
   // Calculate cost of transition from each of the two source states (cost is Hamming difference between
   // received 2b symbol and expected symbol for transition)
   iSrcState0= aTrellisSourceStateLut[iDestState][0];
   nCost0= nCost[iLastBuf][iSrcState0];
   nCost0+= hammWeight(symbol ^ aTrellisTransitionOutput[iDestState][0]);
  
   iSrcState1= aTrellisSourceStateLut[iDestState][1];
   nCost1= nCost[iLastBuf][iSrcState1];
   nCost1+= hammWeight(symbol ^ aTrellisTransitionOutput[iDestState][1]); 
   
   // Select transition that gives lowest cost in destination state, copy that source state's path and add
   // new decoded bit
   if (nCost0 <= nCost1)
   {
    nCost[iCurrBuf][iDestState]= nCost0;
    nMinCost= min(nMinCost, nCost0);
    aPath[iCurrBuf][iDestState]= (aPath[iLastBuf][iSrcState0] << 1) | nInputBit;
   }
   else
   {
    nCost[iCurrBuf][iDestState]= nCost1;
    nMinCost= min(nMinCost, nCost1);
    aPath[iCurrBuf][iDestState]= (aPath[iLastBuf][iSrcState1] << 1) | nInputBit;
   }
  }
  nPathBits++;
  // If trellis history is sufficiently long, output a byte of decoded data
  if (nPathBits==32)
  {
   *pDecData++= (aPath[iCurrBuf][0] >> 24) & 0xFF;
   nOutputBytes++;
   nPathBits -= 8;
   nRemBytes--;
  }
 
  // After having processed 3-symbol trellis terminator, flush out remaining data
  if ((nRemBytes <= 3) && (nPathBits == ((8 * nRemBytes) + 3)))
  {
   while (nPathBits >= 8)
   {
    *pDecData++ = (aPath[iCurrBuf][0] >> (nPathBits - 8)) & 0xFF;
    nOutputBytes++;
    nPathBits-= 8;
   }
   return nOutputBytes;
  }
  // Swap current and last buffers for next iteration
  iLastBuf= (iLastBuf+1) % 2;
  iCurrBuf= (iCurrBuf+1) % 2;
 }

 // Normalize costs so that minimum cost becomes 0
 {
  unsigned char iState;
  for (iState=0; iState < 8; iState++)
   nCost[iLastBuf][iState]-= nMinCost;
 }
 return nOutputBytes;
}

////////////////////////////////////////////////////////////////////////////////////////////////

static const unsigned char pn9table[] =
{
  0xff, 0xe1, 0x1d, 0x9a, 0xed, 0x85, 0x33, 0x24, 0xea, 0x7a, 0xd2, 0x39, 0x70, 0x97, 0x57, 0x0a,
  0x54, 0x7d, 0x2d, 0xd8, 0x6d, 0x0d, 0xba, 0x8f, 0x67, 0x59, 0xc7, 0xa2, 0xbf, 0x34, 0xca, 0x18,
  0x30, 0x53, 0x93, 0xdf, 0x92, 0xec, 0xa7, 0x15, 0x8a, 0xdc, 0xf4, 0x86, 0x55, 0x4e, 0x18, 0x21,
  0x40, 0xc4, 0xc4, 0xd5, 0xc6, 0x91, 0x8a, 0xcd, 0xe7, 0xd1, 0x4e, 0x09, 0x32, 0x17, 0xdf, 0x83,
  0xff, 0xf0, 0x0e, 0xcd, 0xf6, 0xc2, 0x19, 0x12, 0x75, 0x3d, 0xe9, 0x1c, 0xb8, 0xcb, 0x2b, 0x05,
  0xaa, 0xbe, 0x16, 0xec, 0xb6, 0x06, 0xdd, 0xc7, 0xb3, 0xac, 0x63, 0xd1, 0x5f, 0x1a, 0x65, 0x0c,
  0x98, 0xa9, 0xc9, 0x6f, 0x49, 0xf6, 0xd3, 0x0a, 0x45, 0x6e, 0x7a, 0xc3, 0x2a, 0x27, 0x8c, 0x10,
  0x20, 0x62, 0xe2, 0x6a, 0xe3, 0x48, 0xc5, 0xe6, 0xf3, 0x68, 0xa7, 0x04, 0x99, 0x8b, 0xef, 0xc1,
  0x7f, 0x78, 0x87, 0x66, 0x7b, 0xe1, 0x0c, 0x89, 0xba, 0x9e, 0x74, 0x0e, 0xdc, 0xe5, 0x95, 0x02,
  0x55, 0x5f, 0x0b, 0x76, 0x5b, 0x83, 0xee, 0xe3, 0x59, 0xd6, 0xb1, 0xe8, 0x2f, 0x8d, 0x32, 0x06,
  0xcc, 0xd4, 0xe4, 0xb7, 0x24, 0xfb, 0x69, 0x85, 0x22, 0x37, 0xbd, 0x61, 0x95, 0x13, 0x46, 0x08,
  0x10, 0x31, 0x71, 0xb5, 0x71, 0xa4, 0x62, 0xf3, 0x79, 0xb4, 0x53, 0x82, 0xcc, 0xc5, 0xf7, 0xe0,
  0x3f, 0xbc, 0x43, 0xb3, 0xbd, 0x70, 0x86, 0x44, 0x5d, 0x4f, 0x3a, 0x07, 0xee, 0xf2, 0x4a, 0x81,
  0xaa, 0xaf, 0x05, 0xbb, 0xad, 0x41, 0xf7, 0xf1, 0x2c, 0xeb, 0x58, 0xf4, 0x97, 0x46, 0x19, 0x03,
  0x66, 0x6a, 0xf2, 0x5b, 0x92, 0xfd, 0xb4, 0x42, 0x91, 0x9b, 0xde, 0xb0, 0xca, 0x09, 0x23, 0x04,
  0x88, 0x98, 0xb8, 0xda, 0x38, 0x52, 0xb1, 0xf9, 0x3c, 0xda, 0x29, 0x41, 0xe6, 0xe2, 0x7b, 0xf0
};

int pn9index= 0;

unsigned char pn9code_byte (unsigned char input)
{
 if (pn9index>=sizeof(pn9table))
  pn9index= 0;
 return input ^ pn9table[pn9index++];
}

////////////////////////////////////////////////////////////////////////////////////////////////

// Calculate checksum. Updates running checksum with new message byte
// Initialise checksum to 0xFFFF before message, final checksum should be 0000 for complete message
unsigned short calcCRC(unsigned char crcData, unsigned short crcReg)
{
 unsigned char i;
 for (i = 0; i < 8; i++)
 {
  if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
   crcReg = (crcReg << 1) ^ 0x8005;
  else
   crcReg = (crcReg << 1);
  crcData <<= 1;
 }
 return crcReg;
}

////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char   decodebuf[MAX_RX_LEN];
int             decodelen;
unsigned short  checksum;

// Init streaming decode process
void init_decode (void)
{
 pn9index= 0;
 fecDecode(NULL, NULL, 0); // The function needs to be called with a NULL pointer for initialization before every packet to decode
 decodelen= 0;
 checksum = 0xFFFF; // Init value for CRC calculation
}

// FEC decode 4 received bytes and add to decoded output. return 1 when decoded stream passes CRC check.
int stream_decode (unsigned char *coded)
{
 // Always say we still have 255 bytes left to decode - force decoder to continuously decode more bytes
 int bytes_out= fecDecode(decodebuf+decodelen,coded,255);
 while ((bytes_out--)>0)
 {
  decodebuf[decodelen]= pn9code_byte(decodebuf[decodelen]);
  checksum= calcCRC(decodebuf[decodelen],checksum);
  decodelen++;
  if (!checksum) return 1;
  if (decodelen==MAX_RX_LEN) return 0;
 }
 return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////

// Based on TI Design Note DN504

int fecEncodeTable[]=
{
 0, 3, 1, 2,
 3, 0, 2, 1,
 3, 0, 2, 1,
 0, 3, 1, 2
};

void fec_encode (unsigned char *input, int inlen, unsigned char *encoded, int *enclen)
{
 unsigned char  workbuf[256];
 int            sendlen;
 int            count;
 unsigned short crcreg;
 int            i;
 int            j;
 int            fecReg;
 int            fecOutput;
 int            fecNum;
 unsigned int   intOutput;
 unsigned char  fec[520];

 crcreg= 0xFFFF; //Init value for CRC calculation
 // Calculate CRC and whiten TX data
 for (count=0;count<inlen;count++)
 {
  crcreg= calcCRC(input[count],crcreg);
  workbuf[count]= input[count] ^ pn9table[count];
 }
 workbuf[inlen]= (crcreg >> 8) ^ pn9table[inlen];         // CRC1 xor whitening value
 workbuf[inlen+1]= (crcreg & 0x00FF) ^ pn9table[inlen+1]; // CRC0 xor whitening value
 inlen+= 2;

 // Append Trellis Terminator
 workbuf[inlen]= 0x0B;
 workbuf[inlen+1]= 0x0B;
 
 // number of bytes input bytes into fec encoding
 fecNum = 2*((inlen/2)+1);
 
 // FEC encode
 fecReg= 0;
 for (i=0;i<fecNum;i++)
 {
  fecReg= (fecReg & 0x700) | (workbuf[i] & 0xFF);
  fecOutput= 0;
  for (j=0;j<8;j++)
  {
   fecOutput = (fecOutput << 2) | fecEncodeTable[fecReg >> 7];
   fecReg = (fecReg << 1) & 0x7FF;
  }
  fec[i*2]= (fecOutput>>8)&0xFF;
  fec[i*2+1]= fecOutput&0xFF;
 }
 
 // Perform interleaving
 for (i=0;i<fecNum*2;i+=4)
 {
  intOutput = 0;
  for (j = 0; j < 4*4; j++)
   intOutput= (intOutput << 2) | ((fec[i +(~j & 0x03)] >> (2 * ((j & 0x0C) >> 2))) & 0x03);
   
  encoded[i] = (intOutput >> 24) & 0xFF;
  encoded[i + 1] = (intOutput >> 16) & 0xFF;
  encoded[i + 2] = (intOutput >> 8) & 0xFF;
  encoded[i + 3] = (intOutput >> 0) & 0xFF;
 }
 *enclen= fecNum*2;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

/**/ int txdone=0;

void IRAM_ATTR ISR()
{
 //read interrupt status registers
 int ItStatus1= spi_read(0x03); //read the Interrupt Status1 register
 int ItStatus2= spi_read(0x04); //read the Interrupt Status2 register

 /* preamble detected */
 if (ItStatus2&0x40)
  init_decode();

 /* sync word detected */
 if (ItStatus2&0x80)
  RSSI= spi_read(0x26); // RSSI register

 /* rx fifo almost full interrupt occurred */
 if (ItStatus1&0x10)
 {
  unsigned char rxrawbuf[4];
  spi_burst_read (0x7F,rxrawbuf,4,0xFF);
  if (stream_decode(rxrawbuf))
  {
   // successfull decode -- make copy of packet, stop radio and prepare to receive a new packet
   rxlength= decodelen;
   memcpy (rxbuffer,decodebuf,decodelen);
   InitRX();
   rxdone= 1;
  }
  if (decodelen==MAX_RX_LEN)
  {
   InitRX();
   rxfail= 1;
  }
 }

/* packet sent interrupt occurred - clear out possibly stale RX FIFO, restart RX mode */
 if (ItStatus1&0x04)
 {
  InitRX();
/**/  txdone = 1;
 }
}

void SendPacket (unsigned char *data, int length)
{
 unsigned char  sendbuf[256];
 int            sendlen;
 int            count;

 // Calculate CRC, whiten data and FEC encode data
 fec_encode (data,length,sendbuf,&sendlen);

/**/ { const char hexdigit[]= "0123456789ABCDEF"; char msgbuffer[512]; char *p; int count; p= msgbuffer; sprintf (p,"Send data len %d:",sendlen); p+= strlen(p); for (count=0;count<sendlen;count++) { *(p++)= ' '; *(p++)= hexdigit[(sendbuf[count])>>4]; *(p++)= hexdigit[(sendbuf[count])&0x0F]; } *p= 0; printf ("FEC encoded message to send: %s\n",msgbuffer); }

 // We won't do any checks to see if we are currently busy receiving any packets .. chance for data loss
 spi_write(0x07, 0x01);   // Operating Function Control 1 register: Xtal on and turn RX off

 // if we don;t delay here the message is not properly transmitted, power still ramping up.
 delay(75);            // Chip seems to need a slight delay switching from RX mode to TX mode
 
 // reset the RX and TX FIFO, load data into TX FIFO 
 spi_write(0x08, 0x03);                        // Operating Function Control 2 register: clear RX+TX FIFO on
 spi_write(0x08, 0x00);                        // Operating Function Control 2 register: clear RX+TX FIFO off
 spi_write(0x3E,sendlen&0x00FF);               // Set our TX packet length
 spi_burst_write (0x7F,sendbuf,sendlen,0xFF);  // And load data

 // Turn on TX mode and let the packet fly
 spi_write(0x07, 0x09); // Operating Function Control 1 register: Xtal on and turn TX mode on - packet will be sent
}


//// Diagnose trx switching - set the GPIO's according to the RF switch */
//  SpiWriteRegister(0x0C, 0x12);                              //write 0x12 to the GPIO1 Configuration(set the TX state)
//  SpiWriteRegister(0x0D, 0x15);                             //write 0x15 to the GPIO2 Configuration(set the RX state) 

//  set  cap. bank
//  SpiWriteRegister(0x09, 0xD7);                             //write 0xD7 to the Crystal Oscillator Load Capacitance register

// General setup for Si4432 chip operating at 434.195MHz, GFSK, 38.4kbps, 75khz deviation
// Actual frequency value tuned here is slightly different - inaccurate XTAL on board?
void SetupSi4432 (void)
{
 // Values based on EZRadioPro calculator spreadsheet - 38.4kbps, 75khz deviation
 spi_write (0x1C,0x01);   // IF Filter Bandwidth
 spi_write (0x1D,0x3C);   // AFC Loop Gearshift Override
 spi_write (0x1E,0x02);   // AFC Timing Control
 spi_write (0x1F,0x03);   // Clock Recovery Gearshift Override
 spi_write (0x20,0x68);   // Clock Recovery Oversampling Rate
 spi_write (0x21,0x01);   // Clock Recovery Offset 2
 spi_write (0x22,0x3A);   // Clock Recovery Offset 1
 spi_write (0x23,0x93);   // Clock Recovery Offset 0
 spi_write (0x24,0x05);   // Clock Recovery Timing Loop Gain 1
 spi_write (0x25,0xEA);   // Clock Recovery Timing Loop Gain 0
 spi_write (0x2A,0xFF);   // AFC Limiter
 
 //spi_write (0x30,0xA8);   // Data Access Control: Enable RX+TX packet handlers, disable CRC check.
 spi_write (0x30,0x28);   // Data Access Control: Disable RX packet handler, enable TX packet handler, disable CRC check.
 spi_write (0x33,0x0E);   // Header Control 2: No RX/TX header, Fixed length, Sync words D3-D0
 spi_write (0x34,0x08);   // Preamble Length 8 nibbles = 4 bytes (was 6)
 spi_write (0x35,0x2A);   // Preamble Detection Control 1: must detect 5 nibbles
 spi_write (0x36,0x2C);   // Synchronization Word 3: NOT 'D3'
 spi_write (0x37,0x6E);   // Synchronization Word 2: NOT '91'
 spi_write (0x38,0x2C);   // Synchronization Word 1: NOT 'D3'
 spi_write (0x39,0x6E);   // Synchronization Word 0: NOT '91'

 // Values based on EZRadioPro calculator spreadsheet 
 spi_write (0x58,0x80);   // Undocumented, set according to calculator
 spi_write (0x69,0x60);   // AGC Override 1: Enable AGC, max 25dB
 spi_write (0x6D,0x1E);   // TX Power: 1C = mid point. Set 0x1F for MAX :)
 spi_write (0x6E,0x09);   // TX Data Rate 1
 spi_write (0x6F,0xD5);   // TX Data Rate 0
 spi_write (0x70,0x04);   // Modulation Mode Control 1
 spi_write (0x71,0x2B);   // Modulation Mode Control 2: FIFO source, Invert data to mimick CC1101 preamble, GFSK
 spi_write (0x72,0x1A);   // Frequency Deviation
 spi_write (0x75,0x53);   // Frequency Band Select
 spi_write (0x76,0x69);   // Nominal Carrier Frequency
 spi_write (0x77,0x80);   // Nominal Carrier Frequency

 spi_write (0x7E,RX_FIFO_THRESHOLD);   // RX FIFO Control
 // Enable the rx packet complete, tx packet oomplete, preamble dectected and rx FIFO threshold interrupts:
 // Also add sync detection interrupt for RSSI measurement
 spi_write (0x05, 0x16); // Interrupt Enable 1 register: RX FIFO Almost Full, Valid Packet Received PLUS TX complete interrupt
 spi_write (0x06, 0xC0); // Interrupt Enable 2 register: Valid Preamble Detected, Sync Word Detected
}

// Set Si4432 in RX mode
void InitRX (void)
{
 // disable the receiver chain
 spi_write(0x07, 0x01); // Operating Function Control 1 register: Xtal on (RX off)
 // reset the TX and RX FIFOs
 spi_write(0x08, 0x03); // Operating Function Control 2 register: clear TX and RX FIFO on
 spi_write(0x08, 0x00); // Operating Function Control 2 register: clear TX and RX FIFO off
 
 // enable the receiver chain again
 spi_write (0x07, 0x05); // Operating Function Control 1 register: Set RX on and Xtal on
}

void ReadChipInfo()
{
 //use it as you would the regular arduino SPI API
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 vspi->transfer(0x00 || 0x00);   // High bit low = READ, register= 0
 int reg0= vspi->transfer(0x00); // 0x00 = dummy value
 int reg1= vspi->transfer(0x00); // 0x00 = dummy value
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();

 printf ("Chip info: Reg0= %02X, Reg1= %02X\n",reg0,reg1);
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char spi_read (unsigned char reg)
{
 unsigned char temp;
 
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 vspi->transfer(0x7F&reg);   // High bit low = read
 temp= vspi->transfer(0);    // Dummy value
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return temp;
}

void spi_burst_read (unsigned char reg, unsigned char *buffer, int count, unsigned char xorval)
{
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 vspi->transfer(0x7F&reg);   // High bit low = read
 while ((count--)>0)
  *(buffer++) = vspi->transfer(0) ^ xorval;
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
}

void spi_write (unsigned char reg, unsigned char val)
{
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 vspi->transfer(0x80|reg);   // High bit high = write
 vspi->transfer(val);   // High bit high = write
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
}

void spi_burst_write (unsigned char reg, unsigned char *buffer, int count, unsigned char xorval)
{
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 vspi->transfer(0x80|reg);   // High bit high = write
 while ((count--)>0)
  vspi->transfer(*(buffer++) ^ xorval);
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
}


////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

static const char hexdigit[]= "0123456789ABCDEF";

int gethexvalue(char ch)
{
 return ((ch>='0')&&(ch<='9'))?ch-'0':((ch>='A')&&(ch<='F'))?ch-'A'+10:((ch>='a')&&(ch<='f'))?ch-'a'+10:-1;
}

void hextobin (char *hexstr, int length, unsigned char *outbuf, int bufsize, int* outlen)
{
 int            digitval;
 unsigned char  value;
 unsigned char  nibble= 0;

 *outlen= 0;
 while ((length--)>0)
 {
  digitval= gethexvalue(*(hexstr++));
  if (digitval<0)
   continue;
   
  nibble^= 1;
  if (nibble)
  {
   value= digitval;
   continue;
  }
  value= value<<4 | digitval;
  if ((--bufsize)<0)
   break;
  *(outbuf++)= value;
  (*outlen)++;
 }
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void pscallback (char* topic, byte* message, unsigned int length)
{
 unsigned char  binmsg[256];
 int            binlen;
/**/ int now;
   
 printf ("Message arrived on topic: %s.\n",topic);

 // We ignore the topic, since we are only subscriber to a single topic
 hextobin ((char*)message,length,binmsg,sizeof(binmsg),&binlen);

/**/ now= millis();
  SendPacket (binmsg,binlen);
/**/ if ((millis()-now)>250) printf ("=== SendPacket took %d seconds ===\n");
}

void setup()
{
 // Configure WIFI
 WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
 while (WiFi.status() != WL_CONNECTED)
 {
  delay(500);
  printf(".");
 }

 printf("\nWiFi connected\n");
 printf("IP address: %s\n",WiFi.localIP().toString().c_str());
 
 psclient.setServer(MQTT_SERVER,1883);
 psclient.setCallback(pscallback);

 // Reset is done before setting up SPI or interrupt - else we get ESP32 panic/reboots
 pinMode(21, OUTPUT);     // Si4432 shutdown pin
 pinMode(4, INPUT_PULLUP); //Set interrupt pin to input
 digitalWrite(21,HIGH);   // Put Si4432 in shutdown mode
 delay(50);
 digitalWrite(21,LOW);    // Release Si4432 from shutdown mode
 delay(25);               // App note says delay min 15ms after taking exiting shutdown

 // initialise vspi with default pins
 // SCLK = 18, MISO = 19, MOSI = 23, SS = 5
 vspi= new SPIClass(VSPI);
 vspi->begin();
 //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
 pinMode(5, OUTPUT);      // VSPI SS
 
 // read interrupt status registers to clear the interrupt flags and release NIRQ pin - HW reset
 spi_read(0x03);          //read the Interrupt Status1 register
 spi_read(0x04);          //read the Interrupt Status2 register

 // Now perform a software reset
 spi_write(0x07, 0x80);    //write 0x80 to the Operating & Function Control1 register
  
 // wait for POR interrupt from the radio (while the nIRQ pin is high)
 while (digitalRead(4)==HIGH) {;}
 // read interrupt status registers to clear the interrupt flags and release NIRQ pin
 spi_read(0x03);          //read the Interrupt Status1 register
 spi_read(0x04);          //read the Interrupt Status2 register

 //wait for chip ready interrupt from the radio (while the nIRQ pin is high)
 while (digitalRead(4)==HIGH) {;}
 // read interrupt status registers to clear the interrupt flags and release NIRQ pin
 spi_read(0x03);          //read the Interrupt Status1 register
 spi_read(0x04);          //read the Interrupt Status2 register

 ReadChipInfo();
 SetupSi4432(); 
 InitRX();

 // Configure GPIO pin output definitioms:
 // spi_write (0x0B,0x19); // GPIO0 - Preamble detected [output]
 spi_write (0x0B,0x1B); // GPIO0 - Sync detected [output]
 spi_write (0x0C,0x0F); // GPIO1 - RX clock [output]
 spi_write (0x0D,0x14); // GPIO2 - RX data [output]
 
 // Setup interrupt for SI4432 - do this last since ESP32 does not like an interrupt during setup.
 attachInterrupt(4, ISR, FALLING);
}

/// Keep count of how many MQTT messages we tried to publish
int pubcount= 0;
/**/ int lastlooptime= 0;

// the loop function runs over and over again until power down or reset
void loop()
{
 char msgbuffer[512];
 char *p;
 int count;
/**/ int now;

/**/  now= millis(); if ((now-lastlooptime)>250) { printf ("=== time between loops= %d ===\n",now-lastlooptime); } lastlooptime= now;

/**/ now= millis();
 if (!(WiFi.isConnected()))
 {
  printf ("WIFI not connected - attempt reconnection...");
  psclient.disconnect();
  wificlient.stop();
  WiFi.reconnect();
 }
/**/ if ((millis()-now)>250) printf ("=== WIFI reconnect took %d seconds ===\n");

/**/ now= millis();
 if (!psclient.connected())
 {
  printf ("MQTT connection broken (state=%d). Attemping new connection...",psclient.state());
  // Attempt to connect
  if (psclient.connect("ESP32conlog"))
  {
   printf(" connected\n");
   // Subscribe
   psclient.subscribe("conlog/tx");
  }
  else
  {
   printf (" failed, rc= %d\n",psclient.state());
   delay(1000);
   return;
  }
 }
/**/ if ((millis()-now)>250) printf ("=== MQTT reconnect took %d seconds ===\n");

/**/ if (txdone) {  txdone= 0;  printf ("Packet TX done\n"); }
  
/**/ now= millis();
 if (!psclient.loop())
 {
  printf ("MQTT client no longer connected\n");
 }
/**/ if ((millis()-now)>250) printf ("=== MQTT loop took %d seconds ===\n");

 if (rxfail)
 {
  rxfail= 0;
  printf ("Failed to receive a valid packet --");
  p= msgbuffer;
  for (count=0;count<decodelen;count++)
  {
   *(p++)= hexdigit[(decodebuf[count])>>4];
   *(p++)= hexdigit[(decodebuf[count])&0x0F];
   *(p++)= ' ';
  }
  *(--p)= 0;
  printf ("RSSI %3d RX[%d] %s\n",RSSI,decodelen,msgbuffer);
 }
 
 if (rxdone)
 {
  rxdone= 0;      // Clear flag for next notification
  rxlength-= 2;   // Drop the last two received (CRC) bytes

  p= msgbuffer;
  sprintf (p,"%4d RSSI %3d RX[%d]",pubcount++,RSSI,rxlength);
  p+= strlen(p);
  for (count=0;count<rxlength;count++)
  {
   *(p++)= ' ';
   *(p++)= hexdigit[(rxbuffer[count])>>4];
   *(p++)= hexdigit[(rxbuffer[count])&0x0F];
  }
  *p= 0;
  
#define SERIAL_OUT  
#ifdef SERIAL_OUT  
/**/ now= millis();
  printf ("%s\n",msgbuffer);
/**/ if ((millis()-now)>250) printf ("=== Serial printout took %d seconds ===\n");
#endif

/**/ now= millis();
  psclient.publish("conlog/rx",msgbuffer);
/**/ if ((millis()-now)>250) printf ("=== MQTT publish took %d seconds ===\n");
 }

 /**/  lastlooptime= millis();
}

