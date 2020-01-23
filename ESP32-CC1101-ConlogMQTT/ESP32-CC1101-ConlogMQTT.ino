#include <SPI.h>
#include <WiFi.h>
// Must override MQTT_MAX_PACKET_SIZE in the include file below, can't override it here.
#include <PubSubClient.h>

#ifndef WIFI_SSID
#define WIFI_SSID "xxxx"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "xxxx"
#endif
#ifndef MQTT_SERVER
#define MQTT_SERVER "1.2.3.4"
#endif

// Radio config
#define MAX_RX_LEN  64               // Max rx packet length is 64 bytes .. Value must be less than 255
unsigned char rxbuffer[MAX_RX_LEN];  // Saved copy of complete received message
int rxlength= 0;                     // Number of bytes in received message

int rxdone= 0;
int rxfail= 0;
int RSSI;

#define GDO0_PIN    4
#define GDO2_PIN    22
#define SS_PIN      5
#define MOSI_PIN    23
#define MISO_PIN    19
#define SCLK_PIN    18

// WIFI and MQTT client globals
WiFiClient    wificlient;
PubSubClient  psclient(wificlient);

// SPI
static const int spiClk = 1000000; // 1 MHz
SPIClass * vspi = NULL;   //uninitalised pointers to SPI objects


///  Must relook : 1. auto calibration; CCA before TX (retry TX if RX busy)
// Rf settings for CC1101
unsigned char cc1101_regs[]=
{
    0x40,  // IOCFG2        GDO2 Output Pin Configuration -- RX FIFO over treshold, active low. Clears when FIFO no longer orver threshold
    0x2E,  // IOCFG1        GDO1 Output Pin Configuration -- Tristate
    0x46,  // IOCFG0        GDO0 Output Pin Configuration -- Goes low when sync is sent/received, goes high at end of packet
    0x40,  // FIFOTHR       RX FIFO and TX FIFO Thresholds -- 4 bytes RX, 61 bytes TX FIFO, set ADC_RETENTION
    0xD3,  // SYNC1         Sync Word, High Byte
    0x91,  // SYNC0         Sync Word, Low Byte
    0xFF,  // PKTLEN        Packet Length
    0x00,  // PKTCTRL1      Packet Automation Control -- No address, No RSSI,LQI on RX, No bad CRC auto flush, Always accept sync word
    0x44,  // PKTCTRL0      Packet Automation Control -- Whitening on, Normal FIFO mode, CRC enabled for RX and TX, Fixed length packet (update reg 6 PKTLEN between rx and tx)
    0x00,  // ADDR          Device Address -- Don't use device addresses / filtering
    0x00,  // CHANNR        Channel Number -- May in future use a channel number relative to bases Conlog wireless frequency
    0x06,  // FSCTRL1       Frequency Synthesizer Control -- Per SmartRF
    0x00,  // FSCTRL0       Frequency Synthesizer Control -- Per SmartRF
    0x10,  // FREQ2         Frequency Control Word, High Byte -- Per SmartRF
    0xB3,  // FREQ1         Frequency Control Word, Middle Byte -- Per SmartRF
    0x26,  // FREQ0         Frequency Control Word, Low Byte -- Per SmartRF
    0xCA,  // MDMCFG4       Modem Configuration -- Per SmartRF
    0x83,  // MDMCFG3       Modem Configuration -- Per SmartRF
    0x17,  // MDMCFG2       Modem Configuration -- Per SmartRF (GFSK, No manchester, 30 bit sync word detect+above carrier sense threshold
    0xA2,  // MDMCFG1       Modem Configuration -- Enable FEC, 4 bytes preamble, 200kHZ channel spacing (SmartRF)
    0xF8,  // MDMCFG0       Modem Configuration -- 200kHZ channel spacing (SmartRF)
    0x43,  // DEVIATN       Modem Deviation Setting -- Per SmartRF
    0x07,  // MCSM2         Main Radio Control State Machine Configuration -- No RX timeouts
    0x2F,  // MCSM1         Main Radio Control State Machine Configuration -- CCA clear if not busy with RX packet, RXOFF stay in RX, TXOFF goto RX
    0x18,  // MCSM0         Main Radio Control State Machine Configuration -- Autocal when going from IDLE, no pin control for radio
    0x16,  // FOCCFG        Frequency Offset Compensation Configuration -- Per SmartRF
    0x6C,  // BSCFG         Bit Synchronization Configuration -- Per SmartRF
    0x43,  // AGCCTRL2      AGC Control -- Per SmartRF
    0x40,  // AGCCTRL1      AGC Control -- Per SmartRF
    0x91,  // AGCCTRL0      AGC Control -- Per SmartRF
    0x87,  // WOREVT1       High Byte Event0 Timeout -- Per SmartRF
    0x6B,  // WOREVT0       Low Byte Event0 Timeout -- Per SmartRF
    0xFB,  // WORCTRL       Wake On Radio Control -- Per SmartRF
    0x56,  // FREND1        Front End RX Configuration -- Per SmartRF
    0x10,  // FREND0        Front End TX Configuration -- Per SmartRF
    0xE9,  // FSCAL3        Frequency Synthesizer Calibration -- Per SmartRF
    0x2A,  // FSCAL2        Frequency Synthesizer Calibration -- Per SmartRF
    0x00,  // FSCAL1        Frequency Synthesizer Calibration -- Per SmartRF
    0x1F,  // FSCAL0        Frequency Synthesizer Calibration -- Per SmartRF
    0x41,  // RCCTRL1       RC Oscillator Configuration -- Per SmartRF
    0x00,  // RCCTRL0       RC Oscillator Configuration -- Per SmartRF
    0x59,  // FSTEST        Frequency Synthesizer Calibration Control  -- Per SmartRF
    0x7F,  // PTEST         Production Test -- Per SmartRF
    0x3F,  // AGCTEST       AGC Test -- Per SmartRF
    0x81,  // TEST2         Various Test Settings -- Per SmartRF
    0x35,  // TEST1         Various Test Settings -- Per SmartRF
    0x09,  // TEST0         Various Test Settings -- Per SmartRF
};

// CC1101 configuration registers
#define REG_IOCFG2       0x00
#define REG_IOCFG1       0x01
#define REG_IOCFG0       0x02
#define REG_FIFOTHR      0x03
#define REG_SYNC1        0x04
#define REG_SYNC0        0x05
#define REG_PKTLEN       0x06
#define REG_PKTCTRL1     0x07
#define REG_PKTCTRL0     0x08
#define REG_ADDR         0x09
#define REG_CHANNR       0x0A
#define REG_FSCTRL1      0x0B
#define REG_FSCTRL0      0x0C
#define REG_FREQ2        0x0D
#define REG_FREQ1        0x0E
#define REG_FREQ0        0x0F
#define REG_MDMCFG4      0x10
#define REG_MDMCFG3      0x11
#define REG_MDMCFG2      0x12
#define REG_MDMCFG1      0x13
#define REG_MDMCFG0      0x14
#define REG_DEVIATN      0x15
#define REG_MCSM2        0x16
#define REG_MCSM1        0x17
#define REG_MCSM0        0x18
#define REG_FOCCFG       0x19
#define REG_BSCFG        0x1A
#define REG_AGCCTRL2     0x1B
#define REG_AGCCTRL1     0x1C
#define REG_AGCCTRL0     0x1D
#define REG_WOREVT1      0x1E
#define REG_WOREVT0      0x1F
#define REG_WORCTRL      0x20
#define REG_FREND1       0x21
#define REG_FREND0       0x22
#define REG_FSCAL3       0x23
#define REG_FSCAL2       0x24
#define REG_FSCAL1       0x25
#define REG_FSCAL0       0x26
#define REG_RCCTRL1      0x27
#define REG_RCCTRL0      0x28
#define REG_FSTEST       0x29
#define REG_PTEST        0x2A
#define REG_AGCTEST      0x2B
#define REG_TEST2        0x2C
#define REG_TEST1        0x2D
#define REG_TEST0        0x2E

// CC1101 Status regitsters
#define STATUS_PARTNUM      0x30
#define STATUS_VERSION      0x31
#define STATUS_FREQEST      0x32
#define STATUS_LQI          0x33
#define STATUS_RSSI         0x34
#define STATUS_MARCSTATE    0x35
#define STATUS_WORTIME1     0x36
#define STATUS_WORTIME0     0x37
#define STATUS_PKTSTATUS    0x38
#define STATUS_VCO_VC_DAC   0x39
#define STATUS_TXBYTES      0x3A
#define STATUS_RXBYTES      0x3B

// CC11101 command strobes
#define STROBE_SRES      0x30
#define STROBE_SFSTXON   0x31
#define STROBE_SXOFF     0x32
#define STROBE_SCAL      0x33
#define STROBE_SRX       0x34
#define STROBE_STX       0x35
#define STROBE_SIDLE     0x36
#define STROBE_SAFC      0x37
#define STROBE_SWOR      0x38
#define STROBE_SPWD      0x39
#define STROBE_SFRX      0x3A
#define STROBE_SFTX      0x3B
#define STROBE_SWORRST   0x3C
#define STROBE_SNOP      0x3D

// CC1101 FIFO access
#define REG_FIFO         0x3F


////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char IRAM_ATTR spi_read (unsigned char reg)
{
 unsigned char status;
 unsigned char readval;
 
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 status= vspi->transfer((reg&0x3F)|0x80); // High bit high = read, 2MSB low = not burst. Status is returned as side effect.
 readval= vspi->transfer(0);              // Read register value
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return readval;
}

unsigned char IRAM_ATTR spi_read_status_reg (unsigned char reg)
{
 unsigned char  oldvalue;
 unsigned char  newvalue;
 int            tries= 10;
 
 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW);              // pull SS slow to prep other end for transfer
 vspi->transfer((reg&0x3F)|0xC0);   // High bit high = RX FIFO info, 2MSB high = read status regs (0x30-0x3D)
 newvalue= vspi->transfer(0);       // Read register value
 do
 {
  // Read status register until two reads return the same value - work around silicon errata.
  oldvalue= newvalue;
  vspi->transfer((reg&0x3F)|0xC0);   // High bit high = RX FIFO info, 2MSB high = read status regs (0x30-0x3D)
  newvalue= vspi->transfer(0);       // Read register value 
 }
 while ((newvalue!=oldvalue)&&(--tries));
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return newvalue;
}

unsigned char IRAM_ATTR spi_burst_read (unsigned char reg, unsigned char *buffer, int count)
{
 unsigned char status;

 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 status= vspi->transfer((reg&0x3F)|0xC0);   // High bit high = read, 2MSB high = burst.
 while ((count--)>0)
  *(buffer++) = vspi->transfer(0);
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return status;
}

unsigned char IRAM_ATTR spi_write (unsigned char reg, unsigned char val)
{
 unsigned char status;

 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 status= vspi->transfer((reg&0x3F)|0x00);   // High bit low = write, 2MSB low = not burst.
 status= vspi->transfer(val);
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return status;
}

unsigned char IRAM_ATTR spi_write_strobe (unsigned char reg)
{
 unsigned char status;

 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 status= vspi->transfer((reg&0x3F)|0x80);   // High bit high = RX FIFO info, 2MSB low = write strobe regs (0x30-0x3D)
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return status;
}

unsigned char IRAM_ATTR spi_burst_write (unsigned char reg, unsigned char *buffer, int count)
{
 unsigned char status;

 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 status= vspi->transfer((reg&0x3F)|0x40);   // High bit low = write, 2MSB high = burst.
 while ((count--)>0)
  status= vspi->transfer(*(buffer++));
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return status;
}

unsigned char IRAM_ATTR spi_read_radio_status (void)
{
 unsigned char  oldvalue;
 unsigned char  newvalue;
 int            tries= 10;

 vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
 digitalWrite(5, LOW); //pull SS slow to prep other end for transfer
 newvalue= vspi->transfer(STROBE_SNOP|0x80);   // High bit high = RX FIFO info, 2MSB low = write strobe regs (0x30-0x3D)
 do
 {
  // Read status register until two reads return the same value - work around silicon errata.
  oldvalue= newvalue;
  newvalue= vspi->transfer(STROBE_SNOP|0x80);   // High bit high = RX FIFO info, 2MSB low = write strobe regs (0x30-0x3D)
 }
 while ((newvalue!=oldvalue)&&(--tries));
 digitalWrite(5, HIGH); //pull ss high to signify end of data transfer
 vspi->endTransaction();
 return newvalue;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// Calculate checksum. Updates running checksum with new message byte
// Initialise checksum to 0xFFFF before message, final checksum should be 0000 for complete message
unsigned short IRAM_ATTR calcCRC(unsigned char crcData, unsigned short crcReg)
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

unsigned char   incomingbuf[MAX_RX_LEN];      // Holding buffer for incoming packet bytes
int             incominglen;                  // Number of bytes in the incoming packet buffer
unsigned short  checksum;                     // Running checksum of the incoming packet

int             txdone= 0;
unsigned char   TRXmode= 0;

int             LastCAL= 0;

// Init reception of new packet
void IRAM_ATTR StartRX (void)
{
 int  now= millis();

 if ((now-LastCAL)>10000)   // Run calibration if the last calibration was more than 10 seconds ago
 {
  LastCAL= now;
  spi_write (REG_MCSM0,0x18);              // Enable calibration when going from IDLE to RX/TX
 }
 checksum = 0xFFFF; // Init value for CRC calculation
 spi_write_strobe (STROBE_SIDLE);          // try to go into idle before restarting RX. seem to help with restarting RX for new packet
 spi_write_strobe (STROBE_SFRX);           // Flush RX FIFO strobe ... is this the best order?
 spi_write_strobe (STROBE_SRX);            // RX mode strobe
 spi_write (REG_PKTLEN, MAX_RX_LEN);       // Allow for maximum length packet
 spi_write (REG_MCSM0,0x08);               // Disable automatic calibration
 TRXmode= 0;
}

// Add received byte to the incoming packet buffer. return 1 when decoded stream passes CRC check.
int IRAM_ATTR AddNewRX (unsigned char newbyte)
{
 // Check incomingbuf overflow here?
 incomingbuf[incominglen++]= newbyte;
 checksum= calcCRC(newbyte,checksum);
 if (!checksum) return 1;
 return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

/**/ int GDO0int= 0;
/**/ int GDO2int= 0;
/**/ int SyncFlag= 0;
/**/ int EndFlag= 0;
/**/ int LastStatus= 0;


void IRAM_ATTR GDO2_ISR()
{
 unsigned char  numbytes;

 /**/ GDO2int=1;
 /**/ digitalWrite (21, HIGH);

 numbytes= spi_read_status_reg(STATUS_RXBYTES);
 if (numbytes&0x80)              // Test for RX FIFO overflow... should not happen
 {
  StartRX();
  rxfail= 1;
 /**/ digitalWrite (21, LOW);
  return;
 }
 
 // According to datasheet page 56 the RX FIFO should not be totally emptied while still receiving a packet
 while ((numbytes--)>1)
 {
  if (AddNewRX(spi_read(REG_FIFO)))
  {
   // Successfully received a complete packet
   rxlength= incominglen;
   memcpy (rxbuffer,incomingbuf,incominglen);
   StartRX();
   rxdone= 1;
 /**/ digitalWrite (21, LOW);
   return;
  }
  if (incominglen==MAX_RX_LEN)
  {
   StartRX();
   rxfail= 1;
 /**/ digitalWrite (21, LOW);
   return;
  }  
 }
 /**/ digitalWrite (21, LOW);
}

void IRAM_ATTR GDO0_ISR()
{
// unsigned char  status= spi_write_strobe (STROBE_SNOP) & 0x70;
 unsigned char status= spi_read_radio_status() & 0x70;
 unsigned char numbytes;

  /**/ GDO0int=1;
  /**/ digitalWrite (15, HIGH);
  /**/  LastStatus= status;

 if (status==0x70)
 {
  // TX FIFO underflow
  spi_write_strobe (STROBE_SFTX);
  txdone=-1;
  StartRX();
  /**/ digitalWrite (15, LOW);
  return;
 }
 if (status==0x60)
 {
  // RX FIFO overflow
  StartRX();
  rxfail= 1;
  /**/ digitalWrite (15, LOW);
  return;
 }

 if (digitalRead(GDO0_PIN)==LOW)
 {
  /**/ SyncFlag= 1;

  TRXmode= status;
  // Sync word sent/received
  if (status==0x10)
  {
   // Radio in RX mode
   incominglen= 0;
   RSSI= spi_read_status_reg (STATUS_RSSI);  
  }
  /**/ digitalWrite (15, LOW);
  return;
 }

 /**/ EndFlag= 1;

 // End of packet transmission/reception
 if (TRXmode==0x10)
 {
  numbytes= spi_read_status_reg(STATUS_RXBYTES);
  while (numbytes--)
  {
   if (AddNewRX(spi_read(REG_FIFO)))
   {
    // Successfully Received a complete packet
    rxlength= incominglen;
    memcpy (rxbuffer,incomingbuf,incominglen);
    StartRX();
    rxdone= 1;
   /**/ digitalWrite (15, LOW);
    return;
   }
   if (incominglen==MAX_RX_LEN)
   {
    StartRX();
    rxfail= 1;
    /**/ digitalWrite (15, LOW);
    return;
   }  
  }
 }
 if (TRXmode==0x20)
 {
  txdone= 1;
  StartRX();
  /**/ digitalWrite (15, LOW);
 }
 // Should not get here...
 StartRX();
 /**/ digitalWrite (15, LOW);
}

void SendPacket (unsigned char *data, int length)
{
 unsigned char  sendbuf[256];
 int            sendlen;
 int            count;

 // Ideally we would check here if we are actively receiving a packet and delay until done
 // Else we clobber receive packet length 
 spi_burst_write(REG_FIFO,data,length);       // Put data to send in the send FIFO
 spi_write (REG_PKTLEN,length);               // Set packet length. CRC will be added automatically
 spi_write_strobe (STROBE_STX);               // And send the packet
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// General setup for Si4432 chip operating at 434.195MHz, GFSK, 38.4kbps, 75khz(?) deviation
void SetupCC1101 (void)
{
 // Values based on SmartRF calculator
 printf ("Regs size %d\n",sizeof(cc1101_regs));
 spi_burst_write (REG_IOCFG2,cc1101_regs,sizeof(cc1101_regs));    // Sequentially load all CC1101 registers
}

void ReadChipInfo()
{
 //use it as you would the regular arduino SPI API
 int val1= spi_read_status_reg (STATUS_PARTNUM);
 int val2= spi_read_status_reg (STATUS_VERSION);
 printf ("CC1101 part no %u, version %u\n",val1,val2);
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
 Serial.begin(115200);
 //Serial.begin(921600);
 Serial.setDebugOutput(true);

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

 pinMode(GDO2_PIN, INPUT_PULLUP); // Set interrupt2 (GD2) pin to input
 pinMode(GDO0_PIN, INPUT_PULLUP);  // Set interrupt pin (GD0) to input

 // manual reset CC1101, as per datasheet. send sequence using GPIO before (re)assigning pins to SPI interface
 pinMode(SS_PIN, OUTPUT);
 pinMode(SCLK_PIN, OUTPUT);
 pinMode(MOSI_PIN, OUTPUT);
 pinMode(MISO_PIN, INPUT_PULLUP);

 printf ("Initiate manual reset of CC1101\n");
 digitalWrite (SCLK_PIN,HIGH);    // SCLK high
 digitalWrite (MOSI_PIN,LOW);
 digitalWrite (SS_PIN,LOW);
 delay (10);
 digitalWrite (SS_PIN,HIGH);
 delay (10);
 digitalWrite (SS_PIN,LOW);
 while(digitalRead(MISO_PIN)==HIGH) { /* NOP */ }

 // First part of reset is done. Now initialise SPI and then we send the SRES strobe
 
 // Now initialise vspi with default pins (ignores #defines for SPI pins)
 // SCLK = 18, MISO = 19, MOSI = 23, SS = 5 [SS PIN already set up as output]
 vspi= new SPIClass(VSPI);
 vspi->begin();

 printf ("Sending SRES strobe\n");
 spi_write_strobe (STROBE_SRES);
 delay (50);                // instead of testing value of MISO line
 
 SetupCC1101();
 ReadChipInfo();
 StartRX();
 
 // Setup interrupts for CC1101 - do this last since ESP32 does not like an interrupt during setup.
 attachInterrupt(GDO0_PIN, GDO0_ISR, CHANGE);
 attachInterrupt(GDO2_PIN, GDO2_ISR, FALLING);

/**/
 pinMode(15, OUTPUT);
 pinMode(21, OUTPUT);
 digitalWrite (15, LOW);
 digitalWrite (21, LOW);
/**/
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


/**/ if (GDO2int) { printf ("@"); GDO2int=0; }
/**/ if (GDO0int) { printf ("^"); GDO0int=0; printf ("[LSx%02X]",LastStatus); }
/**/ if (SyncFlag) { printf ("<"); SyncFlag=0; }
/**/ if (EndFlag) { printf (">\n"); EndFlag=0; }

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
  char  clientname[128];
  byte  mac[6]; 

  WiFi.macAddress(mac);
  sprintf (clientname,"ESP32conlog_%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],mac[6]);
  printf ("MQTT connection broken (state=%d). Attemping new connection... client %s",psclient.state(),clientname);
  // Attempt to connect
  if (psclient.connect(clientname))
  {
   printf(" connected\n");
   // Subscribe
   psclient.subscribe("conlog/tx2");
  }
  else
  {
   printf (" failed, rc= %d\n",psclient.state());
   delay(1000);
   return;
  }
 }
/**/ if ((millis()-now)>250) printf ("=== MQTT reconnect took %d seconds ===\n");

/**/ if (txdone) { printf ("Packet TX done %d\n",txdone); txdone= 0; }
  
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
  for (count=0;count<incominglen;count++)
  {
   *(p++)= hexdigit[(incomingbuf[count])>>4];
   *(p++)= hexdigit[(incomingbuf[count])&0x0F];
   *(p++)= ' ';
  }
  *(--p)= 0;
  printf ("RSSI %3d RX[%d] %s\n",RSSI,incominglen,msgbuffer);
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
