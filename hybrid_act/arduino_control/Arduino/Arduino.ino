//----------------------------------------------------------------------
// Kyle DeProw
// 10-14-2018
//----------------------------------------------------------------------

#include <SPI.h>
#include <MD_AD9833.h>
#include <Adafruit_SI5351.h>

// Pins for SPI comm with the AD9833 IC
#define DATA  11  ///< SPI Data pin number
#define CLK   13  ///< SPI Clock pin number
#define FSYNC_UFM 10  ///< SPI Load pin number (FSYNC in AD9833 usage)
#define FSYNC_EV 9  ///< SPI Load pin number (FSYNC in AD9833 usage)

MD_AD9833  UFM(FSYNC_UFM); // Hardware SPI
MD_AD9833  EV(FSYNC_EV); // Hardware SPI

int f0 = A0;
int f1 = A1;
int f2 = A2;
int f3 = A3;
int f4 = A4;

int clockReset = 4;
int Ad9833reset = 5;
int relay1 = 6;
int relay2 = 7;

unsigned short f0_read;
unsigned short f1_read;
unsigned short f2_read;
unsigned short f3_read;
unsigned short f4_read;

// deprecated cases, here for historical record byte ufmCase = 2;
byte evCase = 3;
byte freqCase = 3;
// Used cases
byte forceCase = 4;
byte phaseCase = 5;
byte versionCase = 6;

byte count = 0;
byte ufmFreqMsb;
byte ufmFreqLsb;
byte evFreqMsb;
byte evFreqLsb;
byte phase_msb;
byte phase_lsb;
unsigned short phase;
unsigned short freq;
char versionIn ;

// buffers for parameters
char hueVersion ;
unsigned short evFreq ;
unsigned short ufmFreq ;

int iter = 0;
bool IGNORE_FLAG;
int msgTimeout = 1e4;
char inByte;
int checksum;

Adafruit_SI5351 clockgen = Adafruit_SI5351();

//--------------------------------------------------------------------------------
void setup() {
  Serial.begin(57600);

  pinMode(f0,INPUT);
  pinMode(f1,INPUT);
  pinMode(f2,INPUT);
  pinMode(f3,INPUT);
  pinMode(f4,INPUT);

  pinMode(clockReset,OUTPUT);
  pinMode(Ad9833reset,OUTPUT);
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);

  digitalWrite( clockReset, LOW );
  digitalWrite( Ad9833reset, LOW );
  digitalWrite( clockReset, HIGH );

  delay(1000);
  /* Initialise the sensor */
  if (clockgen.begin() != ERROR_NONE)
  {
    /* There was a problem detecting the IC ... check your connections */
    while(1);
  }
  /* FRACTIONAL MODE --> More flexible but introduce clock jitter */
  /* Setup PLLB to fractional mode (XTAL * 16) */
  /* Setup Multisynth 1 to 25MHz (PLLB/16) */
  clockgen.setupPLL(SI5351_PLL_B, 16, 0, 1); 
  clockgen.setupMultisynth(1, SI5351_PLL_B, 16, 0, 1);
    
  /* Enable the clocks */
  clockgen.enableOutputs(true);

  UFM.begin();
  EV.begin();
  //startHueVersion( 1, 31000, 31000 );
}

void loop() {
  if ( Serial.available() > 0 ) {
    inByte = Serial.read();

    if ( inByte == freqCase )
    {
      // Wait for full incoming packet before moving on
      iter = 0;
      IGNORE_FLAG = false;
      while ( Serial.available() < 4 ) {
        iter++;
        // Handling a timeout condition
        if ( iter > msgTimeout ) {
        IGNORE_FLAG = true;
        break;
        }
      }

      if ( !IGNORE_FLAG ) 
      {
        checksum = freqCase;
        
        ufmFreqMsb = Serial.read();
        ufmFreqLsb = Serial.read();
        evFreqMsb = Serial.read();
        evFreqLsb = Serial.read();
  
        Serial.write(freqCase);
        Serial.write(ufmFreqMsb);
        Serial.write(ufmFreqLsb);
        Serial.write(evFreqMsb);
        Serial.write(evFreqLsb);
        checksum += ufmFreqMsb ;
        checksum += ufmFreqLsb ;
        checksum += evFreqMsb ;
        checksum += evFreqLsb ;
        
        ufmFreq = ((unsigned short) ufmFreqMsb << 8) + ufmFreqMsb ;
        evFreq = ((unsigned short) evFreqMsb << 8) + evFreqLsb ;
        checksum = checksum & 0xff;
        Serial.write(checksum);
      }
    }

  
    if ( inByte == forceCase ) {
      checksum = forceCase;

      f0_read = analogRead(f0);
      f1_read = analogRead(f1);
      f2_read = analogRead(f2);
      f3_read = analogRead(f3);
      f4_read = analogRead(f4);

      Serial.write(checksum & 0xff);
      SendTwoByteInt(f0_read);
      SendTwoByteInt(f1_read);
      SendTwoByteInt(f2_read);
      SendTwoByteInt(f3_read);
      SendTwoByteInt(f4_read);
      }

    else if ( inByte == phaseCase ) {
      // Wait for full incoming packet before moving on
      iter = 0;
      IGNORE_FLAG = false;
      while ( Serial.available() < 2 ) {
        iter++;
        // Handling a timeout condition
        if ( iter > msgTimeout ) {
        IGNORE_FLAG = true;
        break;
        }
      }
      if ( !IGNORE_FLAG ) {
        phase_msb = Serial.read();
        phase_lsb = Serial.read();

        phase = ((unsigned short)phase_msb << 8) + phase_lsb;

        checksum = phaseCase + phase_msb + phase_lsb;       // + phase_byte; 
        MD_AD9833::channel_t chan;
        chan = MD_AD9833::CHAN_0;
        EV.setPhase(chan, phase);

        Serial.write(checksum & 0xff);
      }
    }

    else if ( inByte == versionCase ) {
      iter = 0;
      IGNORE_FLAG = false;
      while( Serial.available() < 5 ) {
        iter++;
        // Handling a timeout condition
        if ( iter > msgTimeout ) {
        IGNORE_FLAG = true;
        break;
        }
      }

      if ( !IGNORE_FLAG ) 
      {
        checksum = versionCase;
        hueVersion = Serial.read() ;
        ufmFreqMsb = Serial.read();
        ufmFreqLsb = Serial.read();
        evFreqMsb = Serial.read();
        evFreqLsb = Serial.read();

        checksum += hueVersion ;
        checksum += ufmFreqMsb ;
        checksum += ufmFreqLsb ;
        checksum += evFreqMsb ;
        checksum += evFreqLsb ;
        
        ufmFreq = ((unsigned short) ufmFreqMsb << 8) + ufmFreqMsb ;
        evFreq = ((unsigned short) evFreqMsb << 8) + evFreqLsb ;
        checksum = checksum & 0xff;
        Serial.write(checksum);
        startHueVersion( hueVersion, ufmFreq, evFreq ) ;
      }
    }
  }
}

//--------------------------------------------------------------------------------

// SendTwoByte Int accepts 2 byte int input and sends corresponding msb and lsb
//   over Serial
void SendTwoByteInt( int intin ){
  unsigned char lsb, msb;
  lsb = ( unsigned char )intin;
  msb = getsecondbyte(intin);
  Serial.write(msb);
  Serial.write(lsb);
}

unsigned char getsecondbyte( int input ){
    unsigned char output;
    output = ( unsigned char )(input >> 8);
    return output;
}

void startHueVersion( char versionIn, short ufmFreq, short evFreq ) 
{
  if ( versionIn == 1 ) { digitalWrite( relay1, HIGH ) ; digitalWrite( relay2, HIGH ) ; }
  if ( versionIn == 2 ) { digitalWrite( relay1, LOW ) ; digitalWrite( relay2, LOW ) ; }
  resetAd9833( evFreq, ufmFreq ) ;
  hueVersion = versionIn ;
}

void resetAd9833( short ufmFreq, short evFreq ) 
{
  digitalWrite( Ad9833reset, LOW );
  delay( 1500 ) ;
  digitalWrite( Ad9833reset, HIGH );
  delay( 200 ) ;
  MD_AD9833::mode_t mode;
  mode = MD_AD9833::MODE_SINE;
  UFM.setMode(mode);
  EV.setMode(mode);

  MD_AD9833::channel_t chan;
  chan = MD_AD9833::CHAN_0;
  EV.setFrequency(chan, evFreq); 
  UFM.setFrequency(chan, ufmFreq);
}
