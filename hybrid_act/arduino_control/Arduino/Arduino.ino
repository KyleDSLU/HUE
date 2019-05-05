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

int Ad9833reset = 5;
int relay1 = 6;
int relay2 = 7;

unsigned short f0_read;
unsigned short f1_read;
unsigned short f2_read;
unsigned short f3_read;
unsigned short f4_read;

byte ufmCase = 2;
byte evCase = 3;
byte forceCase = 4;
byte phaseCase = 5;
byte versionCase = 6;
byte count = 0;
byte freq_msb;
byte freq_lsb;
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
byte checksum;

Adafruit_SI5351 clockgen = Adafruit_SI5351();

//--------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(f0,INPUT);
  pinMode(f1,INPUT);
  pinMode(f2,INPUT);
  pinMode(f3,INPUT);
  pinMode(f4,INPUT);

  pinMode(Ad9833reset,OUTPUT);
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);

  digitalWrite( Ad9833reset, LOW );
  UFM.begin();
  EV.begin();
  startHueVersion( 1, 21000, 30900 ) ;
}

void loop() {
  if ( Serial.available() > 0 ) {
    inByte = Serial.read();
    if ( inByte == ufmCase ) {
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
        freq_msb = Serial.read();
        freq_lsb = Serial.read();
        //phase_byte = Serial.read();

        checksum = ufmCase + freq_msb + freq_lsb;// + phase_byte;
        freq = ((unsigned short)freq_msb << 8) + freq_lsb;

        //startHueVersion( hueVersion, evFreq, freq ) ;
        ufmFreq = freq ;
        Serial.write(checksum);
      }
    }

    else if ( inByte == evCase ) {
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
        freq_msb = Serial.read();
        freq_lsb = Serial.read();

        checksum = evCase + freq_msb + freq_lsb;// + phase_byte;
        freq = ((unsigned short)freq_msb << 8) + freq_lsb;
        //startHueVersion( hueVersion, freq, ufmFreq ) ;
        evFreq = freq ;
        Serial.write(checksum);
      }
    }

    else if ( inByte == forceCase ) {
      checksum = forceCase;

      f0_read = analogRead(f0);
      f1_read = analogRead(f1);
      f2_read = analogRead(f2);
      f3_read = analogRead(f3);
      f4_read = analogRead(f4);

      Serial.write(checksum);
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
          //EV.setFrequency(chan, freq);
          EV.setPhase(chan, phase);

          Serial.write(checksum);
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
      if ( !IGNORE_FLAG ) {
      versionIn = Serial.read();
      hueVersion = versionIn ;
      checksum = versionCase + versionIn ;

      freq_msb = Serial.read();
      freq_lsb = Serial.read();
      evFreq = ((unsigned short) freq_msb << 8) + freq_lsb ;
      checksum += freq_msb + freq_lsb ;

      freq_msb = Serial.read();
      freq_lsb = Serial.read();
      ufmFreq = ((unsigned short) freq_msb << 8) + freq_lsb ;
      checksum += freq_msb + freq_lsb ;
      Serial.write(checksum);
      startHueVersion( hueVersion, evFreq, ufmFreq ) ;
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

void startHueVersion( char versionIn, short evFreq, short ufmFreq ) 
{
  resetAd9833( evFreq, ufmFreq ) ;
  if ( versionIn == 1 ) { digitalWrite( relay1, LOW ) ; digitalWrite( relay2, LOW ) ; }
  if ( versionIn == 2 ) { digitalWrite( relay1, HIGH ) ; digitalWrite( relay2, HIGH ) ; }
  hueVersion = versionIn ;
}

void resetAd9833( short evFreq, short ufmFreq ) 
{
  digitalWrite( Ad9833reset, LOW );
  delay( 1500 ) ;
  digitalWrite( Ad9833reset, HIGH );
  delay( 0 ) ;
  /* Initialise the sensor */
  if (clockgen.begin() != ERROR_NONE)
  {
    /* There was a problem detecting the IC ... check your connections */
    while(1);
    Serial.println("Here");
  }
  /* FRACTIONAL MODE --> More flexible but introduce clock jitter */
  /* Setup PLLB to fractional mode (XTAL * 16) */
  /* Setup Multisynth 1 to 25MHz (PLLB/16) */
  clockgen.setupPLL(SI5351_PLL_B, 16, 0, 1); 
  clockgen.setupMultisynth(1, SI5351_PLL_B, 16, 0, 1);
    
  /* Enable the clocks */
  clockgen.enableOutputs(true);
  MD_AD9833::mode_t mode;
  mode = MD_AD9833::MODE_SINE;
  UFM.setMode(mode);
  EV.setMode(mode);

  MD_AD9833::channel_t chan;
  chan = MD_AD9833::CHAN_0;
  EV.setFrequency(chan, evFreq); 
  UFM.setFrequency(chan, ufmFreq);
}
