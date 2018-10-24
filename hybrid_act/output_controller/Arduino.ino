//--------------------------------------------------------------------------
// Kyle DeProw 
// 9-13-2018 
//--------------------------------------------------------------------------

//#define encoderPinA 2
//#define encoderPinB 3


int pwmPin = 5;
int dir1 = 6;
int dir2 = 7;

int s1 = 3;
int s2 = 4;
int s3 = 5;
int s4 = 6;
int s5 = 7;

//array a = [1,2,3,4,5]);
int incomingMsgSize = 3;
byte motor_speed_msb;
byte motor_speed_lsb;
signed short motor_speed;
char resetChar = 'r';
char startChar = '3';
byte currentCase = startChar;

int iter = 0;
bool IGNORE_FLAG;
int msgTimeout = 1e4;
char inByte;
byte checksum;

//  encoder
/*volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile short encoderCount = 0;
*/


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------

void setup()
{
  Serial.begin(57600);

  // encoder
  //pinMode(encoderPinA, INPUT); 
  //pinMode(encoderPinB, INPUT); 

  // H-Bridge
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  
  // turn on pullup resistors
  //digitalWrite(encoderPinA, HIGH);
  //digitalWrite(encoderPinB, HIGH);

  // encoder pin on interrupt 0 (pin 2)
 // attachInterrupt(0, doEncoder, CHANGE);

}

// SendTwoByte Int accepts 2 byte int input and sends corresponding msb and lsb
//   over Serial
void SendTwoByteInt(int intin )
{
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

// CommandMotor accepts accepts a speed between -255 and 255 and outputs
//   speed to PWM pin
void CommandMotor(int speed){
  if (speed > 0){
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwmPin, speed);
  }
  else if (speed < 0){
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, abs(speed));
  }
  else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //Loop until Serial is available
  if ( Serial.available() > 0 ) {
    inByte = Serial.read();
    // When Serial is available, look for start character
    if ( inByte == resetChar ) { 
      //encoderCount = 0;
      checksum = resetChar;
      Serial.write(checksum);
    }
    else if ( inByte == '2' ) {
        startChar = '2';
        // UFM Case
        iter = 0;
        IGNORE_FLAG = false;
        while ( Serial.available() < incomingMsgSize - 1 ) 
        {
            iter++;
            // Handling a timeout condition
            if ( iter > msgTimeout ) {
                IGNORE_FLAG = true;
                break;
            }
        }
        if ( !IGNORE_FLAG ) {
          
          motor_speed_msb = Serial.read();
          motor_speed_lsb = Serial.read();
          checksum = startChar + motor_speed_msb + motor_speed_lsb;
          motor_speed = ((unsigned short)motor_speed_msb << 8) + motor_speed_lsb;
          Serial.write(checksum);
          }
   }
    
    else if (inByte ==  '3'){
        startChar = '3';
    // EV Case
        iter = 0;
        IGNORE_FLAG = false;
        while ( Serial.available() < incomingMsgSize - 1 ) 
        {
            iter++;
            // Handling a timeout condition
            if ( iter > msgTimeout ) {
                IGNORE_FLAG = true;
                break;
            }
        }
        if ( !IGNORE_FLAG ) {
          
          motor_speed_msb = Serial.read();
          motor_speed_lsb = Serial.read();
          checksum = startChar + motor_speed_msb + motor_speed_lsb;
          motor_speed = ((unsigned short)motor_speed_msb << 8) + motor_speed_lsb;
          Serial.write(checksum);
        }
    }
    else if (inByte == '4'){
    // Analog Signal reads
        startChar = '4';
        iter = 0;
        IGNORE_FLAG = false;
        int val = 0;
        int i = 0;
        while ( Serial.available() < incomingMsgSize - 1 ) 
        {
            iter++;
            // Handling a timeout condition
            if ( iter > msgTimeout ) {
                IGNORE_FLAG = true;
                break;
            }
          }
          if ( !IGNORE_FLAG ) {
          
          motor_speed_msb = Serial.read();
          motor_speed_lsb = Serial.read();
          checksum = startChar + motor_speed_msb + motor_speed_lsb;
          motor_speed = ((unsigned short)motor_speed_msb << 8) + motor_speed_lsb;
          Serial.write(checksum);
          
          for(int i = 1; i<=5; ++i){
          val = analogRead(i);
          SendTwoByteInt(val);
          }
    };

          
          //SendTwoByteInt(encoderCount);
        }
        
        }
    }
  

