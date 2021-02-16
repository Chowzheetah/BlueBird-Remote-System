#include <AltSoftSerial.h> //https://github.com/PaulStoffregen/AltSoftSerial
/*
const int buttonZ=A2;
const int buttonA=A3;
const int buttonB=3;
const int buttonC=10;
const int buttonD=5;
const int buttonR1=6;
const int buttonR2=9;
*/

const int buttons[7]={A2,A3,11,3,6,5,10};//buttons RL, RR, A, B, C, D, Z
const int channelSelectLED[6]={2,4,7,12,A4,A5}; // Pins Selected for Channel Display missing last two digits, but need switch on board with ChUp ChDown

const int joyX=A0; //X Axis
const int joyY=A1; //Y Axis
const int chDownButton=A4; //Switch with Channel Select LED 2^5
const int chUpButton=A5; //Switch with Channel Select LED 2^6

int joystickX;
int joystickY;
int readA4; //will be connected to variable resitor bridge and momentary
int readA5; //will be connected to variable resitor bridge and momentary
int ioValue[7]={0};
int checkSum = 0;

char joyOutput[30]={0};

unsigned long flashLEDtime=0;

const int ATpin=13;

int chButtTime; //Channel Press Time
int chA4State; //saves previous state
int chA5State; //saves previous state


//////////////////////////////HC 12/////////////////////////////
byte incomingByte;
String moduleINFO = "";
char moduleINFOchar;
int channel=0;
char moduleCommand[10];

AltSoftSerial HC12; // HC-12 TX Pin, HC-12 RX Pin
//////////////////////////////HC 12/////////////////////////////

void setup() {  
/////////////initialize Channel Display LEDs////////////////////    
  for(int i=0;i<6;i++){ //Sets Channels Select LEDs
    pinMode(channelSelectLED[i],OUTPUT);
    digitalWrite(channelSelectLED[i],HIGH);
    delay(50);
    digitalWrite(channelSelectLED[i],LOW);
  }
/////////////initialize Channel Display LEDs//////////////////// 

Serial.begin(115200);
HC12.begin(9600);

pinMode(ATpin,OUTPUT);
digitalWrite(ATpin, LOW);//Sets HC12 to AT mode
delay (100);
HC12.write("AT+V");// ask for module model number
delay (50);

  // ==== Storing the incoming data into a String variable
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    moduleINFO += char(incomingByte);    // Add each byte to ReadBuffer string variable  
  }

Serial.println(moduleINFO);

  if (moduleINFO.indexOf("HC-12")){ // string contains HC12 can also use .startsWith

    moduleINFO = "";
    HC12.write("AT+RC");// ask for channel
    delay (50);
    while (HC12.available()) {             // If HC-12 has data
      incomingByte = HC12.read();          // Store each icoming byte from HC-12
      moduleINFO += char(incomingByte);    // Add each byte to ReadBuffer string variable
      }
    moduleINFO.remove(0,5);//remove 5 characters starting from index 0
    channel = (char)moduleINFO.toInt(); //converts moduleINFO to integer as Char

    ChannelLED(); //////updates LED Channel Display/////    
    }
    digitalWrite(ATpin, HIGH);//Sets HC12 to transparent mode

    Serial.println(channel);
    if (channel==0){
    HC12.end();
    delay(50);
    HC12.begin(57600);
    delay(50);
    }

////////////////////initialize Buttons//////////////////////////  
for (int j=0;j<7;j++){ //initiate button pins
  pinMode(buttons[j],INPUT);  
  }
////////////////////initialize Buttons//////////////////////////   



}

void loop() {
/*
  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) {      // If Serial monitor has data
    HC12.write(Serial.read());      // Send that data to HC-12
  }
*/




  joystickX=analogRead(joyX);
    checkSum=checkSum + joystickX;
  joystickY=analogRead(joyY);
    checkSum=checkSum + joystickY;

  readA4 = analogRead(chDownButton);
    checkSum=checkSum + readA4;
  readA5 = analogRead(chUpButton);
    checkSum=checkSum + readA5;

  if (readA4==1023 && chA4State!=readA4 && (millis()-chButtTime)>50){
      chButtTime = millis();
      checkChannel();
      channel=channel-1;
      setChannel();
    }
    chA4State=readA4;

  if (readA5==1023 && chA5State!=readA5 && (millis()-chButtTime)>50){
      chButtTime = millis();
      checkChannel();
      channel=channel+1;
      setChannel();
    }
    chA5State=readA5;

  ChannelLED();

//joyX,joyY,RL,RR,A,B,C,D,readA4,readA5

for(int j=0;j<7;j++){
  ioValue[j]=digitalRead(buttons[j]);
  checkSum=checkSum + ioValue[j];
  }

checkSum=checkSum % 10; //takes last modulus
//delay(5000);
sprintf(joyOutput,"#%04d%04d%01d%01d%01d%01d%01d%01d%01d%04d%04d%01d$\n",joystickX,joystickY,ioValue[0],ioValue[1],ioValue[2],ioValue[3],ioValue[4],ioValue[5],ioValue[6],readA5,readA4,checkSum);
HC12.write(joyOutput);
//Serial.print(joyOutput);



checkSum=0;
//delay(10);
}



void checkChannel(){
digitalWrite(ATpin, LOW);//Sets HC12 to AT mode

delay (100);
HC12.write("AT+V");// ask for module model number
delay (50);

  // ==== Storing the incoming data into a String variable
  while (HC12.available()) {             // If HC-12 has data
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    moduleINFO += char(incomingByte);    // Add each byte to ReadBuffer string variable  
    }
  
  if (moduleINFO.indexOf("HC-12")){ // string contains HC12 can also use .startsWith
    moduleINFO = "";
    HC12.print("AT+RC");// ask for channel
    delay (50);
    while (HC12.available()) {             // If HC-12 has data
      incomingByte = HC12.read();          // Store each icoming byte from HC-12
      moduleINFO += char(incomingByte);    // Add each byte to ReadBuffer string variable

      }
    moduleINFO.remove(0,5);//remove 5 characters starting from index 0
    channel = (char)moduleINFO.toInt(); //converts moduleINFO to integer as Char
    }
/*      else{
      Serial.print("no HC12 detected");
      }
*/
digitalWrite(ATpin, HIGH);//Sets HC12 to transparent mode 
}



void ChannelLED(){
 
if (channel==0 && (millis()-flashLEDtime)<1000){ //Serial.println("flashState = high channel");
    for(int i=0;i<6;i++){ //Sets Channels Select LEDs
        digitalWrite(channelSelectLED[i], HIGH);
    }
  //Serial.print(channel);
  }
else if(channel==0 && (millis()-flashLEDtime)<2000){ //Serial.println("flashState = equal to channel");
    for(int i=0;i<6;i++){ //Sets Channels Select LEDs
        digitalWrite(channelSelectLED[i], bitRead(channel, i));
    }
  //Serial.print(channel);  
  }
else {
  flashLEDtime=millis();
      for(int i=0;i<6;i++){ //Sets Channels Select LEDs
        digitalWrite(channelSelectLED[i], bitRead(channel, i));
    }  
  //Serial.println(channel);  
  }  
}


void setChannel(){
      moduleINFO = "";
      //Serial.println(channel);
      digitalWrite(ATpin, LOW);//Sets HC12 to AT mode
      delay(100);//very imporant to keep this long may need to set to 200 if error
      if(channel<1){
        channel=63;
        }
      else if(channel>63){
        channel=1;
        }
      Serial.print(channel);
      sprintf(moduleCommand,"AT+C%03d", channel);
      HC12.print(moduleCommand);
      //Serial.print(channel);
      delay(100);
      digitalWrite(ATpin, HIGH);//Sets HC12 to Transparent mode
      //delay(50);
        //Will have OK+Cxxx as response
  }
