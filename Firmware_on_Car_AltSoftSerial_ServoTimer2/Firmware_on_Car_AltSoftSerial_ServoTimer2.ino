#include <AltSoftSerial.h> //https://github.com/PaulStoffregen/AltSoftSerial or add from Libary
#include <ServoTimer2.h> //

const int chDownButton=A6;
const int chUpButton=A7;
int readA6; //will be connected to variable resitor bridge and momentary
int readA7; //will be connected to variable resitor bridge and momentary
unsigned long chButtTime; //Channel Press Time
int chA4State; //saves previous state
int chA5State; //saves previous state


//massage into int joyX,joyY,RL,RR,A,B,C,D,Z,tuneA4,tuneA5,checksum - checkSum=checkSum % 10; //takes last modulus

int joyX;
int joyY;
int RL;
int RR;
int A;
int B;
int C;
int D;
int Z;
int tuneA4;
int tuneA5;
int checkSumIncome;
int checkSum;


////// Car Configuration /////
int steeringPin = 3;
int steeringAngle;
ServoTimer2 steeringServo;
int deadzone = 15;

const int motorDirectionPin = A1;
const int motorPWMPin = 5;
int motorDir;
int motorPWM = 0;

unsigned long indicatorLightsTime;
const int frontLeftPin = 11;
const int frontRightPin = 6;
const int backLeftPin = A3;
const int backRightPin = A2;

const int horn = 10;

unsigned long buttATime;
int lights = 0;
int lightsState=0;

unsigned long buttBTime;
int rightFlash = 0;
int rightFlashState = 0;

int backLights = 0;

unsigned long buttCTime;
int hazards = 0;
int hazardState = 0;

unsigned long buttDTime;
int leftFlash = 0;
int leftFlashState=0;


////// Car Configuration ///// 

///// Radio Configruration /////
const int ATpin=13;
const int channelSelectLED[6]={2,4,7,12,A4,A5}; // Pins Selected for Channel Display
unsigned long flashLEDtime=0;
AltSoftSerial HC12;
byte incomingByte;
String moduleINFO = "";
char moduleINFOchar;
int channel=0;
char moduleCommand[26];
String commandLine;
byte moduleByte;
///// Radio Configruration /////



void setup() {
  Serial.begin(115200);

  HC12.begin(9600);
  HC12.setTimeout(3);

///////////////////////////////////Radio Setup/////////////////////////////////////////  
    for(int i=0;i<6;i++){ //Sets Channels Select LEDs
        pinMode(channelSelectLED[i],OUTPUT);
        digitalWrite(channelSelectLED[i],HIGH);
        delay(50);
        digitalWrite(channelSelectLED[i],LOW);        
        } 
  pinMode(ATpin,OUTPUT);
  checkChannel();
  Serial.println(channel);

    if (channel==0){
    HC12.end();
    delay(50);
    HC12.begin(57600);
    delay(50);
    }
///////////////////////////////////Radio Setup/////////////////////////////////////////  

///////////////////////////////////Confguration Setup/////////////////////////////////////////
  steeringServo.attach(3);

  pinMode(motorDirectionPin,OUTPUT);
  pinMode(motorPWMPin,OUTPUT);

  pinMode(frontLeftPin,OUTPUT);
  pinMode(frontRightPin,OUTPUT);
  pinMode(backLeftPin,OUTPUT);
  pinMode(backRightPin,OUTPUT);

  pinMode(horn,OUTPUT);
///////////////////////////////////Confguration Setup/////////////////////////////////////////





  
}

void loop() {

  readA6 = analogRead(chDownButton);
  readA7 = analogRead(chUpButton);

  if (readA6==1023 && chA4State!=readA6 && (millis()-chButtTime)>50){
      chButtTime = millis();
      checkChannel();
      channel=channel-1;
      setChannel();
    }
    chA4State=readA6;

  if (readA7==1023 && chA5State!=readA7 && (millis()-chButtTime)>50){
      chButtTime = millis();
      checkChannel();
      channel=channel+1;
      setChannel();
    }
    chA5State=readA7;

  ChannelLED();


//////////////////////Read Command//////////////////////

if (HC12.available()>=26){
  if(HC12.read()=='#'){
      for(int i=0;i<25;i++){
      moduleCommand[i]=HC12.read();
      //Serial.write(moduleCommand[i]);
      } 
      //Serial.println();
    }
  }
//////////////////////Read Command//////////////////////


//transform incoming message to int, joyX,joyY,RL,RR,A,B,C,D,tuneA4,tuneA4,checksum - checkSum=checkSum % 10; //takes last modulus

joyX = 1000*((int)moduleCommand[0] - 48) + 100*((int)moduleCommand[1] - 48) + 10*((int)moduleCommand[2] - 48) + 1*((int)moduleCommand[3] - 48);
  checkSum=checkSum + joyX;
//  Serial.println(joyX);  
joyY = 1000*((int)moduleCommand[4] - 48) + 100*((int)moduleCommand[5] - 48) + 10*((int)moduleCommand[6] - 48) + 1*((int)moduleCommand[7] - 48);
  checkSum=checkSum + joyY;
//  Serial.println(joyY);   
RL =(int)moduleCommand[8] - 48;
  checkSum=checkSum + RL;
//  Serial.println(RL); 
RR =(int)moduleCommand[9] - 48;
  checkSum=checkSum + RR;
//  Serial.println(RR); 
A =(int)moduleCommand[10] - 48;
  checkSum=checkSum + A;
B =(int)moduleCommand[11] - 48;
  checkSum=checkSum + B;
C =(int)moduleCommand[12] - 48;
  checkSum=checkSum + C;
D =(int)moduleCommand[13] - 48;
  checkSum=checkSum + D;
Z =(int)moduleCommand[14] - 48;
  checkSum=checkSum + Z;
//  Serial.print(A); 
//  Serial.print(B);
//  Serial.print(C);   
//  Serial.println(D); 
tuneA5 = 1000*((int)moduleCommand[15] - 48) + 100*((int)moduleCommand[16] - 48) + 10*((int)moduleCommand[17] - 48) + 1*((int)moduleCommand[18] - 48);
  checkSum=checkSum + tuneA5;
//  Serial.println(tuneA5); 
tuneA4 = 1000*((int)moduleCommand[19] - 48) + 100*((int)moduleCommand[20] - 48) + 10*((int)moduleCommand[21] - 48) + 1*((int)moduleCommand[22] - 48);
  checkSum=checkSum + tuneA4;
//  Serial.println(tuneA4); 

checkSum=checkSum % 10;
checkSumIncome = (int)moduleCommand[23] - 48;



/*
Serial.print(checkSum);
Serial.print(" ");
Serial.println(checkSumIncome);
*/

/*
  for(int j=0;j<180;j++){
    delay(50);
    steeringServo.write(j);
    Serial.println(j);
    }
*/

if (checkSum == checkSumIncome){

  //map motor 
  if (joyY>512+deadzone) {
      motorPWM = map(joyY,512+deadzone,1023,0,255);
      motorDir = 1;
      backLights = 0;
    }
  else if (joyY<512-deadzone) {
      motorPWM = map(joyY,512-deadzone,0,0,255);
      motorDir = 0;
      backLights = 1;
    }
  else {
      motorPWM = 0;
      backLights = 1;
    } 

   digitalWrite(motorDirectionPin,motorDir);
   analogWrite(motorPWMPin,motorPWM);

    
  //Serial.print(joyY);
  //  Serial.print("  ");
  //  Serial.println(motorPWM);

  steeringAngle = 2000-map(joyX,0,1023,171,853) + (50-map(readA6,0,512,0,200)); //+ map(readA6,340,683,-30,30); //340 to 680 = 1023/3 to 1023*2/3. 10k variable resistor between resistor two 10k resistors
  //Serial.println(readA6);
  //Serial.println(steeringAngle);
  steeringServo.write(steeringAngle);



  //map lights - if front/back light are on && L/R lights on, then 50% flash
  if (A==0 && lightsState!=A && (millis()-buttATime)>50){
    buttATime = millis();
    lights = !lights;
    }
  lightsState = A;

  if (B==0 && rightFlashState!=B && (millis()-buttBTime)>50){
    buttBTime = millis();
    rightFlash = !rightFlash;
    if (rightFlash==1){
      leftFlash = 0;
      }
    }
  rightFlashState = B;

  if (C==0 && hazardState!=C && (millis()-buttCTime)>50){
    buttCTime = millis();
    hazards = !hazards;
    }
  hazardState = C;

  if (D==0 && leftFlashState!=D && (millis()-buttDTime)>50){
    buttDTime = millis();
    leftFlash = !leftFlash;
    if (leftFlash==1){
      rightFlash = 0;
      }
    }
  leftFlashState = D;

  digitalWrite(horn,!Z); //Horn

  }


indicatorlights();
checkSum = 0;

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



void indicatorlights(){
//put lights into their state
//right/left/hazards lights will flash no matter what%
//rightFlash
//hazards

  if (millis()-indicatorLightsTime<1000 && (rightFlash==1 || leftFlash ==1 || hazards==1)){
      digitalWrite(frontLeftPin, ((lights) || (hazards || leftFlash)));
      digitalWrite(frontRightPin, ((lights) || (hazards || rightFlash)));
      digitalWrite(backLeftPin, ((lights || backLights) || (hazards || leftFlash)));
      digitalWrite(backRightPin, ((lights || backLights) || (hazards || rightFlash)));
    }
  else if(millis()-indicatorLightsTime<2000 && (rightFlash==1 || leftFlash ==1 || hazards==1)){
      digitalWrite(frontLeftPin, ((lights) * !(hazards || leftFlash)));
      digitalWrite(frontRightPin, ((lights) * !(hazards || rightFlash)));
      digitalWrite(backLeftPin, ((lights || backLights) * !(hazards || leftFlash)));
      digitalWrite(backRightPin, ((lights || backLights) * !(hazards || rightFlash)));
    }
  else{
      indicatorLightsTime = millis();
      digitalWrite(frontLeftPin, ((lights) || (hazards || leftFlash)));
      digitalWrite(frontRightPin, ((lights) || (hazards || rightFlash)));
      digitalWrite(backLeftPin, ((lights || backLights) || (hazards || leftFlash)));
      digitalWrite(backRightPin, ((lights || backLights) || (hazards || rightFlash)));
    }

}  
