//*----------------------------------
//Emphebion CheckLock program
//Version 1.0
//22-7-2019
//-----------------------------------*

#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <Servo.h>

#define DEBUG 0

/* Pin Definitions */
#define STAT_LED_RED 6
#define STAT_LED_GREEN 5
#define STAT_LED_BLUE 3

// Lock signal pin: D12
#define LOCK_PIN 9

//Signaal_lichtkist: D9
#define GAME_LED1 7  //Front Left
#define GAME_LED2 8
#define GAME_LED3 10
#define GAME_LED4 11 //Front Right

//RFID input: D13
#define RFID_RX 13
#define RFID_IRQ_PIN 2

#define Fadespeed 1200
//Optioneel A0 t/m A5
#define BUT1 A1 //Front Left
#define BUT2 A2
#define BUT3 A3
#define BUT4 A4 //Front Right

// Integers for 

int keyIndex = 23;                                              // starting position of Posible new keys
bool hacking = false;
int fails = 0;
int successes = 0;
int prevSuccesses = 0;
int prevFails = 0;
int code[4];
int inputCode[4];
int amount;
int input[4];
bool codeCheck;

// Difficulty parameters
int difficulty = 28;                      // Number of successes required
unsigned long hackTimeout = 1000;
unsigned long stepTimeout = 6000;
int stepOffset = 2500;                   // Minimum wait time between steps (10s)
int stepRange = 45;                       // Variable additional wait time between steps (up to 60*100ms = 6s)

Servo lock;
int pos = 0;
static const int locked = 0;
static const int unlocked = 180;

//create a Serial object RFID
SoftwareSerial RFID = SoftwareSerial(RFID_RX, 13);        // RFID object to control the read
uint8_t buff[28];
char c_buff[28];
byte tag_buf[4];
uint8_t* buffer_at = buff;
uint8_t* buffer_end = buff + sizeof(buff);
volatile bool rdyRFID = false;
uint32_t rfidTag = 0;

unsigned long rfidReadTimeout = 1000; // Time before another Tag can be read (1 s)
unsigned long lockReset = 5000;    // Time before another Tag can be read (200 ms)

unsigned long rfidReadTimer = 0;
unsigned long lockTimer = 0;
unsigned long hackTimer = 0;
unsigned long stepTimer = 0;
unsigned long timer = 0;

// 12 valid RFID tags each of 12 chars in 13 char array terminated by null char
uint32_t tag01 = 0; //eerste 50 tags zijn sleutels
uint32_t tag02 = 0;
uint32_t tag03 = 0;
uint32_t tag04 = 0;
uint32_t tag05 = 0;
uint32_t tag06 = 0;
uint32_t tag07 = 0;
uint32_t tag08 = 0;
uint32_t tag09 = 0;
uint32_t tag10 = 0;
uint32_t tag11 = 0;
uint32_t tag12 = 0;
uint32_t tag13 = 0;
uint32_t tag14 = 0;
uint32_t tag15 = 0;
uint32_t tag16 = 0;
uint32_t tag17 = 0;
uint32_t tag18 = 0;
uint32_t tag19 = 0;
uint32_t tag20 = 0;
uint32_t tag21 = 0;
uint32_t tag22 = 0;
uint32_t tag23 = 0;
uint32_t tag24 = 0;
uint32_t tag51 = 14490303; //van 51 tot 100 zijn Onklaar Makers
uint32_t tag52 = 0; 
uint32_t tag53 = 0;
uint32_t tag54 = 0;
uint32_t tag55 = 0;
uint32_t tag56 = 0;
uint32_t tag57 = 0;
uint32_t tag58 = 0;
uint32_t tag59 = 0;
uint32_t tag60 = 0;
uint32_t tag61 = 0;
uint32_t tag62 = 0;
uint32_t tag63 = 0;
uint32_t tag64 = 0;
uint32_t tag65 = 0;
uint32_t tag66 = 0;
uint32_t tag67 = 0;
uint32_t tag68 = 0;
uint32_t tag69 = 0;
uint32_t tag70 = 0;
uint32_t tag71 = 0;
uint32_t tag72 = 0;
uint32_t tag73 = 0;
uint32_t tag74 = 0;
uint32_t tag101 = 13426395; // Van 101 tot 105 zijn spelleiders
uint32_t tag102 = 13420919;
uint32_t tag103 = 13421961;
uint32_t tag104 = 0;
uint32_t tag105 = 0;

// Array of pointers to valid tags also stored in flash memory  
uint32_t Sleutel[] = {tag01,tag02,tag03,tag04,tag05,tag06,tag07,tag08,tag09,tag10,tag11,tag12,tag13,tag14,tag15,tag16,tag17,tag18,tag19,tag20,tag21,tag22,tag23,tag24};
                                 
uint32_t Onklaarmaken[] = {tag51,tag52,tag53,tag54,tag55,tag56,tag57,tag58,tag59,tag60,tag61,tag62,tag63,tag64,tag65,tag66,tag67,tag68,tag69,tag70,tag71,tag72,tag73,tag74};
                                      
uint32_t Spelleider[] = {tag101,tag102,tag103,tag104,tag105};


void setup(){
  Serial.begin(9600);
  RFID.begin(9600);
  
  pinMode(STAT_LED_RED, OUTPUT);
  pinMode(STAT_LED_GREEN, OUTPUT);
  pinMode(STAT_LED_BLUE, OUTPUT);
  pinMode(GAME_LED1, OUTPUT);
  pinMode(GAME_LED2, OUTPUT);
  pinMode(GAME_LED3, OUTPUT);
  pinMode(GAME_LED4, OUTPUT);
  
  pinMode(BUT1,INPUT);
  pinMode(BUT2,INPUT);
  pinMode(BUT3,INPUT);
  pinMode(BUT4,INPUT);

  attachInterrupt(digitalPinToInterrupt(RFID_IRQ_PIN), rfidRead, FALLING);
    
  setSTATLED(200,200,0);
  setGameLEDs(0,0,0,0);
  
  analogWrite(BUT1,LOW);
  analogWrite(BUT2,LOW);
  analogWrite(BUT3,LOW);
  analogWrite(BUT4,LOW);
  
  lock.attach(LOCK_PIN);
  lock.write(180);
  Serial.println("Lock armed and open");
}

void loop(){
  if(DEBUG == 1)
    debugMode();
  else{
    timer = millis();
    
    if(hacking){
      lockTimer = timer;
      hacking = hackLockTick(timer);
    }
    else if(rdyRFID){
      detachInterrupt(digitalPinToInterrupt(RFID_IRQ_PIN));
      rdyRFID = false;
      lockTimer = timer;
      rfidTag = readRFIDTag(tag_buf);
      Serial.print("Tag = ");
      Serial.println(rfidTag);
      skillSelect(findSkill(rfidTag));
    }
    else{
      if(RFID.available())
        RFID.read();
    }

    if((timer - lockTimer) >= lockReset){
//      Serial.println("skillTimer Timeout reached");
      attachInterrupt(digitalPinToInterrupt(RFID_IRQ_PIN), rfidRead, FALLING);
      lockTimer = timer;
      setSTATLED(0,0,0);
    }
  }
}

//***************************************************************************//
// Find Skill by Tag                                                         //
//***************************************************************************//
int findSkill(uint32_t tag){
  for(int i = 0; i<24; i++){
    if(i<5){
      if(tag == Spelleider[i])
        return 1;
    }
    if(tag == Sleutel[i])
      return 1;
  }
  for(int i = 0; i<24; i++){
    if(tag == Onklaarmaken[i])
      return 2;
  }
  return 0;
}

//***************************************************************************//
// Select skill to run                                                       //
//***************************************************************************//
void skillSelect(int s){
  switch(s){
    case 0:
      //RED LED
      hacking = false;
      setSTATLED(200,0,0);
      break;
    case 1:
      // Has KEY
      hacking = false;
      setSTATLED(0,200,0);
      changeLockPos();
      break;
    case 2:
      // ONKLAARMAKEN
      codeCheck = false;
      prevSuccesses = 0;
      successes = 0;
      prevFails = 0;
      fails = 0;
      hackStartLEDs();
      setSTATLED(0,0,0);
      hacking = true;
      hackTimer = millis();
      stepTimer = millis();
      setStepTimeout();
      resetValues(code,inputCode,input);
      createCode(code);
      break;
    default:
      // DO NOTHING
      hacking = false;
      setSTATLED(0,0,0);
      break;
  }
}

//***************************************************************************//
// Onklaarmaken skill                                                        //
//***************************************************************************//
bool hackLockTick(unsigned long currentTime){
  //Loop that determines the periode between inputs
  Serial.print("currentTime = ");
  Serial.print(currentTime);
  Serial.print(" ; ");
  Serial.print("stepTimer = ");
  Serial.print(stepTimer);
  if((currentTime - stepTimer) <= stepTimeout){
    maxInput(input,inputCode);
    codeCheck = checkCode(code,inputCode);
    if(codeCheck && (successes <= prevSuccesses)){
      successes = prevSuccesses + 1;
      setGameLEDs(100,100,100,100);
    }
    // FUTURE WORK: break if inputCode can no longer match code
    else if(!codeCheck && (successes <= prevSuccesses)){
      if((currentTime - hackTimer) >= hackTimeout){
        if(fails <= prevFails)
          fails = prevFails + 1;
        blinkGameLEDs();
      }
    }
    else if(!codeCheck)
      successes = prevSuccesses;
  }
  else{
    if(successes >= difficulty){
      createKey(rfidTag);
      setSTATLED(0,200,0);
      changeLockPos();
      setGameLEDs(0,0,0,0);
      return false;
    }
    else if(fails >= 3){
      setSTATLED(200,0,0);
      setGameLEDs(0,0,0,0);
      return false;
    }
    resetValues(code,inputCode,input);
    createCode(code);
    codeCheck = false;
    prevSuccesses = successes;
    prevFails = fails;
    hackTimer = currentTime;
    stepTimer = currentTime;
    setStepTimeout();
  }
  return true;
}

//***************************************************************************//
// Functions for hack Code                                                   //
//***************************************************************************//
void createCode(int* c){
  int temp = random(3);
  c[temp] = 1;
  setGameLEDs(100*c[0],100*c[1],100*c[2],100*c[3]);
}

void resetValues(int* c, int* cc, int* in){
    for(int i = 0; i<=3; i++){
      c[i] = 0;
      cc[i] = 0;
      in[i] = 0;
    }
}

bool checkCode(int* c, int* cc){
  bool check = true;
  for(int i = 0; i<=3; i++){
    if(c[i] != cc[i])
      check = false;
  }
  return check;
}

void createKey(uint32_t tag){
  Sleutel[keyIndex] = tag;
  keyIndex--;
}

void setStepTimeout(){
  stepTimeout = 100*random(stepRange)+stepOffset;
}

void changeLockPos(){
  if(pos == locked)
    pos = unlocked;
  else
    pos = locked;

  lock.write(pos);
  delay(500);
}

//***************************************************************************//
// Read button inputs                                                        //
//***************************************************************************//
void maxInput(int* data, int* c){
  int in[] = {analogRead(BUT1),analogRead(BUT2),analogRead(BUT3),analogRead(BUT4)};
  for(int i = 0; i<=3; i++){
    if(data[i] < in [i] && in[i] > 100){
      data[i] = in[i];
      c[i] = 1;
    }
  }
}

//***************************************************************************//
// Read Presented Tag                                                        //
//***************************************************************************//
uint32_t readRFIDTag(byte* data){
  uint32_t result = 0;
  delay(350);
  // Vul de buffer
  buffer_at = buff;
  while ( buffer_at < buffer_end )
    *buffer_at++ = RFID.read();

  // Reset de buffer pointer om het uitlezen makkelijker te maken
  buffer_at = buff;
    
  // Skip the preamble
  ++buffer_at;
  // Begin de sommatie van de Checksum
  uint8_t checksum = rfidGetNext();
  
  // De tag bestaat uit nog 4 waardes
  for(int i = 0; i <= 3; i++){
    // Pak de volgende waarde
    uint8_t value = rfidGetNext();
    data[i] = value;
    result <<= 8;
    result |= value;
        
    // Xor de waarde met de checksum
    checksum ^= value;
  }
  
  // Vraag de checksum op
  uint8_t data_checksum = rfidGetNext();
  
  // Controleer de checksum
  if ( checksum == data_checksum ){
    Serial.println("OK");
  }
  else{
    Serial.println("CHECKSUM FAILED");
    result = 0;
  }

  while(RFID.available()){
    RFID.read();
  }
  
  return result;
}

//***************************************************************************//
// RFID Functies                                                             //
//***************************************************************************//
uint8_t rfidGetNext(void)
{
  // sscanf needs a 2-byte space to put the result but we
  // only need one byte.
  uint16_t result;
 
  // Working space to assemble each byte
  static char byte_chars[3];
  
  // Pull out one byte from this position in the stream
  snprintf(byte_chars,3,"%c%c",buffer_at[0],buffer_at[1]);
  sscanf(byte_chars,"%x",&result);
  buffer_at += 2;
  
  return static_cast<uint8_t>(result);
}

//***************************************************************************//
// INTERUPT SERVICE ROUTINES                                                 //
//***************************************************************************//
void rfidRead(void){
  // Lees alleen een waarde als er niet nog een verwerkt word
  if ( !rdyRFID ){
//    Serial.println("Interrupt");
    rfidReadTimer = millis();
    rdyRFID = true;                            // Zet de rdyRFID flag
  }
  else
    RFID.flush();
}

//***************************************************************************//
// Set LED Functions                                                         //
//***************************************************************************//
void setSTATLED(int r, int g, int b){
  digitalWrite(STAT_LED_RED,r);
  digitalWrite(STAT_LED_GREEN,g);
  digitalWrite(STAT_LED_BLUE,b);
}

void setGameLEDs(int n1, int n2, int n3, int n4){
  digitalWrite(GAME_LED1,n1);
  digitalWrite(GAME_LED2,n2);
  digitalWrite(GAME_LED3,n3);
  digitalWrite(GAME_LED4,n4);
}

void blinkGameLEDs(){
  setGameLEDs(100,100,100,100);
  delay(100);
  setGameLEDs(0,0,0,0);
  delay(100);
}

void hackStartLEDs(){
  for(int i = 0; i<3; i++){
    setGameLEDs(100,100,100,100);
    delay(200);
    setGameLEDs(0,0,0,0);
    delay(800);
  }
}

//***************************************************************************//
// DEBUG MODE                                                                //
//***************************************************************************//
void debugMode(){
  int b1 = analogRead(BUT1);
  int b2 = analogRead(BUT2);
  int b3 = analogRead(BUT3);
  int b4 = analogRead(BUT4);
  Serial.print("Button values: ");
  Serial.print(b1);
  Serial.print(", ");
  Serial.print(b2);
  Serial.print(", ");
  Serial.print(b3);
  Serial.print(", ");
  Serial.println(b4);
  if(b1>500)
    digitalWrite(GAME_LED1,b1/4);
  else
    digitalWrite(GAME_LED1,0);
  if(b2>500)
    digitalWrite(GAME_LED2,b2/4);
  else
    digitalWrite(GAME_LED2,0);
  if(b3>500)
    digitalWrite(GAME_LED3,b3/4);
  else
    digitalWrite(GAME_LED3,0);
  if(b4>500)
    digitalWrite(GAME_LED4,b4/4);
  else
    digitalWrite(GAME_LED4,0);

  lock.write(180);
  delay(3000);
  lock.write(0);
  delay(3000);

  if(rdyRFID){
    Serial.print("Tag found: ");
    Serial.println(readRFIDTag(tag_buf));
    Serial.println(" ");
    setSTATLED(0,150,0);
    rdyRFID = false;
  }
  else
    setSTATLED(150,0,0);
}
