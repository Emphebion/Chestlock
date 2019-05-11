//*----------------------------------
//Emphebion RFID programma
//Versie 0.3
//10-5-2013
//-----------------------------------*

#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

/* Pin Definitions */
//RGB Strip 1 (knop_: D3(rood), D5(groen), D6(blauw)
#define Rood_1 6
#define Groen_1 5
#define Blauw_1 3

// Lock signal pin: D12
#define LOCK_PIN 12

//Signaal_lichtkist: D9
#define Signaal_Lichtkist 9

//RFID input: D13
#define RFID_RX 13
#define RFID_TX 2

#define Fadespeed 1200
//Optioneel A0 t/m A5
#define SECRET_KEY A0

// Integers for 

int key_gen = 23;                                              // starting position of Posible new keys
int key_created = 255;                                         // temp value for Lockpicking

//create a Serial object RFID
SoftwareSerial RFID = SoftwareSerial(RFID_RX, RFID_TX);        // RFID object to control the read
char Read_String[14];                                          // String that contians the initial
char dump[14];                                                 // Dump array to slowly decrease the buffer as a time base
char Read_Tag[13] = {'\0'};                                    // String containing the eventual tag ID (with '0' terminator)                              
byte ls;

volatile unsigned long lock_timer = 0;                         // Lock status; 1 = Open, 0 = Closed
long lock_delay = 3000000;
int pos;                                                       // Will contain the position of the servo

// 12 valid RFID tags each of 12 chars in 13 char array terminated by null char
char tag01[] = "000000000001\0"; //eerste 50 tags zijn sleutels
char tag02[] = "000000000002\0";
char tag03[] = "000000000003\0";
char tag04[] = "000000000004\0";
char tag05[] = "000000000005\0";
char tag06[] = "000000000006\0";
char tag07[] = "000000000007\0";
char tag08[] = "000000000008\0";
char tag09[] = "000000000009\0";
char tag10[] = "000000000010\0";
char tag11[] = "000000000011\0";
char tag12[] = "000000000012\0";
char tag13[] = "000000000013\0";
char tag14[] = "000000000014\0";
char tag15[] = "000000000015\0";
char tag16[] = "000000000016\0";
char tag17[] = "000000000017\0";
char tag18[] = "000000000018\0";
char tag19[] = "000000000019\0";
char tag20[] = "000000000020\0";
char tag21[] = "000000000021\0";
char tag22[] = "000000000022\0";
char tag23[] = "000000000023\0";
char tag24[] = "000000000024\0";
char tag51[] = "1800CC7E67CD\0"; //van 51 tot 100 zijn Onklaar Makers
char tag52[] = "1800CCC9776A\0"; 
char tag53[] = "1800CCD3B2B5\0";
char tag54[] = "1800CCD17C79\0";
char tag55[] = "000000000055\0";
char tag56[] = "000000000056\0";
char tag57[] = "000000000057\0";
char tag58[] = "000000000058\0";
char tag59[] = "000000000059\0";
char tag60[] = "000000000060\0";
char tag61[] = "000000000061\0";
char tag62[] = "000000000062\0";
char tag63[] = "000000000063\0";
char tag64[] = "000000000064\0";
char tag65[] = "000000000065\0";
char tag66[] = "000000000066\0";
char tag67[] = "000000000067\0";
char tag68[] = "000000000068\0";
char tag69[] = "000000000069\0";
char tag70[] = "000000000070\0";
char tag71[] = "000000000071\0";
char tag72[] = "000000000072\0";
char tag73[] = "000000000073\0";
char tag74[] = "000000000074\0";
char tag101[] = "1800CCD49A9A\0"; // Van 101 tot 105 zijn spelleiders
char tag102[] = "000000000102\0";
char tag103[] = "000000000103\0";
char tag104[] = "000000000104\0";
char tag105[] = "000000000105\0";

// Replace "??????????01\0" etc with 12 chars of each valid RFID tag - must include null char


// Array of pointers to valid tags also stored in flash memory  
char *Sleutel[] = {tag01,tag02,tag03,tag04,tag05,tag06,tag07,tag08,tag09,tag10,tag11,tag12,tag13,tag14,tag15,tag16,tag17,tag18,tag19,tag20,tag21,tag22,tag23,tag24};
                                 
char *OnklaarMaken[] = {tag51,tag52,tag53,tag54,tag55,tag56,tag57,tag58,tag59,tag60,tag61,tag62,tag63,tag64,tag65,tag66,tag67,tag68,tag69,tag70,tag71,tag72,tag73,tag74};
                                      
char *Spelleider[] = {tag101,tag102,tag103,tag104,tag105};


void setup(){
  Serial.begin(9600);
  RFID.begin(9600);
  pinMode(LOCK_PIN,OUTPUT);
  pinMode(Rood_1, OUTPUT);
  pinMode(Groen_1, OUTPUT);
  pinMode(Blauw_1, OUTPUT);
  pinMode(SECRET_KEY,INPUT);
  analogWrite(SECRET_KEY,LOW);
  digitalWrite(LOCK_PIN,0);
  analogWrite(Blauw_1, 0);
  analogWrite(Rood_1,255);
  analogWrite(Groen_1,0);
  Serial.println("Lock armed");
}

void loop(){
  if(SECRET_KEY == HIGH){
    analogWrite(Groen_1,255);
    analogWrite(Rood_1,0);
    while(micros() - lock_timer <= lock_delay){
      digitalWrite(LOCK_PIN,HIGH);
    }
    digitalWrite(LOCK_PIN,LOW);
  }
  if(RFID.available()){                                         // When something is offered to the RFID reader
    ls = RFID.readBytesUntil(13,Read_String,sizeof(Read_String));       // Read the offered RFID tag
    for(int i = 0; i < sizeof(Read_Tag)-1; i++){                // Remove the first and last byte from the bit stream
      Read_Tag[i] = Read_String[i+1];
    }
    Serial.println(Read_Tag);                                   // Print the tag for test and readout purposes
    compareToSleutel();                                         // Check if the tag is a key
    compareToOnklaarMaken();                                    // Check if the tag is valid for Lockpicking
    compareToSpelleider();                                      // Check if the tag is from a Gamemaster
  }
  else
    digitalWrite(LOCK_PIN,LOW);
    analogWrite(Rood_1,255);
    analogWrite(Groen_1,0);
}

void compareToSleutel(){
  for (int i = 0; i < 24; i++){
    if (strcmp(Sleutel[i],Read_Tag)==0){
      Serial.println("Sleutel gevonden!");
      analogWrite(Groen_1, 255);                              // Set the RGB led to green only
      analogWrite(Rood_1, 0);
      lock_timer = micros();
      while(micros() - lock_timer <= lock_delay){
        digitalWrite(LOCK_PIN,HIGH);
      }
      digitalWrite(LOCK_PIN,LOW);
      for(int j = 0; j < sizeof(Read_Tag)-1;j++){
        Read_Tag[j] = '0'; // Clear contents of Read_Tag String
      }
    }
  }
}

void compareToOnklaarMaken()
{
  for (int i = 0; i < 24; i++) // Iterate through each of the 24 valid tags
  {
    if (strcmp(OnklaarMaken[i],Read_Tag)==0){
      Serial.println("Breaking Lock");
      Serial.println(Read_Tag);
      delay(500);
      while(RFID.available()>0){
        Serial.println("Still breaking lock");
        for(int Groen_1_Val = 1; Groen_1_Val < 256 ; Groen_1_Val++){
          Serial.println("Breaking lock part 1");
          analogWrite(Groen_1, Groen_1_Val);
          RFID.readBytes(dump,10);
          delay(Fadespeed);
          if(RFID.available()==0){
            break;}
        }
        if(RFID.available()==0){
            break;}
        for(int Rood_1_Val = 255; Rood_1_Val > 0 ; Rood_1_Val--){
          Serial.println("Breaking lock part 2");
          analogWrite(Rood_1, Rood_1_Val);
          key_created = Rood_1_Val;
          RFID.readBytes(dump,10);
          delay(Fadespeed);
          if(RFID.available()==0){
            break;}
        }
        if(RFID.available()==0){
            break;}
        if(key_created <= 1){
          Serial.println("Lock opened");
          Sleutel[--key_gen] = OnklaarMaken[i];
          digitalWrite(Signaal_Lichtkist,HIGH);
          key_created = 255;
          while(micros() - lock_timer <= lock_delay){
            digitalWrite(LOCK_PIN,HIGH);
          }
          digitalWrite(LOCK_PIN,LOW);
        }
        for(int j = 0; j < sizeof(Read_Tag)-1;j++){
          Read_Tag[j] = '0'; // Clear contents of Read_Tag String
        }
        break;
      }
      RFID.flush();
    }  
  }
}

void compareToSpelleider(){
  for (int i = 0; i < 5; i++){
    if (strcmp(Spelleider[i],Read_Tag)==0){
      Serial.println("Spelleider gevonden!");
      analogWrite(Groen_1, 255);                              // Set the RGB led to green only
      analogWrite(Rood_1, 0);
      lock_timer = micros();
      while(micros() - lock_timer <= lock_delay){
        digitalWrite(LOCK_PIN,HIGH);
      }
      digitalWrite(LOCK_PIN,LOW);
      for(int j = 0; j < sizeof(Read_Tag)-1;j++){
        Read_Tag[j] = '0'; // Clear contents of Read_Tag String
      }
    }
  }
}
