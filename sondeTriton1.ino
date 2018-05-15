
//#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include <LowPower.h>
#include <string.h>
#include <VirtualWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//branchement de la sonde HTU21
// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin (A5 on UNO)
// Connect SDA to I2C data pin (A4 on UNO)

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

char message[VW_MAX_MESSAGE_LEN];

/* Utilisation du capteur Ultrason HC-SR04 */

// définition des broches utilisées 
int trig = 6; 
int echo = 7; 
long lecture_echo; 
long cm;

// param pH meter 

#define SensorPin 0          //pH meter Analog output to Arduino Analog Input 0
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;



// paramétrage écran LCD

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

// init entrée PIR

int pirInput = 2; 
int compteur = 0;
  

void setup() {
  Serial.begin(9600);

  vw_set_tx_pin(12);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(1000);   // Bits per sec

  //Serial.println("HTU21D-F test");

  if (!htu.begin()) {
    //Serial.println("Couldn't find sensor!");
    while (1);
  }

//config sonde distance HCSR04

  pinMode(trig, OUTPUT); 
  digitalWrite(trig, LOW); 
  pinMode(echo, INPUT); 

// init LCD 
  lcd.init();
}
/// récupération de la température et de l'hygrométrie du meuble 

void loop() {
  float TAir = htu.readTemperature();
  float HAir = htu.readHumidity();
  //Serial.print("Temp: "); Serial.print(TAir);
  //Serial.print("\t\tHum: "); Serial.println(HAir);
  delay(1500);

//récupération de la hauteur d'eau osmosée

  digitalWrite(trig, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trig, LOW); 
  lecture_echo = pulseIn(echo, HIGH); 
  cm = lecture_echo / 58; 
  //Serial.print("Distance en cm : "); 
  //Serial.println(cm); 
  delay(1000);

// récupération pH eau de mer
delay (2000);
 for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(SensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+= buf[i];
  
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  //Serial.print("    pH:");  
  //Serial.print(phValue,2);
  //Serial.println(" ");

// recup température aquarium via sonde dallas
sensors.requestTemperatures(); // Send the command to get temperature readings

float Taqua = sensors.getTempCByIndex(0); // Why "byIndex"?

// affichage lcd

int val = 0 ;
  String ligne1 = String("");
  ligne1 += "T:";
  ligne1 += String(TAir,1);
  ligne1 += " H:";
  ligne1 += String(HAir,0);
  String ligne2 = String("");
  ligne2 += "TA:";
  ligne2 += String(Taqua,1);
  ligne2 += " TOs:";
  ligne2 += String(0.00,1);
  String ligne3 = String("");
  ligne3 += "D:";
  ligne3 += cm;
  ligne3 += " pH:";
  ligne3 += String(phValue,1);

int tour = 0;

delay(500);

  String msg_string = String("");
  msg_string += "399,";
  msg_string += HAir;
  msg_string += ",";
  msg_string += TAir;
  msg_string += ",";
  msg_string += Taqua; 
  msg_string += ",";
  msg_string += phValue;
  msg_string += ",";

   msg_string += cm;
   msg_string += ",";
   msg_string += "0";
   msg_string += ",";


  Serial.println(msg_string);
// envoi des données

  char* msg = (char*)malloc(msg_string.length());
  msg_string.toCharArray(msg, msg_string.length());

  strcpy (message, msg);
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
delay(500);
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
delay(500);

free(msg);

delay (1000);


while(tour <= 2){
int t = 0;
while (t <= 10){
t++;
delay (150);
val = digitalRead(pirInput);  // read input value
if (val == HIGH){
lcd.backlight();
}
}

delay(1000);
  tour++;
  // page 1 
  lcd.init();
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print(ligne1);
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(ligne2);

t = 0;
while (t <= 10){
t++;
delay (150);
val = digitalRead(pirInput);  // read input value
if (val == HIGH){
lcd.backlight();
}
}
  
  // page 2
  lcd.init();
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print(ligne2);
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(ligne3);


t = 0;
while (t <= 10){
t++;
delay (150);
val = digitalRead(pirInput);  // read input value
if (val == HIGH){
lcd.backlight();
}
}
  
  // page 0
  lcd.init();
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print(ligne3);
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(ligne1);

t = 0;
while (t <= 10){
t++;
delay (150);
val = digitalRead(pirInput);  // read input value
if (val == HIGH){
lcd.backlight();
}
}
  }

    lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print("Repos...");
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("...en cours...");
  lcd.noBacklight();



delay(10000);
  
  }


