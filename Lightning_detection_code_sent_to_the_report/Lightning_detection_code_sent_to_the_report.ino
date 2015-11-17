/*Date : 
  */

//include for communicating with LCD 16x2 to display data
#include <LiquidCrystal.h>
#include <SPI.h>
#include <AS3935.h>

//definitions
#define LED1 6
#define LED2 7
#define LED3 8
#define LED4 9
#define RELAY A4

#define REFERENCE_TRESHOLD 500  //500mV

int WarningLevel=0;

void printAS3935Registers();

// Function prototype that provides SPI transfer and is passed to
// AS3935 to be used from within library, it is defined later in main sketch.
// That is up to user to deal with specific implementation of SPI
// Note that AS3935 library requires this function to have exactly this signature
// and it can not be member function of any C++ class, which happens
// to be almost any Arduino library
// Please make sure your implementation of choice does not deal with CS pin,
// library takes care about it on it's own
byte SPItransfer(byte sendByte);

// Iterrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;

// First parameter - SPI transfer function, second - Arduino pin used for CS
// and finally third argument - Arduino pin used for IRQ
// It is good idea to chose pin that has interrupts attached, that way one can use
// attachInterrupt in sketch to detect interrupt
// Library internally polls this pin when doing calibration, so being an interrupt pin
// is not a requirement
AS3935 AS3935(SPItransfer,SS,2);


void setup() {
  
  //but while A0 is input A1 is work as output and reference to that one
  pinMode(A0, input); //set A0 pin as input to getting the input of the GND voltage
  pinMode(A1,input);  //set A1 as input to get the input voltage of the plate

  //to collecting data use serial port to collect data
  Serial.begin(9600);

  //Create instance of a LiquidCrystal class(object)
  LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.

  //display project name at the begining
  lcd.setCursor(0,1);
  lcd.print("    Lighning    ");
  lcd.seCursor(0,2);
  lcd.print("   Detection    ")

  delay(2000);
  
  lcd.setCursor(0,1);
  lcd.print("  and Isolate   ");
  lcd.setCursor(0,2);
  lcd.print("   the System   ");
  
  delay(2000);

  //set pins as output for LED indicators
  pinMode(LED1,output);
  pinMode(LED2,output);
  pinMode(LED3,output);
  pinMode(LED4,output);

  //set pins as output to drive the relay
  pinMode(RELAY,output); 

  setupA()
}

void loop() {

  //this indicates that the lighning sensor interrupt trigerd 
  if(AS3935IrqTriggered)
  {
    // reset the flag
    AS3935IrqTriggered = 0;
    // first step is to find out what caused interrupt
    // as soon as we read interrupt cause register, irq pin goes low
    int irqSource = AS3935.interruptSource();
    // returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!
    if (irqSource & 0b0001)
      Serial.println("Noise level too high, try adjusting noise floor");
    if (irqSource & 0b0100)
      Serial.println("Disturber detected");
    if (irqSource & 0b1000)
    {
      // need to find how far that lightning stroke, function returns approximate distance in kilometers,
      // where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
      // everything in between is just distance in kilometers
      int strokeDistance = AS3935.lightningDistanceKm();
      if (strokeDistance == 1)
        Serial.println("Storm overhead, watch out!");
        WarningLevel = 1;
      if (strokeDistance == 63)
        Serial.println("Out of range lightning detected.");
      if (strokeDistance < 63 && strokeDistance > 1)
      {
        Serial.print("Lightning detected ");
        Serial.print(strokeDistance,DEC);
        Serial.println(" kilometers away.");
      }
    }
  }
  else{
    if(WarningLevel==2) Warning Level = 2;
    if(WarningLevel==1) Warning Level = 0;
  }

  lcd.setCursor(0,1);
  lcd.print("Warning Level -");
  lcd.setCursor(16,1);  
  lcd.print(Warninglevel);
  
  int tmpVoltage = readVoltage();
  
  lcd.setCursor(0,1);
  lcd.print("Voltage : ");
  lcd.setCursor(11,2);
  lcd.print(tmpVoltage);
  
  if(tmpVoltage > REFERENCE_TRESHOLD){

    //both sensors are triggered
    if(WarningLevel==1) Warning Level = 3;
    //only sensor 2 is triggered
    else WarningLevel =  2; 
  }
  else{
    if(WarningLevel==1) Warning Level = 1;
    if(WarningLevel==2) Warning Level = 0;
  }

  switch(WarningLevel){
     case : 1
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,LOW);
      digitalWrite(LED1,LOW);
      
      digitalWrite(RELAY,LOW); 
     break;
     case : 2
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,LOW);
      
      digitalWrite(RELAY,LOW); 
     break;
     case : 3
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,HIGH);
      
      digitalWrite(RELAY,HIGH); 
     break;
     default   
      digitalWrite(LED1,HIGH);
      digitalWrite(LED1,LOW);
      digitalWrite(LED1,LOW);
      digitalWrite(LED1,LOW);
      
      digitalWrite(RELAY,LOW);  
  }
}

void setupAS3935(){
  // first begin, then set parameters
  SPI.begin();
  // NB! chip uses SPI MODE1
  SPI.setDataMode(SPI_MODE1);
  // NB! max SPI clock speed that chip supports is 2MHz,
  // but never use 500kHz, because that will cause interference
  // to lightning detection circuit
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  // and chip is MSB first
  SPI.setBitOrder(MSBFIRST);
  // reset all internal register values to defaults
  AS3935.reset();
  // and run calibration
  // if lightning detector can not tune tank circuit to required tolerance,
  // calibration function will return false
  if(!AS3935.calibrate())
    Serial.println("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");

  // since this is demo code, we just go on minding our own business and ignore the fact that someone divided by zero

  // first let's turn on disturber indication and print some register values from AS3935
  // tell AS3935 we are indoors, for outdoors use setOutdoors() function
  AS3935.setIndoors();
  // turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()
  AS3935.enableDisturbers();
  printAS3935Registers();
  AS3935IrqTriggered = 0;
  // Using interrupts means you do not have to check for pin being set continiously, chip does that for you and
  // notifies your code
  // demo is written and tested on ChipKit MAX32, irq pin is connected to max32 pin 2, that corresponds to interrupt 1
  // look up what pins can be used as interrupts on your specific board and how pins map to int numbers

  // ChipKit Max32 - irq connected to pin 2
  //attachInterrupt(1,AS3935Irq,RISING);
  // uncomment line below and comment out line above for Arduino Mega 2560, irq still connected to pin 2
  attachInterrupt(0,AS3935Irq,RISING);
  }

  void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  Serial.print("Noise floor is: ");
  Serial.println(noiseFloor,DEC);
  Serial.print("Spike rejection is: ");
  Serial.println(spikeRejection,DEC);
  Serial.print("Watchdog threshold is: ");
  Serial.println(watchdogThreshold,DEC);  
}

// this is implementation of SPI transfer that gets passed to AS3935
// you can (hopefully) wrap any SPI implementation in this
byte SPItransfer(byte sendByte)
{
  return SPI.transfer(sendByte);
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935Irq()
{
  AS3935IrqTriggered = 1;
}

void int readVoltage(){
  int tmp;
  pinMode(A1,output);
  pinMode(A0,input);
  digitalWrite(A1,LOW); //set reference as A1 and then get the input of A0
  tmp = analogRead(A0);
  //wait for 20 miliseconds
  delay(20);
  tmp = analogRead(A0);
  //if this is zero that means our reference should change
  if (tmp > 0){
    return tmp*5000/1023;
  }
  else{
    pinMode(A1,input);
    pinMode(A0,output);
    digitalWrite(A0,LOW);  
    tmp = analogRead(A1);
    //wait for 20 miliseconds
    delay(20);
    tmp = analogRead(A1);
    //if this is zero that means our reference should change
    if (tmp > 0){
      return (-1)*tmp*5000/1023;
    } 
  }
  return 0;
}

