/*  Desk Plant Watering System 
 * 
 *  Be sure to look through the settings and test your sensor value on 
 *  your plant and make adjustments as needed. Different soils and plants 
 *  may require modifications to the variables below.
 */
 
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

//#define servoWaterUsed //comment out if servo is not used.
const bool debug = 1; //set to 1 to enable serial debugging

/***** pin assignments *****/
//const int LEDPIN         = 6;
//const int NUMPIXELS      = 2;
const int sensorPin      = A4; //sensor AD 4
const int sensorEn       = 4;  // pin para habilitar en sensor
const int setButton      = 8; 
const int waterButton    = 9;
const int pumpPin        = 2;  
const int servoPowerPin  = 5;
const int servoPin       = A5;

//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

#ifdef servoWaterUsed
  Servo waterServo;
#endif

/***** Global Constants *****/
//color values = switch-case value
const int flashRed = 2;
const int flashGreen = 3;
const int green=4;
const int blue =5;
const int amber =6;
const int led1 = 0;   
const int led2 = 1;   

/***** Global Variables *****/
float filteredMoisture =0; 
int moistureValue = 0;  //initialize at max value to keep the water from turning upon power-up
int prevMoisture = 600; //this will initialize the beginning average higher - water won't start until it gets a chance to settle
int sensorValue = 0;  // variable to store the value coming from the sensor
int attempt = 0;      //variable used to count how many times the plant has tried to water
int flashDelay = 250; //delay used for flashing the led
int moistureMax = 1000; //moisture value mapped from 0 to this value
int rawMoistureMax = 600; //max raw value used to scale sensor reading
int rawMoistureMin = 200; //min raw value  used to scale sensor reading
int menuPressTime=3000; //time in milliseconds the button needs to be pressed before resetting moisture value 
int maxServoAngle = 30; //servo starts at 90 and will move to this position in both directions
int servoCenter = 90; //center position for the servo

/***** Boolean Flags *****/
boolean waterNeeded=false;
boolean waterOut=false;
boolean firstPress=true;
boolean waterButtonState=true;

/***** Timer Variables *****/
unsigned long buttonStartTime = 0;
unsigned long moistureTimer = 0;
unsigned long waterTimer = 0;

/************ Settings ***************/
unsigned long moistureCheckTime = 10000; //time in milliseconds between moisture checks
unsigned long waterCheckTime = 120000;//time in milliseconds between waterings - when needed
int flowTime = 10000; //how long the pump will run each cycle/attempt
int maxAttempts =15;   //max number of times the water pump will run before the 'no water' shut off will trigger

/***** Custom Plant-dependant Variables *****/
int desiredMoisture = 750; // this can be changed by holding the button (set moisture function)
int moistureDiff = 10;    //pump will run when moisture reading is less than desired moisture - this value
void setup() 
{
  pinMode(pumpPin, OUTPUT); 
  pinMode(servoPowerPin, OUTPUT);
  pinMode(setButton,INPUT_PULLUP);
  pinMode(waterButton,INPUT_PULLUP);
    
  pixels.begin(); //initialize neopixels
  pixels.setPixelColor(led1, pixels.Color(0,0,0)); //Turn LED off
  pixels.setPixelColor(led2, pixels.Color(100,100,100)); //Set LED2 to White
  pixels.show();
  moistureTimer=millis(); //Begin moisture timer at current millis value
  waterTimer=millis();  //Begin water timer at current millis value
  checkMoisture();  //get first moisture reading
  #ifdef servoWaterUsed
    waterServo.attach(servoPin);
    digitalWrite(servoPowerPin,HIGH);  //turn on survo
    waterServo.write(servoCenter); //rotate servo to the center position (90)
    delay(1000); //give the servo time to move to the center.
    digitalWrite(servoPowerPin,LOW); //turn off servo
  #endif
  if(debug) Serial.begin(9600); //turn on serial for debugging or monitoring values
  digitalWrite(pumpPin,LOW);
}

void loop() 
{
  //check moisture if the set amount of time has passed
  if(millis()-moistureTimer>=moistureCheckTime)
  {
    checkMoisture();
    moistureTimer=millis();//reset moisture timer to millis;
  }
  //If left button is pressed water the plant
  if(digitalRead(waterButton)==!waterButtonState)   
  {
    water();
    waterButtonState=!waterButtonState;
  }
  if(digitalRead(setButton)==LOW)  //if right button is pressed reset the water out alarm.  If held, reset desired moisture level to current moisture

  {
    if(firstPress==true) 
    {
      waterOut=false; //reset the waterOut alarm
      attempt=0;      //reset watering attempt count
      setLED2(amber); 
      buttonStartTime=millis(); //Start timer to count how long the button is held
      firstPress=false; //set flag to skip this if statement until after the button has been released
    }
    //if button is held for more than menuPressTime (in milliseconds) reset desired moisture value to 
current moisture
    else if(digitalRead(setButton)==LOW && (millis()-buttonStartTime)>menuPressTime && firstPress == false) 
    {
        desiredMoisture=checkMoisture(); //reset desired moisture value
        if(debug) Serial.println("Desired Moisture set to: ");
        if(debug) Serial.println(desiredMoisture);
        setLED2(flashGreen);
    }
  }
  else firstPress=true; //reset button flag 
  
  if(attempt>=maxAttempts)  //if watering attemps has exceded maxAttempts, set waterOut alarm and flash red LED
  { 
    waterOut=true;
    setLED2(flashRed);
  }
  else if(moistureValue<desiredMoisture-moistureDiff) waterNeeded=true; //if moisture is lower than desired moisture minus differential set waterNeeded flag

   else if(moistureValue>=desiredMoisture+moistureDiff) 
  {
    waterNeeded=false; //if moisture is greater than desired moisture plus differential reset waterNeeded
 flag
    attempt=0;
  }

  if(!waterOut && waterNeeded==true && millis()-waterTimer>waterCheckTime)   //water the plant if the water is not out, the waterNeeded flag is set, and the water check time has
  {  
 passed
    water();
    attempt++;
    waterTimer=millis();
  }
  else setLED1(); //led1 will fade with the moisture reading.
}

/***** Water function to run pump without servo *****/
void water(void)
{
  #ifdef servoWaterUsed
  {
    int positionDelay=flowTime/4;
    //setLED2(blue);
    //digitalWrite(pumpPin, HIGH);  //turn on pump
    digitalWrite(servoPowerPin,HIGH); //turn on servo power
    for(servoCenter; servoCenter<(90+maxServoAngle); servoCenter++) //move servo from center to max
 servo position
    {
      waterServo.write(servoCenter);
      delay(positionDelay/maxServoAngle);
    }
    for(servoCenter; servoCenter>(90-maxServoAngle); servoCenter--)  //move servo to min servo position
    {
      waterServo.write(servoCenter);
      delay(positionDelay/maxServoAngle);
    }
      for(servoCenter; servoCenter<90; servoCenter++) //move servo to center
    {
      waterServo.write(servoCenter);
      delay(positionDelay/maxServoAngle);
    }
    delay(1000);
    digitalWrite(servoPowerPin,LOW); //turn off servo power
    digitalWrite(pumpPin, LOW); //turn off pump
    }
  #else
  setLED2(blue);
  digitalWrite(pumpPin, HIGH);
  delay(flowTime);
  digitalWrite(pumpPin, LOW);
  #endif
}

/***** Water function to run pump with servo *****/

/***** Check Moisture Value *****/
int checkMoisture()
{
  float weight=0.5;   //weight used to average the reading - lower value=  slower changing average
  digitalWrite(sensorEn,HIGH); //turn on sensor
  delay(250); //wait here for a bit
  filteredMoisture=(weight*analogRead(sensorPin))+(1-weight)*prevMoisture; //filter -weighted average with new reading and previouse reading
 
  moistureValue= map(filteredMoisture,rawMoistureMin,rawMoistureMax,moistureMax,0); //constrain moisture value.
 

    if(debug)Serial.print("Desired Moisture: ");
    if(debug)Serial.println(desiredMoisture);
    if(debug)Serial.print("Filtered Value: ");
    if(debug)Serial.println(filteredMoisture);
    if(debug)Serial.print("Moisture Value: ");
    if(debug)Serial.println(moistureValue);
    

  prevMoisture=filteredMoisture;
  return moistureValue;
}

/***** Set Upper LED *****/
void setLED1(void)
{
  int colorVal,gColorVal,rColorVal; //use map function to set colorVal to usable range (0-255) from current moisture value
  
  colorVal=map(moistureValue,desiredMoisture-moistureDiff,desiredMoisture+moistureDiff,0,255);  
  gColorVal=colorVal;
  if(gColorVal>255) gColorVal=255;
  rColorVal=255-gColorVal;
  if(rColorVal<0) rColorVal=0;
  pixels.setPixelColor(led1, pixels.Color(gColorVal,rColorVal,0)); //set led1 to match moisture
  pixels.setPixelColor(led2, pixels.Color(0,0,0)); //turn off led2
  pixels.show();
}

/***** Set Lower LED *****/
void setLED2(int mode)
{
  switch(mode)
  {
    case flashRed:
      for(int x=0;x<5;x++)
      {
        pixels.setPixelColor(led2, pixels.Color(0,180,0));
        pixels.show();
        delay(flashDelay);
        pixels.setPixelColor(led2, pixels.Color(0,0,0));
        pixels.show();
        delay(flashDelay);
      }
    break;
    case flashGreen:
      for(int x=0;x<5;x++)
      {
        pixels.setPixelColor(led2, pixels.Color(180,0,0));
        pixels.show();
        delay(flashDelay);
        pixels.setPixelColor(led2, pixels.Color(0,0,0));
        pixels.show();
        delay(flashDelay);
      }
    break;
    case green:
        pixels.setPixelColor(led2, pixels.Color(180,0,0));
        pixels.show();
    break;
    case blue:
        pixels.setPixelColor(led2, pixels.Color(0,0,180));
        pixels.show();
    break;
    case amber:
        pixels.setPixelColor(led2, pixels.Color(150,150,0));
        pixels.show();
    break;    
  }
}
