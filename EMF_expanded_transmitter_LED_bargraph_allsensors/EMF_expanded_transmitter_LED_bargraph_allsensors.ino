// EMF Detector for LED Bargraph v1.0 with orientation and obstacle detection + wireless data transmission
// 5.12.2009
// original code/project by Aaron ALAI - aaronalai1@gmail.com
// modified for use w/ LED bargraph by Collin Cunningham - collin@makezine.com
// expanded by Luis Bustamante - protonumerique.net


#define NUMREADINGS 15 // raise this number to increase data smoothing

// Include sensor library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


int senseLimit = 250; // raise this number to decrease sensitivity (up to 1023 max)
int probePin = 3; // analog 
int val = 0; // reading from probePin

int LED1 = 2;  // connections
int LED2 = 3;  // to
int LED3 = 4;   // LED
int LED4 = 5;   // bargraph
int LED5 = 6;   // anodes
/*int LED6 = 7;   // with
int LED7 = 8;   // resistors
int LED8 = 9;   // in
int LED9 = 10;   // series
int LED10 = 11;  // 
*/
// variables for smoothing

int readings[NUMREADINGS];                // the readings from the analog input
int index = 0;                            // the index of the current reading
int total = 0;                            // the running total
int average = 0;                          // final average of the probe reading

const int buttonPin = 12;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;

//variables for ultrasonics
const int pwPin1 = 7;
long pulse1, sensor1;


void setup() {

  pinMode(2, OUTPUT);  // specify LED bargraph outputs
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT); 
 /* pinMode(6, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); */

  pinMode(ledPin, OUTPUT);      
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  //ultrasonic input
  pinMode(pwPin1, INPUT);

  Serial.begin(9600);  // initiate serial connection for debugging/etc

  /* Initialise compass sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }


  for (int i = 0; i < NUMREADINGS; i++)
    readings[i] = 0;                      // initialize all the readings to 0
}

void loop(void) {
  
  val = analogRead(probePin);  // take a reading from the probe
   // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  detect();
  
  
  
}


void detect(){

 

  if(val >= 1 && buttonState == LOW){                // if the reading isn't zero, proceed

    val = constrain(val, 1, senseLimit);  // turn any reading higher than the senseLimit value into the senseLimit value
    val = map(val, 1, senseLimit, 1, 1023);  // remap the constrained value within a 1 to 1023 range

    total -= readings[index];               // subtract the last reading
    readings[index] = val; // read from the sensor
    total += readings[index];               // add the reading to the total
    index = (index + 1);                    // advance to the next index

    if (index >= NUMREADINGS)               // if we're at the end of the array...
      index = 0;                            // ...wrap around to the beginning

    average = total / NUMREADINGS;          // calculate the average


    if (average > 50){                // if the average is over 50 ...
      digitalWrite(LED1, HIGH);   // light the first LED
    }
    else{                         // and if it's not ...
      digitalWrite(LED1, LOW);    // turn that LED off
    }


    if (average > 150 * 2){               // and so on ...
      digitalWrite(LED2, HIGH);
    }
    else{
      digitalWrite(LED2, LOW);
    }

    if (average > 250 *2){
      digitalWrite(LED3, HIGH);
    }
    else{
      digitalWrite(LED3, LOW);
    }


    if (average > 350*2){
      digitalWrite(LED4, HIGH);
    }
    else{
      digitalWrite(LED4, LOW);
    }

    if (average > 450*2){
      digitalWrite(LED5, HIGH);
    }
    else{
      digitalWrite(LED5, LOW);
    }

    /* if (average > 500*2){
      digitalWrite(LED6, HIGH);
    }
    else{
      digitalWrite(LED6, LOW);
    }
    
    digitalWrite(ledPin, LOW);

   if (average > 650){
      digitalWrite(LED7, HIGH);
    }
    else{
      digitalWrite(LED7, LOW);
    }

    if (average > 750){
      digitalWrite(LED8, HIGH);
    }
    else{
      digitalWrite(LED8, LOW);
    }

    if (average > 850){
      digitalWrite(LED9, HIGH);
    }
    else{
      digitalWrite(LED9, LOW);
    }

    if (average > 950){
      digitalWrite(LED10, HIGH);
    }
    else{
      digitalWrite(LED10, LOW);
    }*/


    //Serial.println(val); // use output to aid in calibrating
  }
  
  else if (buttonState == HIGH) {    
    // turn LED on:
    Serial.print("D");   
    Serial.println(val);  
    
    digitalWrite(ledPin, HIGH);  
    digitalWrite(LED1, LOW);digitalWrite(LED2, LOW);digitalWrite(LED3, LOW);digitalWrite(LED4, LOW);digitalWrite(LED5, LOW);
    /* digitalWrite(LED6, LOW);digitalWrite(LED7, LOW);digitalWrite(LED8, LOW);digitalWrite(LED9, LOW);digitalWrite(LED10, LOW);*/
    read_sensor();
    printall();
    readCompass();
    
    delay(50);
  }
 

}

void readCompass(){
  /* Get a new sensor event */
  sensors_event_t event; 
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); 
  Serial.print(event.magnetic.x); 
  Serial.print("  ");
  Serial.print("Y: "); 
  Serial.print(event.magnetic.y); 
  Serial.print("  ");
  Serial.print("Z: "); 
  Serial.print(event.magnetic.z); 
  Serial.print("  ");
  Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  Serial.print("Heading (degrees): "); 
  Serial.println(headingDegrees);

  delay(500);

}

void read_sensor(){
  pulse1 = pulseIn(pwPin1, HIGH);
  sensor1 = pulse1/58;
}

//This section of code is if you want to print the range readings to your computer too remove this from the code put /* before the code section and */ after the code
void printall(){         
  Serial.print("S1");
  Serial.print(" ");
  Serial.print(sensor1);
  Serial.println(" ");
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" uT");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  Serial.println(" uT");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

