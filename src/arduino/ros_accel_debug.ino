#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac; // Create an MCP4725 object
float Kp = 0.0050;         // Proportional gain
float Ki = 0.000628;         // Integral gain
float Kd = 0.00003;        // Derivative gain
float alpha = 0.1;//Smooth
float errorCap = 50;
int idle = 10;
float receive;
int flag = 0; // 0 means controlled using the pedal, 1 means control using Arduino
int flagpin = 6;

// these are checked for in the main program
volatile unsigned long timerCounts; // apparently the volatile keyword is to let the compiler know this can have very sharp changes? other than that, this is self explanatory
volatile boolean counterReady; // time to calculate after counting
unsigned long myTime;
bool reinitialize = false;
String mode = "";
bool brake = false;
String direction = "";
int steering_angle = 0;
String debug = "";


float speed = 0;
float pidOutput = 0;
float vvv = 0;
int dacValue = 0;
float frq = 0;
float last_frq = 0;

// internal to counting routine
unsigned long overflowCount; // keeps track of overflows for Timer 1. will make more sense later
unsigned int timerTicks;
unsigned int timerPeriod;
double pulses = 0;


void startCounting (unsigned int ms)
{
  counterReady = false;         // time not up yet
  timerPeriod = ms;             // how many 1 ms counts to do
  timerTicks = 0;               // reset interrupt counter
  overflowCount = 0;            // no overflows yet


  // reset Timer 1 and Timer 2
  TCCR1A = 0;            
  TCCR1B = 0;              
  TCCR2A = 0;
  TCCR2B = 0;


  // Timer 1 - counts events on pin D5
  TIMSK1 = bit (TOIE1);   // interrupt on Timer 1 overflow


  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs.
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit (WGM21) ;   // CTC mode
  OCR2A  = 124;            // count up to 125  (zero relative!!!!)


  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit (OCIE2A);   // enable Timer2 Interrupt


  TCNT1 = 0;      // Both counters to zero
  TCNT2 = 0;    


  // Reset prescalers
  GTCCR = bit (PSRASY);        // reset prescaler now
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);
}  // end of startCounting


ISR (TIMER1_OVF_vect)
{
  ++overflowCount;               // count number of Counter1 overflows  
}  // end of TIMER1_OVF_vect


//**********************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz


ISR (TIMER2_COMPA_vect)
{
  // grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;


  // see if we have reached timing period
  if (++timerTicks < timerPeriod)
    return;  // not yet


  // if just missed an overflow
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 256)
    overflowCopy++;


  // end of gate time, measurement ready


  TCCR1A = 0;    // stop timer 1
  TCCR1B = 0;    


  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;    


  TIMSK1 = 0;    // disable Timer1 Interrupt
  TIMSK2 = 0;    // disable Timer2 Interrupt
   
  // calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
}  // end of TIMER2_COMPA_vect

// Variables for PID calculation
float previousError = 0.0;
float integral = 0.0;
float smoothedDerivative = 0.0;
unsigned long lastTime = 0;

float PID(float setpoint, float measuredValue) {
    // Calculate time elapsed
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    // Calculate the error term
    float error = setpoint - measuredValue;
    if (abs(error) < errorCap)
    {
      error = 0;
    }

    // Proportional term
    float Pout = Kp * error;

    // Integral term with clamping
    integral += error * elapsedTime;
    if (integral > 1.47/Ki) integral = 1.47/Ki;         // Upper clamp
    else if (integral < -1.47/Ki) integral = -1.47/Ki;  // Lower clamp
    float Iout = Ki * integral;

    // Derivative term with smoothing
    float rawDerivative = (error - previousError) / elapsedTime;
    smoothedDerivative = alpha * rawDerivative + (1 - alpha) * smoothedDerivative;
    float Dout = Kd * smoothedDerivative;
    previousError = error;
    // Calculate the PID output
    float output = Pout + Iout + Dout;
    if (debug == "ON"){

    Serial.print("Setpoint:");
    Serial.print(setpoint);
    Serial.print(", Pout:");
    Serial.print(Pout);
    Serial.print(", Iout:");
    Serial.print(Iout);
    Serial.print(", Dout:");
    Serial.print(Dout);
    

    
    Serial.print(", PID:");
    Serial.print(output);
    Serial.print(", Speed:");
    Serial.println(measuredValue, 2);   // 2 decimal places for float
    }
    
    return output;
}

void setup()
{
  Serial.begin(9600);
  
  
  dac.begin(0x60);
  // if (dac.begin(0x60))   // Initialize MCP4725 DAC at default address (0x60)
  // {
  //   Serial.println("MCP4725 Initialized Successfully.");
  // } 
  // else
  // {
  //   Serial.println("Failed.");
  // }
  
  pinMode(idle, OUTPUT);
  dac.setVoltage(0, false);  // Set DAC output to 0V
  // Serial.println("Frequency Counter"); // to show that we have initialized successfully
  startCounting(200); // Start counting for 10 ms (or another period)
}

void loop()
{
  myTime = millis();
  if (counterReady) {
    // adjust counts by counting interval to give frequency in Hz
    frq = (timerCounts *  1000.0) / timerPeriod; // frequency is obtained by dividing the number of detected pulses by the period, then multiplying by 1000 because the period is in ms
    float speed = ((frq) / 740.0) * (2 * 3.1415 * 0.2032)/0.2;
    pulses += timerCounts; // we may use this to keep track of total distance travelled
    // Send pulses and frequency to serial, separated by a comma
    if (abs(last_frq-frq)>900)
    {
      frq = last_frq;
    }
    else
    {
      last_frq = frq;
    }
    Serial.println(frq, 2);   // 2 decimal places for float
    
    // restart counting
    startCounting(200); // Continue counting for 10 ms (or another period)
  }
   if (Serial.available() > 0)
  {
    // Take in Serial input
    String input = Serial.readStringUntil('\n'); // Read the incoming string until a newline character

    // Split the input into individual parameters
    int lastIndex = 0;
    while (lastIndex < input.length()) {
      // Find the next comma
      int nextComma = input.indexOf(',', lastIndex);
      if (nextComma == -1) nextComma = input.length(); // Handle the last parameter

      // Extract the key-value pair
      String pair = input.substring(lastIndex, nextComma);
      pair.trim(); // Remove any leading/trailing whitespace

      // Parse the key-value pair
      if (pair.startsWith("Reinitialize = ")) {
        reinitialize = pair.substring(15).equalsIgnoreCase("True");
      } else if (pair.startsWith("Mode = ")) {
        mode = pair.substring(7);
      } else if (pair.startsWith("Brake = ")) {
        brake = pair.substring(8).equalsIgnoreCase("True");
      } else if (pair.startsWith("Direction = ")) {
        direction = pair.substring(12);
      } else if (pair.startsWith("Speed = ")) {
        receive = pair.substring(8).toFloat();
      } else if (pair.startsWith("Steering Angle = ")) {
        steering_angle = pair.substring(17).toInt();
      } else if (pair.startsWith("DEBUG = ")){
        debug = pair.substring(8);        
      }

      // Move to the next key-value pair
      lastIndex = nextComma + 1;
    }
    if (debug == "ON"){

    Serial.print("Reinitialize: "); Serial.print(reinitialize);
    Serial.print(", Mode: "); Serial.print(mode);
    Serial.print(", Brake: "); Serial.print(brake);
    Serial.print(", Direction: "); Serial.print(direction);
    Serial.print(", Speed: "); Serial.print(receive);
    Serial.print(", Steering Angle: "); Serial.println(steering_angle);
    }
  }
    if (mode == "Manual")
    {
      flag = 0;
      digitalWrite(idle,LOW);
      dac.setVoltage(0, false);
      digitalWrite(flagpin, LOW);
    }
    else
    {
      flag = 1;
      digitalWrite(idle,LOW);
      dac.setVoltage(0, false);
      digitalWrite(flagpin, HIGH);
      
    }
    if (flag == 1)
    {
    if (receive == 0)
    {
      digitalWrite(idle,LOW);
      dac.setVoltage(0, false);
      
    }
    else
    {
      digitalWrite(idle,HIGH);
      // dac.setVoltage(receive*4096/5, false);
    
      dacValue = int(4095 / 5.0 * receive);  // Convert 0-5V range to 0-4095 range for DAC
      dac.setVoltage(dacValue, false);            // Send the corresponding voltage to DAC
      // Serial.println(dacValue);                   // Print the DAC value for debugging
      }
      }
}