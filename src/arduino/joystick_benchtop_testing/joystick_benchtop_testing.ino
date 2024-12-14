volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
bool reinitialize = false;
String mode = "";
bool brake = false;
String direction = "";
bool debug = false;
float receive;
int steering_angle = 0;

void setup() {
  Serial.begin (115200);

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
   //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
  }
   
  void loop() {
  // Send the value of counter
  if( counter != temp ){
  Serial.println (counter);
  temp = counter;
  
  }
  if (counter>60){
    counter=0;
    
  }
  else if (counter<-60){
    counter=0;
    
  }
  counter++; //dummy data
  delay(500); //dummy delay
  
  //Serial.println(counter);
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
      } else if (pair.startsWith("Debug Mode = ")){
        debug = pair.substring(13).equalsIgnoreCase("True");        
      }

      // Move to the next key-value pair
      lastIndex = nextComma + 1;
    }
    if (debug == true){
    Serial.print("Reinitialize: "); Serial.print(reinitialize);
    Serial.print(", Mode: "); Serial.print(mode);
    Serial.print(", Brake: "); Serial.print(brake);
    Serial.print(", Direction: "); Serial.print(direction);
    Serial.print(", Speed: "); Serial.print(receive);
    Serial.print(", Steering Angle: "); Serial.println(steering_angle);
    Serial.print(", Debug Mode: "); Serial.println(debug);                                                        
    }
  }
  }
   
  void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }
  else{
  counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }
  else{
  counter++;
  }
  }
