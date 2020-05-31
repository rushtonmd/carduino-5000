int d13pin = 13;
int d13value = 0;
int d12pin = 12; 
int d12value = 0;
int d11pin = 11; 
int d11value = 0;
int d10pin = 10; 
int d10value = 0;

int a5pin = A5; 
int a5value = 0;
int a4pin = A4; 
int a4value = 0;
int a3pin = A3; 
int a3value = 0;
int a2pin = A2; 
int a2value = 0;
int a1pin = A1; 
int a1value = 0;
int a0pin = A0; 
int a0value = 0;



void setup() {
  Serial.begin(115200);
  pinMode(d13pin, INPUT);
  pinMode(d12pin, INPUT);
  pinMode(d11pin, INPUT);
  pinMode(d10pin, INPUT);
  //digitalWrite(d13value,0); 
  pinMode(a5pin, INPUT);
  digitalWrite(a5pin,0); 
  pinMode(a0pin, INPUT);
  digitalWrite(a0pin,0); 
  pinMode(a4pin, INPUT);
  digitalWrite(a4pin,0); 

}

void readAllPinValues(){

  // Read 2 analog pins as sensor inputs. 
  // a5pin = Temp
  // a4pin = Pressure
  a5value = analogRead(a5pin);
  a4value = analogRead(a4pin);

  // Read the 6 digital pins that map to the 
  // transmission selector switch.
  a3value = digitalRead(a3pin);
  a2value = digitalRead(a2pin);
  a1value = digitalRead(a1pin);
  a0value = digitalRead(a0pin);
  d12value = digitalRead(d12pin);
  d11value = digitalRead(d11pin);

  // R
}

void setTransmissionControllerOutputs() {

  /// INPUTS
  // A3 = Park
  // A2 = Reverse
  // A1 = Neutral - DEFAULT
  // A0 = Drive
  // D12 = Second
  // D11 = Low

  if (a3value == 1) { // Park
    Serial.println("PARK!");
    //digitalWrite(d12pin, 0); // A
    //digitalWrite(d11pin, 1); // B
    //digitalWrite(d10pin, 0); // C
  }
  else if (a2value == 1) { // Reverse
    Serial.println("REVERSE!");
    //digitalWrite(d12pin, 1); // A
    //digitalWrite(d11pin, 1); // B
    //digitalWrite(d10pin, 0); // C
  }
  else if (a0value == 1) { // Drive
     Serial.println("DRIVE!");
    //digitalWrite(d12pin, 0); // A
    //digitalWrite(d11pin, 1); // B
    //digitalWrite(d10pin, 1); // C
  }
  else if (d12value == 1) { // Second
     Serial.println("SECOND!");
    //digitalWrite(d12pin, 0); // A
    //digitalWrite(d11pin, 1); // B
    //digitalWrite(d10pin, 1); // C
  }
  else if (d11value == 1) { // L
     Serial.println("LOW!");
    //digitalWrite(d12pin, 1); // A
    //digitalWrite(d11pin, 0); // B
    //digitalWrite(d10pin, 0); // C
  }
  else {
     Serial.println("NEUTRAL!");
    //digitalWrite(d12pin, 0); // A
    //digitalWrite(d11pin, 0); // B
    //digitalWrite(d10pin, 0); // C
  }
}

void printAllInputValues() {
  Serial.print("A5:");
  Serial.print(a5value * (5.0 / 1023.0)); // print the voltage
  Serial.print(" - ");
  Serial.print("A4:");
  Serial.print(a4value);
  Serial.print(" - ");
  Serial.print("A3:");
  Serial.print(a3value);
  Serial.print(" - ");
  Serial.print("A2:");
  Serial.print(a2value);
  Serial.print(" - ");
  Serial.print("A1:");
  Serial.print(a1value);
  Serial.print(" - ");
  Serial.print("A0:");
  Serial.print(a0value);
  Serial.println();

  Serial.print("D12:");
  Serial.print(d12value);
  Serial.print(" - ");
  Serial.print("D11:");
  Serial.print(d11value);
  Serial.println();
}

void loop() {

  printAllInputValues();
  readAllPinValues();
  setTransmissionControllerOutputs();
  
  //val = digitalRead(d13pin);   // read the input pin
  //digitalWrite(d9pin, a5value);  // sets the LED to the button's value
  delay(10);
}
