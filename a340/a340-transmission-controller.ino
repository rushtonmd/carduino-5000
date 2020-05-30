int d9pin = 9;
int d9value = 0;
int d12pin = 12; 
int d12value = 0;
int d11pin = 11; 
int d11alue = 0;
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
  pinMode(d9pin, OUTPUT);
  pinMode(d12pin, OUTPUT);
  pinMode(d11pin, OUTPUT);
  pinMode(d10pin, OUTPUT);
  //digitalWrite(d13value,0); 
  pinMode(a5pin, INPUT);
  digitalWrite(a5pin,0); 
  pinMode(a0pin, INPUT);
  digitalWrite(a0pin,0); 
  pinMode(a4pin, INPUT);
  digitalWrite(a4pin,0); 

}

void readAllPinValues(){

  // Read all analog pins. The 6 analog pins are mapped to the 
  // shifter switch on the a340e transmission
  a5value = digitalRead(a5pin);
  a4value = digitalRead(a4pin);
  a3value = digitalRead(a3pin);
  a2value = digitalRead(a2pin);
  a1value = digitalRead(a1pin);
  a0value = digitalRead(a0pin);
}

void setTransmissionControllerOutputs() {

  /// INPUTS
  // A5 = Park
  // A4 = Reverse
  // A3 = Neutral
  // A2 = Drive
  // A1 = Low
  // A0 = Overdrive

  // OUTPUTS
  // D12 = A
  // D11 = B
  // D10 = C

  if (A5 == 1) { // Park
    digitalWrite(d12pin, 0); // A
    digitalWrite(d11pin, 1); // B
    digitalWrite(d10pin, 0); // C
  }
  else if (A4 == 1) { // Reverse
    digitalWrite(d12pin, 1); // A
    digitalWrite(d11pin, 1); // B
    digitalWrite(d10pin, 0); // C
  }
  else if (A3 == 1) { // Neutral
    digitalWrite(d12pin, 0); // A
    digitalWrite(d11pin, 1); // B
    digitalWrite(d10pin, 0); // C
  }
  else if (A2 == 1) { // Drive
    digitalWrite(d12pin, 0); // A
    digitalWrite(d11pin, 1); // B
    digitalWrite(d10pin, 1); // C
  }
  else if (A1 == 1) { // L
    digitalWrite(d12pin, 1); // A
    digitalWrite(d11pin, 0); // B
    digitalWrite(d10pin, 0); // C
  }
  else {
    digitalWrite(d12pin, 0); // A
    digitalWrite(d11pin, 0); // B
    digitalWrite(d10pin, 0); // C
  }
}

void printAllInputValues() {
  Serial.print("A5:");
  Serial.print(a5value);
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

  Serial.print("D9:");
  Serial.print(d9value);
  Serial.println();
}

void loop() {

  printAllInputValues();
  readAllPinValues();
  
  //val = digitalRead(d13pin);   // read the input pin
  digitalWrite(d9pin, a5value);  // sets the LED to the button's value
  delay(10);
}
