//int d13pin = 10;  // LED connected to digital pin 13
//int d13value = 0;      // variable to store the read value
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
  //pinMode(d13pin, INPUT);
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
}

void loop() {

  printAllInputValues();
  readAllPinValues();
  
  //val = digitalRead(d13pin);   // read the input pin
  //digitalWrite(ledPin, val);  // sets the LED to the button's value
  delay(10);
}
