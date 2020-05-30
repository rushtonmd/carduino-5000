int d13pin = 10;  // LED connected to digital pin 13
int d13value = 0;      // variable to store the read value
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
  digitalWrite(d13value,0); 
  pinMode(a5pin, INPUT);
  digitalWrite(a5pin,0); 
  pinMode(a0pin, INPUT);
  digitalWrite(a0pin,0); 
  pinMode(a4pin, INPUT);
  digitalWrite(a4pin,0); 

}

void loop() {
  Serial.print("D13: ");
  Serial.println(d13value);
  Serial.print("A5: ");
  Serial.println(a5value);
  Serial.print("A0: ");
  Serial.println(a0value);
  Serial.print("A4: ");
  Serial.println(a4value);
  d13value = digitalRead(d13pin);
  a5value = digitalRead(a5pin);
  a4value = digitalRead(a4pin);
  a0value = digitalRead(a0pin);
  //val = digitalRead(d13pin);   // read the input pin
  //digitalWrite(ledPin, val);  // sets the LED to the button's value
  delay(10);
}
