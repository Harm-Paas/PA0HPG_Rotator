// Relay test for CW and CCW motor relays
/// 2021-06-07 PA0HPG

int in1 = 8; // Relay #1 
int in2 = 9; // Relay #2

void setup() {
  pinMode(in1, OUTPUT);
  digitalWrite(in1, HIGH);
  pinMode(in2, OUTPUT);
  digitalWrite(in2, HIGH); // Both off
}
void loop() {
  digitalWrite(in1, LOW);  // ON #1
  delay(1000);             // One second
  digitalWrite(in1, HIGH); // OFF
  delay(1000);
digitalWrite(in2, LOW);    // ON #2
  delay(2000);             // Two seconds
  digitalWrite(in2, HIGH); // OFF #2
  delay(2000);

}
