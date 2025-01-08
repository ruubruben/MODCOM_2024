void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure pins for encoder 1
  pinMode(2, INPUT); // Encoder 1 Channel A
  pinMode(3, INPUT); // Encoder 1 Channel B

  // Configure pins for encoder 2
  pinMode(A1, INPUT); // Encoder 2 Channel A
  pinMode(A2, INPUT); // Encoder 2 Channel B

  Serial.println("Two Encoder State Monitor Initialized");
}

void loop() {
  // Read the states of encoder 1
  int stateA1 = digitalRead(2); // Encoder 1 Channel A
  int stateB1 = digitalRead(3); // Encoder 1 Channel B

  // Read the states of encoder 2
  int stateA2 = digitalRead(A1); // Encoder 2 Channel A
  int stateB2 = digitalRead(A2); // Encoder 2 Channel B

  // Print the states to the Serial Monitor
  Serial.print("Encoder 1 - A: ");
  Serial.print(stateA1);
  Serial.print(" | B: ");
  Serial.print(stateB1);
  Serial.print(" || Encoder 2 - A: ");
  Serial.print(stateA2);
  Serial.print(" | B: ");
  Serial.println(stateB2);

  // Small delay for readability
  delay(100);
}
