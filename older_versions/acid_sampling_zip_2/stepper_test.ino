#define STEP_PIN 3
#define DIR_PIN 4
#define EN_PIN 5

void setup(){

pinMode(STEP_PIN, OUTPUT);
pinMode(DIR_PIN, OUTPUT);
pinMode(EN_PIN, OUTPUT);

digitalWrite(EN_PIN, LOW); // enables driver (LOW for A4988)
}


void loop(){
  // Forward
  digitalWrite(DIR_PIN, HIGH);

  for(int i = 0; i< 200; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }

  // Reverse
  digitalWrite(DIR_PIN, LOW);

  for(int i =0; i<200; i++){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }
  delay(2000);
}
