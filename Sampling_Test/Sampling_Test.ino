const int T_SAMP = 200;                         // sample period
uint32_t  period = 0;                           // stores time between now and previous sample time

void setup() {
  Serial.begin(9600);                           // set up serial uart
}

uint32_t timer = millis();


void loop() {  
   if (timer > millis())  timer = millis();     // wrap timer reset
   
    period = millis() - timer;                  // calculate period now so it doesn't change when we print
    if (period >= T_SAMP) {                     // once we've counted up to the sample period, do stuff
      timer = millis();
    
      /*Delay Simulating various processing*/    
      delay(100);
      Serial.println(period);                   // print the time period
  }
}

