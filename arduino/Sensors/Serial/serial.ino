void setup(){
  Serial.begin(9600);
  
}

void loop(){

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    Serial.println(inChar + 1);
    } 
  }
}