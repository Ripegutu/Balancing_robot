#include <Wire.h>

// Variables
  float pitch, roll;
  
union data_tag {
  float pwm_input;
  byte byte_recieved[4]; 
} data;


void setup(){
    Wire.begin();
    Serial.begin(9600);
}

void loop(){

  Wire.requestFrom(9,4);
  while(Wire.available()){
      for(int i=0; i < 4; i++){
    data.byte_recieved[i] = Wire.read();
    }}
  
  float pwm = data.pwm_input;
  if(pwm == pwm){
    Serial.println(pwm);
    Serial.println("");
  }
  delay(100);
}
