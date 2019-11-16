#include <Wire.h>

float pitch, roll;
  
union data_tag {
float orientation[2];

byte byte_recieved[8]; 
} data;


void setup()
{
  Wire.begin();
    Serial.begin(9600);
}

void loop()
{
Wire.requestFrom(9,8);
while(Wire.available()){
    for(int i=0; i < 8; i++){
  data.byte_recieved[i] = Wire.read();
  }}
  
  pitch = data.orientation[0];
  roll = data.orientation[1];
  Serial.println(pitch);
  Serial.println(roll);
  Serial.println("");
  delay(100);
}
