#include <Wire.h>

union data_tag {
  float send_output;
  byte byte_to_send[4]; 
} data;

//// Variables for use in the program
  float elapsedTime, time, timePrev;        //Variables for time control
  float rad_to_deg = 180/3.141592654;      //This value is for pasing from radians to degrees values
  float calibErrors[] = {5.04, 4.51, -1.94, 1.99}; // Contains errors from calibration accx,y then gyrox,y (Calibrated externally) Acc_angle_error_x = 5.04;Acc_angle_error_y = 4.51;Gyro_raw_error_x = -1.94;Gyro_raw_error_y = 1.99;
  float pitch, roll, w_feedback;          // Variables from the MPU
  float setpoint = 0;                    // Want to stand still by default
  float output, P, I, D;                  // Control variables     
//
void initializeMPU_i2c(){
  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 
}

void calculateAngles(){
  //////////////////////////////////////Gyro read////////////////////////////////////

    Wire.beginTransmission(0x68);            //begin, Send the slave address (in this case 68) 
    Wire.write(0x43);                        //Address for the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);           //Ask for 6 registers
        
    float Gyr_rawX = Wire.read()<<8|Wire.read();     
    float Gyr_rawY = Wire.read()<<8|Wire.read();
    float Gyr_rawZ = Wire.read()<<8|Wire.read();
    /*In order to obtain the gyro data in degrees/seconds, first divide
    the raw value by 32.8 because that is the value that the datasheet gives for a 1000dps range*/
    
    Gyr_rawX = (Gyr_rawX/32.8) - calibErrors[2]; // Subtract bias error from raw gyro data
    Gyr_rawY = (Gyr_rawY/32.8) - calibErrors[3]; // Same
    Gyr_rawZ = (Gyr_rawZ/32.8); // Missing this one - Need to find it
    w_feedback = Gyr_rawY;

    float Gyro_angle_x = Gyr_rawX*elapsedTime; //Intregrate angular speed to get angle
    float Gyro_angle_y = Gyr_rawY*elapsedTime; //Same as above
    float Gyro_angle_z = Gyr_rawZ*elapsedTime; //Same
  
  //////////////////////////////////////Acc read/////////////////////////////////////

    Wire.beginTransmission(0x68);     //begin, Send the slave address (in this case 68) 
    Wire.write(0x3B);                 //Ask for the 0x3B register (Accelerometer)
    Wire.endTransmission(false);      
    Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting with the 3B  
 
    float Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
    float Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ; // Same
    float Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; // Same
    /*Using atan2 to obtain the angle for the full circle. */  

    float acc_pitch = (atan2(Acc_rawZ, Acc_rawX)*rad_to_deg) - calibErrors[0];
    float acc_roll = (atan2(Acc_rawY, Acc_rawX)*rad_to_deg) - calibErrors[1];    


  //////////////////////////////////////Total angle and filter///////////////////////
    /*---total pitch---*/
    pitch = 0.98 *(pitch + Gyro_angle_y) + 0.02*acc_pitch;
    /*---total roll---*/
    roll = 0.98 *(roll - Gyro_angle_z) + 0.02*acc_roll;
}

void requestEvent(){                        // Data send on request from master
  data.send_output = output;              //Updating value which is to be sent 
  Wire.write(data.byte_to_send, 4);           // Sending 8 bytes (2 floats)
  }
//

void controlLoop(float P_gain, float I_gain, float D_gain) {
  float error = setpoint - pitch;           // Calculate error
  P = P_gain * error;                       // P contribution
  I = I + I_gain * error * elapsedTime;     // I contribution
  D = D_gain * w_feedback;                  // D contribution
  output = P+I+D;                           // output
}

void setup() {   
  Wire.begin(9);                           //begin the wire comunication  
  initializeMPU_i2c();
  
  Wire.onRequest(requestEvent);
  Serial.begin(9600);                     //baudrate
  time = millis();                        //Start counting time in milliseconds
}

void loop() {
  timePrev =  time;                           //Self explaining
  time = millis();                            // update time
  elapsedTime = (time - timePrev) / 1000;     //Self explaining
  calculateAngles();                          //Calculate the pitch and roll.
  controlLoop(3, 0, 0.01);                  // Control loop with gain inputs
  // Serial.println("OUTPUT:");
  // Serial.print(output);

}
