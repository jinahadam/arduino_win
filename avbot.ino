
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"

#else
#include <WProgram.h>
#endif



#include <I2C.h>
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>
#define SERIAL_SPEED 115200  // Default is 115200
#define REALTIME_DATA_REQUEST_INTERAL_MS 200 // interval between reatime data requests

static SBGC_cmd_realtime_data_t rt_data;
static uint16_t cur_time_ms, last_cmd_time_ms, rt_req_last_time_ms;
static int16_t debug1, debug2, debug3, debug4, free_memory;
static float gimbal_pitch,gimbal_yaw, up_lidar, frame_roll, frame_pitch, frame_yaw;
unsigned long pulse_width;

 
HardwareSerial &serial = Serial1;

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.



void setup(){

  //ALEXMOS
  serial.begin(SERIAL_SPEED);
  Serial.begin(SERIAL_SPEED);
  SBGC_Demo_setup(&serial);
  //END ALEXMOS

  //LIDAR
  
  //Serial2.begin(SERIAL_SPEED); // Start serial communications
  //serial2.begin(SERIAL_SPEED); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  //ENDLIDAR

  //LIDAR2
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  pinMode(3, INPUT); // Set pin 3 as monitor pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
  //LIDAR2

  gimbal_pitch = 0.0;
  gimbal_yaw = 0.0;
  up_lidar = 0.0;
}


void loop(){  

   
    //ALEXMOS
  cur_time_ms = millis();
 
  process_in_queue();
 
  //Request realtime data with the fixed rate
  if ((cur_time_ms - rt_req_last_time_ms) > REALTIME_DATA_REQUEST_INTERAL_MS) {
    lidar_pub();
    
   // Serial.println("LOOP");
    SerialCommand cmd;
   
    cmd.init(SBGC_CMD_REALTIME_DATA_4);
    sbgc_parser.send_cmd(cmd, 0);
    rt_req_last_time_ms = cur_time_ms;

    
    //LIDAR2
    pulse_width = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
    if(pulse_width != 0){ // If we get a reading that isn't zero, let's print it
        pulse_width = pulse_width/10; // 10usec = 1 cm of distance for LIDAR-Lite
    Serial.print(gimbal_pitch);
    Serial.print(",");
    Serial.print(gimbal_yaw);
    Serial.print(",");
    Serial.print(up_lidar);
    Serial.print(",");
    Serial.println(pulse_width); // Print the distance
  
    
    }
    //LIDAR2
   
   
    
  }
    
   
}



void lidar_pub() {
   // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses 
  byte distanceArray[2];

      
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
  
  }

  
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses
  int k = 2000;
  while (nackack != 0) {
    k == k - 1;
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
    if (k == 0) {
      break;
    }
  }

 // Serial.println("lidarpub()");
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  
   //Serial.println(distance); 
   up_lidar = distance;
}


// Process incoming commands. Call it as frequently as possible, to prevent overrun of serial input buffer.
void process_in_queue() {
  
  
  while (sbgc_parser.read_cmd()) {
    SerialCommand &cmd = sbgc_parser.in_cmd;
    last_cmd_time_ms = cur_time_ms;
 
    uint8_t error = 0;
 
    switch (cmd.id) {
      // Receive realtime data
      case SBGC_CMD_REALTIME_DATA_3:
      case SBGC_CMD_REALTIME_DATA_4:
        error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
 
        if (!error) {
          //Serial.println("Roll / Pitch / Yaw:");
 
        //  Serial.print(rt_data.imu_angle[ROLL]);
        //  Serial.println(" ALEXIMU -  ");
         // Serial.print(rt_data.frame_imu_angle[PITCH]);
        //  Serial.print(" / ");
        //  Serial.println(rt_data.imu_angle[YAW]);
          gimbal_pitch = rt_data.imu_angle[PITCH];
          gimbal_yaw = rt_data.imu_angle[YAW];
         
          
        } else {
          sbgc_parser.onParseError(error);
        }
        break;
 
    }
   
  }//end of while

}


//ENDALEX MOSS



