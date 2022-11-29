

#include <DynamixelShield.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

int total_legs=4; //number of legs

uint8_t IDs[]={1,7,3,7}; // Leg ID corresponding to [RF,L1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111F,LR,RR] legs.  if not changed, leg RF= ID 1, leg LF= ID 2, leg LR= ID 3, leg RR= ID 4; 
uint8_t Directions[]={0,1,1,0}; // Leg rotating direction corresponding to [RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; If not changed, leg RF and leg RR will rotate CCW, and leg LF and leg LR will rotate CW; 
float gait[]={0,0,0}; // (\phi_1, \phi_2, \phi_3) = [LF-RF,LR-RF,RR-RF]; 
float gait_deg[]={0,0,0,0};  //gait in deg; [RF,LF,LR,RR]; the value for RF will always be 0, the value for LF= 360*\phi_1, LR=360*\phi_2, RR=360*\phi_3; calculated in translate_gait_deg();
float gait_dir_compensation[]={0,240,240,0}; //degree compensation for the legs going in difference direction, [RF,LF,LR,RR], RF 0 as the origin direction, 180*direction+zero_offset

float gait_bound[]={0,0.5,0.5}; // bound gait
float gait_trot[]={0.5,0,0.5}; // trot gait
float gait_walk[]={0.5,0.25,0.75};// walk gait

int Leg_zeroing_offset[]={60,60,150,60}; //zeroing calibration offset in deg; leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                        //by default is 60, which means when position command is 0, all legs should be pointing vertically upward.  

//clock parameters belows need to be calculated in clock_init()///////////////////////////////////////////////
float time_slow_start=1; // in seconds, time after the start of the period that the the leg enters the slow phase.
float time_slow_end=3; // in seconds, time after the start of the period that the the leg exits the slow phase.
float degree_slow_start=150; //in deg, the slow phase starting position
float degree_slow_end=210; //in deg, the slow phase starting position, if degree_slow_start=150 and degree_slow_end=210, this means whenever the leg's position is between 150 and 210, it will be in the slow phase.  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//You can change the desired Buehler clock parameters here
float phi_s=0.85; //in rad, ϕ_s is the angular extent of the slow phase
float phi_0=2.75; //in rad, ϕ_0 is the center of the slow phase 
float d_c=0.56;//d_c is the duty factor of the slow phase (i.e. fraction of the period spent in the slow phase). 
float clock_period=4; //in seconds, time to complete 1 rotation

int obstaclePin = 8;
int hasObstacle = HIGH;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

float convert_rad_to_degree(float rad_val) {
  float degree_val = 0.0;
  degree_val = (rad_val / (2 * 3.14) ) * 360;

  return degree_val;
}

float convert_degree_to_rad(float degree_val) {
  float rad_val = 0.0;
  rad_val = (degree_val / 360) * 2 * 3.14;

  return rad_val;
}

//change code to return desired speed in slow phase
float omega_slow(){ 
  float angular_speed = 0.0;
  angular_speed = phi_s / (d_c * clock_period);
  
  return convert_rad_to_degree(angular_speed);  //return desired leg speed in slow phase in deg/s
}


//change code to return desired speed in fast phase
float omega_fast(){  
  float angular_speed = 0.0;
  angular_speed = (2.0 * 3.14 - phi_s) / ((1.0 - d_c) * clock_period);
  
  return convert_rad_to_degree(angular_speed); //return desired leg speed in fast phase in deg/s
}

//configure your timing parameters
void clock_init(){   
  //Insert your calculated time_slow_start and time_slow_end here. You do not have to use degree_slow_start and degree_slow_end, but they may be helpful.
  // at the beginning of each stride period, the desired angle, \phi, should be 0 degree (leg should point vertically upward if you have a leg installed). 
  // we suggest that you make sure that the deadzone (300deg to 360deg) is fully within the fast phase (i.e., 0<degree_slow_start<degree_slow_end<300). Also make sure 0<time_slow_start<time_slow_end<clock_period
  // notice degree_slow_start, degree_slow_end here are in deg  
  
  //degree_slow_start=  //FIXME(optional)  //return the position that the the leg enters the slow phase in deg
  //degree_slow_end= //FIXME(optional)  //return the position that the the leg exits the slow phase in deg
  time_slow_start= (phi_0 - (phi_s / 2.0)) / convert_degree_to_rad(omega_fast());  //return the time after the start of the period that the the leg enters the slow phase
  time_slow_end= time_slow_start + (phi_s / convert_degree_to_rad((omega_slow()))); //return the time after the start of the period that the the leg exits the slow phase
 
  return;
}

// compute desired motor angle at any time instance
float get_desired_angle(int leg,                    // Robot's leg enum, in 0,1,2,3
                   long elapsed,               // time elapsed since start, in ms
                   bool *start) { // return the desired postion of the robot's leg given the leg number and the time elapsed in deg. It's an absolute position from 0 to 360. Don't use cumulative positions. It's ok to return positions in the deadzone (>300).    
  // calculate the time based on the time and phase difference
  float cycle_time = fmod(((elapsed / 1000.0) + (gait_deg[leg] / 360.0 * clock_period)), clock_period);
  
  // calculate the position in radiance
  float cycle_pos = 0.0;
  if(cycle_time < time_slow_start)
  {
      // fast phase before entering slow phase
      cycle_pos = omega_fast() * cycle_time;
  }
  else if (cycle_time < time_slow_end)
  {
      // within slow phase
      cycle_pos = omega_fast() * time_slow_start + omega_slow() * (cycle_time - time_slow_start);
      if(cycle_time >= time_slow_start + (time_slow_end-time_slow_start)/2)
      {
        *start = true; 
        // start wheel
      }
  }
  else
  {
      // exited slow phase, in fast phase again
      cycle_pos = omega_fast() * time_slow_start + omega_slow() * (time_slow_end - time_slow_start) + omega_fast() * (cycle_time - time_slow_end);
      *start = false;
      // stop wheel
  }

  // account for zeroing and direction compensation
  return fmod((cycle_pos + Leg_zeroing_offset[leg] + gait_dir_compensation[leg]), 360.0); //in deg
}

// print desired motor position and associated elapsed time to debug serial
void print_position(long t, int leg, float desired_pos){
  // print leg 
  DEBUG_SERIAL.print("[");
  DEBUG_SERIAL.print(leg);
  // print time stamp in ms
  DEBUG_SERIAL.print("] time: ");
  DEBUG_SERIAL.print(t);
  // print desired position in degrees
  DEBUG_SERIAL.print(" ms , position: ");
  DEBUG_SERIAL.print(desired_pos);
  DEBUG_SERIAL.print(", Present Position(degree): ");
  DEBUG_SERIAL.print(dxl.getPresentPosition(1, UNIT_DEGREE));
  DEBUG_SERIAL.print("\n");
        
  //return 0;
}


void translate_gait_deg(){ //calculate the value of the array 'gait_deg[]' 
  //notice the unit of 'gait_deg[]' is in deg and has 4 elements
  gait_deg[0]=0;  //RF
  gait_deg[1]=gait[0]*360; //LF
  gait_deg[2]=gait[1]*360; //LB
  gait_deg[3]=gait[2]*360; //RB

  //also calculate the degree compensation needed for the direction difference
  gait_dir_compensation[0]=0; //RF
  gait_dir_compensation[1]=180*Directions[1]+Leg_zeroing_offset[1]*Directions[1];
  gait_dir_compensation[2]=180*Directions[2]+Leg_zeroing_offset[2]*Directions[2];
  gait_dir_compensation[3]=180*Directions[3]+Leg_zeroing_offset[3]*Directions[3];
  
   return;
}

int dead_zone_speed_tuning=0; //adjustment to tune deadzone speed, MAGIC Variable as we are using position control outside the deadzone and speed control inside deadzone so the speed might be different; this variable is manually selected from observation, and it's ok that it's not working well.  
//int different_direction_offset=-110; //adjustment to compensate position offset between 2 legs with different rotating direction. Another MAGIC variable that requires manual observation. 


////////////////////////////////////////////// Do not change any code below this line/////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void start_back_wheel(bool state){
  if((state == true)){
   dxl.setGoalVelocity(5, 1024+2*omega_fast());
   dxl.setGoalVelocity(6, 2*omega_fast());
   
  }
  else if ((state == false)){
    dxl.setGoalVelocity(5, 0);
   dxl.setGoalVelocity(6, 0);
    
  }
}

long start;
float start_orientation;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  
  // Turn off torque when configuring items in EEPROM area
  for (int i=0;i<=0;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
    dxl.setGoalPosition(IDs[i], 100, UNIT_DEGREE);

   
  }
  dxl.torqueOff(5);
  dxl.setOperatingMode(5, OP_VELOCITY);
  dxl.torqueOn(5);
  dxl.torqueOff(6);
  dxl.setOperatingMode(6, OP_VELOCITY);
  dxl.torqueOn(6);  

  dxl.torqueOff(2);
  dxl.setOperatingMode(2, OP_POSITION);
  dxl.torqueOn(2);
    dxl.torqueOff(4);
  dxl.setOperatingMode(4, OP_POSITION);
  dxl.torqueOn(4);

  dxl.setGoalPosition(2, 150 , UNIT_DEGREE);
  dxl.setGoalPosition(4, 150  , UNIT_DEGREE);
  
  start = millis();
  clock_init();
  translate_gait_deg(); 

  pinMode(obstaclePin, INPUT);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    DEBUG_SERIAL.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true); 
  
  
}


int time_step=100;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in
bool start_leg=false;

void Advance_Robot()
{
  // Record the start position and start time
  DEBUG_SERIAL.print("starting loop");
  long last_time=0;
  start = millis();
  int start_pos= 100;
  dxl.setGoalPosition(1, start_pos, UNIT_DEGREE);
  int curr_pos=0;
  int count =0;
  long elapsed=0;
  
  do
  {
    elapsed = millis() - start;
  
    if (elapsed-last_time>time_step){
      last_time=elapsed;
      for (int i=0;i<total_legs;i++){
        
        float desired_pos=get_desired_angle(i,elapsed,&start_leg);
        start_back_wheel(start_leg);
        if (Directions[i]==1) desired_pos=360.0-desired_pos;
          
      //  print_position(elapsed, i,desired_pos);
        
        if (in_dead_zone[i]==0){

          
          if (desired_pos<300)
            dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
          else{
            in_dead_zone[i]=1;

            float present_speed=dxl.getPresentVelocity(IDs[i]);
            dxl.torqueOff(IDs[i]);
            dxl.setOperatingMode(IDs[i], OP_VELOCITY);
            dxl.torqueOn(IDs[i]);
            //1 rpm=6 deg/s=9 unit
            //1 unit= 2/3 deg/s
            delay(10);
            if(Directions[i]==0)
              dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
            else
              dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
            
          }
          delay(10);   
        }
        else{
          int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
          bool flag_temp=0;
          
          
          if(Directions[i]==0)
              flag_temp=current_pos>20;
          else
              flag_temp=current_pos<280;
            
          
          if (flag_temp && desired_pos>45&& desired_pos<300){
            in_dead_zone[i]=0;
            dxl.torqueOff(IDs[i]);
            dxl.setOperatingMode(IDs[i], OP_POSITION);
            dxl.torqueOn(IDs[i]);
            //1 rpm=6 deg/s=9 unit
            //1 unit= 2/3 deg/s
            delay(10);
            dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
          }
          
        }
      }
      curr_pos=dxl.getPresentPosition(1, UNIT_DEGREE);
      count++;
      
      DEBUG_SERIAL.print("count=");
      DEBUG_SERIAL.print(count);
      DEBUG_SERIAL.print(" corrpos=");
      DEBUG_SERIAL.println(curr_pos); 
    }
    
  } while (elapsed < clock_period*1000);//(count<10) || !((curr_pos > 100) && (curr_pos < 132)));
  
  
}

float get_x_orientation(){
  sensors_event_t event; 
  bno.getEvent(&event);

  return event.orientation.x;
}

bool sense_obs(int leg){

  hasObstacle = digitalRead(obstaclePin);
  if (hasObstacle == LOW)
  {
   
    return LOW;
  }
  else
  {
    
    return HIGH;
  }
  
}

void set_search_angles(float val){
  dxl.setGoalPosition(2, 150 +val , UNIT_DEGREE);
  dxl.setGoalPosition(4, 150+val  , UNIT_DEGREE);
  return;
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  DEBUG_SERIAL.print("X: ");
  DEBUG_SERIAL.print(event.orientation.x, 4);
  DEBUG_SERIAL.print("\tY: ");
  DEBUG_SERIAL.print(event.orientation.y, 4);
  DEBUG_SERIAL.print("\tZ: ");
  DEBUG_SERIAL.print(event.orientation.z, 4);
  DEBUG_SERIAL.println("");/*
  float orientation =event.orientation.x;
  float sigma = orientation - start_orientation;

  DEBUG_SERIAL.print("orient raw=");
      DEBUG_SERIAL.print(orientation);
      DEBUG_SERIAL.print(" sigma=");
      DEBUG_SERIAL.println(sigma);*/

  dxl.torqueOff(2);
  dxl.torqueOff(4);
  //set_search_angles(sigma);
  Advance_Robot();
  dxl.torqueOn(2);
  dxl.torqueOn(4);
  dxl.setGoalPosition(2, 150  , UNIT_DEGREE);
  dxl.setGoalPosition(4, 150 , UNIT_DEGREE);
  delay(2000);
  /*long elapsed = millis() - start;
  
  if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      
      float desired_pos=get_desired_angle(i,elapsed,&start_leg);
      start_back_wheel(start_leg);
      if (Directions[i]==1) desired_pos=360.0-desired_pos;
        
      print_position(elapsed, i,desired_pos);
      
      if (in_dead_zone[i]==0){

        
        if (desired_pos<300)
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        else{
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          if(Directions[i]==0)
            dxl.setGoalVelocity(IDs[i], 1.5*omega_fast()+dead_zone_speed_tuning);
          else
            dxl.setGoalVelocity(IDs[i], 1024+1.5*omega_fast()+dead_zone_speed_tuning);
          
        }
        delay(10);   
      }
      else{
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool flag_temp=0;
        
        
        if(Directions[i]==0)
            flag_temp=current_pos>20;
         else
            flag_temp=current_pos<280;
          
        
        if (flag_temp && desired_pos>45&& desired_pos<300){
          in_dead_zone[i]=0;
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_POSITION);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        }
        
      }
    }
  }*/
  
}
