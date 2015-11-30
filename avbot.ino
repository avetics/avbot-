

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"

#else
#include <WProgram.h>
#endif

#include <I2C.h>

unsigned long lastFiredTime;
unsigned long interval;

//ALEX MOX CODE
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>


//#define SERIAL_BUFFER_SIZE 150
// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200  // Default is 115200
#define REALTIME_DATA_REQUEST_INTERAL_MS 200 // interval between reatime data requests
 
// Functions prototypes
//inline void process_cmd_realtime_data();
//inline void process_in_queue();
 
// Global variables
static SBGC_cmd_realtime_data_t rt_data;
static uint16_t cur_time_ms, last_cmd_time_ms, rt_req_last_time_ms;
static int16_t debug1, debug2, debug3, debug4, free_memory;

 
HardwareSerial &serial = Serial1;
 
//END ALEXMOS

volatile unsigned int Start_Pulse =0;
volatile unsigned int Stop_Pulse =0;
volatile unsigned int Pulse_Width =0;
int upper_PWM_bound=1850;
int lower_PWM_bound=1150;
volatile int Test=0;
volatile int Test2=0;
volatile int Temp=0;
volatile int Counter=0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {
  2400,2400,2400,2400,2400,2400,2400,2400};
int All_PWM=1500;

long timer=0;
long timer2=0;
int ch0=1500,ch1=1500;
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include <avr/interrupt.h>


void OutputCh(byte ch, int pwm)
{
  pwm=constrain(pwm,900,2100);
  pwm*=2;

  switch(ch)
  {
  case 0:  
    OCR5B=pwm; 
    break;  //ch0
  case 1:  
    OCR5C=pwm; 
    break;  //ch1
  case 2:  
    OCR1B=pwm; 
    break;  //ch2
  case 3:  
    OCR1C=pwm; 
    break;  //ch3
  case 4:  
    OCR4C=pwm; 
    break;  //ch4
  case 5:  
    OCR4B=pwm; 
    break;  //ch5
  case 6:  
    OCR3C=pwm; 
    break;  //ch6
  case 7:  
    OCR3B=pwm; 
    break;  //ch7
  case 8:  
    OCR5A=pwm; 
    break;  //ch8,  PL3
  case 9:  
    OCR1A=pwm; 
    break;  //ch9,  PB5
  case 10: 
    OCR3A=pwm; 
    break;  //ch10, PE3
  } 

}

int InputCh(byte ch)
{
  return (PWM_RAW[ch]+600)/2.0; 
}

void Init_PWM1(void)
{
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);

  //Remember the registers not declared here remains zero by default... 
  TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR1A = 3000; //PB5, none
  OCR1B = 3000; //PB6, OUT2
  OCR1C = 3000; //PB7  OUT3
  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
}

void Init_PWM3(void)
{
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  //Remember the registers not declared here remains zero by default... 
  TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR3A = 3000; //PE3, NONE
  OCR3B = 3000; //PE4, OUT7
  OCR3C = 3000; //PE5,  OUT6
  ICR3 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
}

void Init_PWM5(void)
{
  pinMode(44,OUTPUT);
  pinMode(45,OUTPUT);
  pinMode(46,OUTPUT);

  TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)); 
  TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51); //Prescaler set to 8
  OCR5A = 3000; //PL3, 
  OCR5B = 3000; //PL4, OUT0
  OCR5C = 3000; //PL5  OUT1
  ICR5 = 40000;
  //ICR5 = 43910; //So (16000000hz/8)/50hz=40000,
}

/*Note that timer4 is configured to used the Input capture for PPM decoding and to pulse two servos 
 OCR4A is used as the top counter*/
void Init_PPM_PWM4(void)
{
  pinMode(49, INPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  //Remember the registers not declared here remains zero by default... 
  TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));  
  TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR4A = 40000; ///50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000, 
  //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff...   
  OCR4B = 3000; //PH4, OUT5
  OCR4C = 3000; //PH5, OUT4

  TIMSK4 |= (1<<ICIE4); //Timer interrupt mask
  sei();

}
/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER4_CAPT_vect)//interrupt. 
{
  if(((1<<ICES4)&TCCR4B) >= 0x01)
  { 
    if(Start_Pulse>Stop_Pulse) //Checking if the Stop Pulse overflow the register, if yes i normalize it. 
    {
      Stop_Pulse+=40000; //Nomarlizing the stop pulse.
    }
    Pulse_Width=Stop_Pulse-Start_Pulse; //Calculating pulse 
    if(Pulse_Width>5000) //Verify if this is the sync pulse
    {
      PPM_Counter=0; //If yes restart the counter
    }
    else
    {
      PWM_RAW[PPM_Counter]=Pulse_Width; //Saving pulse. 
      PPM_Counter++; 
    }
    Start_Pulse=ICR4;
    TCCR4B &=(~(1<<ICES4)); //Changing edge detector. 
  }
  else
  {
    Stop_Pulse=ICR4; //Capturing time stop of the drop edge
    TCCR4B |=(1<<ICES4); //Changing edge detector. 
    //TCCR4B &=(~(1<<ICES4));
  }
  //Counter++;
}



ros::NodeHandle  nh;

geometry_msgs::Twist fb_msg;
geometry_msgs::Vector3 camera;
std_msgs::Int16 dist;
ros::Publisher pub_feedback("feedback", &fb_msg);
ros::Publisher pub_camera_pitch("camera", &camera);
ros::Publisher pub_up_lidar("up_lidar", &dist);

geometry_msgs::Twist cmd_vel;
std_msgs::Int8 joystick_status;
//std_msgs::Float64 T,W;
float Tx=0,Ty=0,Tz=0,W=0,RT=0,LT=0;
float Tcx,Tcy,Tcz,Wc;
int M1=1500,M2=1500,M3=1500,M4=1500,M5=1500;

void motor_cb( const geometry_msgs::Twist& cmd_msg){
  
  Tx=cmd_msg.linear.x;
  Ty=cmd_msg.linear.y;
  Tz=cmd_msg.linear.z;
  
  W=cmd_msg.angular.z;

  LT=cmd_msg.angular.x+1;      //angular x is LT button
  RT=cmd_msg.angular.y+1;      //angular y is RT button
  int Toffset=0,Woffset=0;


  Tcx=Tx*500+Toffset;
  Tcy=Ty*500+Toffset;
  Tcz=Tz*500+Toffset;
  Wc=W*-500+Woffset;

  
   

  M1= constrain(-Tcx, -500, 500) + 1500; //pitch
  M2= constrain(-Tcy, -500, 500) + 1500;  //roll
  M3= constrain(Tcz, -500, 500) + 1500; //throttle 
  M4= constrain(Wc, -500, 500) + 1500; //yaw
  
  M5=1500+LT*500-RT*500;
  M5= constrain(M5, 1000, 2000); 
  // M2= (Tc-Wc + 1500); 
  /*
  fb_msg.data=M1;
  pub_feedback.publish(&fb_msg);
  fb_msg.data=M2;
  pub_feedback.publish(&fb_msg);
  fb_msg.data=M3;
  pub_feedback.publish(&fb_msg);
  fb_msg.data=M4;
  pub_feedback.publish(&fb_msg);
  fb_msg.angular.x = M1;
  fb_msg.angular.y = M2;
  fb_msg.angular.z = M3;
  fb_msg.linear.x = M4;
  fb_msg.linear.y = M5;
  fb_msg[1].data=M2;
  pub_feedback.publish(&fb_msg);
  */
  
  // Serial.println("feedback()");

  
  
}

void ex_failsafe( const std_msgs::Int8& cmd_msg){
  //Serial.println(cmd_msg.data);
  joystick_status = cmd_msg;
  
   unsigned long now = millis ();
   interval = now - lastFiredTime;
   lastFiredTime = now;
  
}


ros::Subscriber<geometry_msgs::Twist> motorsub("cmd_vel", motor_cb);
ros::Subscriber<std_msgs::Int8> failsafesub("failsafe", ex_failsafe);

//std_msgs::Int16 fb_msg;
int loopcounter=0;
unsigned long time;

//LIDAR
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


//ENDLIDAR

void setup(){
  nh.initNode();
  nh.subscribe(motorsub);
  nh.subscribe(failsafesub);
  nh.advertise(pub_feedback);
  nh.advertise(pub_camera_pitch);
  nh.advertise(pub_up_lidar);
  Init_PWM1(); //OUT2&3 
  Init_PWM3(); //OUT6&7
  Init_PWM5(); //OUT0&1
  Init_PPM_PWM4(); //OUT4&5

  OutputCh(0, 1500);
  OutputCh(1, 1500);
  OutputCh(2, 1500);
  OutputCh(3, 1500);
  OutputCh(4, 1500);
  OutputCh(5, 1500);
  time=millis();

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

 
  interval = 0;
}



void loop(){


    //ALEXMOS
  cur_time_ms = millis();
 
  process_in_queue();
 
  //Request realtime data with the fixed rate
  if ((cur_time_ms - rt_req_last_time_ms) > REALTIME_DATA_REQUEST_INTERAL_MS) {
    lidar_pub();
    
  //  Serial.println("LOOP");
    SerialCommand cmd;
    cmd.init(SBGC_CMD_REALTIME_DATA_4);
    sbgc_parser.send_cmd(cmd, 0);
 
    rt_req_last_time_ms = cur_time_ms;
    
  }
    
  //ENDALEXMOS
  
  
  

  
  
  nh.spinOnce();
  delay(4);
  // here pin refer to physical pin writting on the casing of APM
  // note APM1 and APM2 series has entirely difference pin mapping
 
  OutputCh(4, M1);//pin3 pitch 
  OutputCh(5, M2);//pin4 roll
  OutputCh(6, M4);//pin6 yaw
   
 //JOYSTICK FAILSAFE
 
 if (joystick_status.data != 1) {
   OutputCh(2, 0);//pin1 thr
 } else {
   OutputCh(2, M3);//pin1 thr
 }


 unsigned long now = millis ();
 unsigned long interval_2 = now - lastFiredTime;

 //Serial.println("LAST FIRED TIME"); 
 //Serial.println(interval_2);

 if (interval_2 > 3000) {
  joystick_status.data = 0;
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
  
  dist.data=distance;
  pub_up_lidar.publish(&dist);
  
}


// Process incoming commands. Call it as frequently as possible, to prevent overrun of serial input buffer.
void process_in_queue() {
  
  
  while (sbgc_parser.read_cmd()) {
   // Serial.println("PROCESS IN Q");
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
 
         // Serial.print(rt_data.imu_angle[ROLL]);
         // Serial.print(" ALEXMOS ");
          //Serial.print(rt_data.imu_angle[PITCH]);
          //Serial.print(" / ");
         // Serial.println(rt_data.imu_angle[YAW]);
          camera.x =  rt_data.imu_angle[ROLL];//, rt_data.imu_angle[PITCH], rt_data.imu_angle[YAW]];
          camera.y =  rt_data.imu_angle[PITCH];//, rt_data.imu_angle[PITCH], rt_data.imu_angle[YAW]];
          camera.z =  rt_data.imu_angle[YAW];//, rt_data.imu_angle[PITCH], rt_data.imu_angle[YAW]];
          pub_camera_pitch.publish(&camera);

          

         
        } else {
          sbgc_parser.onParseError(error);
        }
        break;
 
    }
   
  }//end of while

  

}


//ENDALEX MOSS




