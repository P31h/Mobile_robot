#include <esp32_imu.h>





/***********************************
           ENCODER_MACRO
************************************/

// M1 : front left
#define ENC1_1   16   // This pin give encoder feedback for the system (11 pulses per motor's rotation)
#define ENC2_1   4
#define PWM_1    2    // This pin set the PWM value defining the output power of motor
#define IN1_1    0    // This pin define the rotational direction of the motor  
#define IN2_1    15   // This pin define the rotational direction of the motor

// M2 : front right
#define ENC1_2   26
#define ENC2_2   27
#define PWM_2    14 
#define IN1_2    12 
#define IN2_2    13

// M3 : back_left

#define ENC1_3   17 
#define ENC2_3   5
#define PWM_3    18 
#define IN1_3    19 
#define IN2_3    23 

// pin 35 and pin 34 can only be used as input mode


// M4 : back right
#define ENC1_4   35  
#define ENC2_4   34
#define PWM_4    25  
#define IN1_4    33 
#define IN2_4    32 


 
#define PWM_RESOLUTION 255
 


// GLOBAL variable

float IMU_x_accel_data=0;
float IMU_y_accel_data=0;
float IMU_yaw_data=0;

short pulse_val_1 = 0;
short pulse_val_2 = 0;
short pulse_val_3 = 0;
short pulse_val_4 = 0;

int PrevT = 0;
int PrevT2 = 0;

float vel_array[3];


void set_motor(int input_data);
void IMU_Get_Data_func();



// ************************************************************************** // 


// ************************************************************************** // 

void setup() {
  Serial.begin(250000); //230400 //250000 // 115200 SerialBT.begin("ESP32test"); //Bluetooth device name
  while (!Serial){}
    delay(10); 

 
 

  delay(2000);
  // init IMU 
  Init_MPU6050_func();
  IMU_Calib_func();
  delay(100);

  vel_array[0]= 0;
  vel_array[1]= 0;
  vel_array[2]= 0;

  // init motor and encoder pins
  pinMode(ENC1_1,INPUT);
  pinMode(ENC2_1,INPUT);
  pinMode(PWM_1,OUTPUT);
  pinMode(IN1_1,OUTPUT); 
  pinMode(IN2_1,OUTPUT);
  

  pinMode(ENC1_2,INPUT);
  pinMode(ENC2_2,INPUT);
  pinMode(PWM_2,OUTPUT);
  pinMode(IN1_2,OUTPUT);
  pinMode(IN2_2,OUTPUT);


  
  pinMode(ENC1_3,INPUT);
  pinMode(ENC2_3,INPUT);
  pinMode(PWM_3,OUTPUT);
  Serial.println("3-1");
  pinMode(IN1_3,OUTPUT);
  Serial.println("3-2");
  pinMode(IN2_3,OUTPUT);
  Serial.println("3-3");

  
  pinMode(ENC1_4,INPUT);
  pinMode(ENC2_4,INPUT);
  pinMode(PWM_4,OUTPUT);
  pinMode(IN1_4,OUTPUT);
  pinMode(IN2_4,OUTPUT);

  delay(100);

  // setting interrupt for ENC1 of each encoder
  attachInterrupt(digitalPinToInterrupt(ENC1_1),readEncoder_A1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC1_2),readEncoder_A2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC1_3),readEncoder_A3,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC1_4),readEncoder_A4,RISING);
  

}



uint8_t calculate_checksum(uint8_t *data) 
{
  uint8_t checksum = 0;
  checksum |= 0b11110000 & data[1];
  checksum |= 0b00001111 & data[2];
  return checksum;
}

// 121 122 123 --- 100
// 0 --- 40
// 256 --- 160
// left_y: recv_data[1] 
// right_x: recv_data[2]
void loop() 
{
  

  /* send imu data and encoder every 0.1s */
  int CurrentT = millis();
  if(CurrentT - PrevT > 100)
  {
  IMU_Get_Data_func();
  
  Serial.print(pulse_val_1);
  Serial.print(";");
  Serial.print(pulse_val_2);
  Serial.print(";");
  Serial.print(pulse_val_3);
  Serial.print(";");
  Serial.print(pulse_val_4);
  Serial.print(";");
  Serial.print(IMU_x_accel_data, 2);
  Serial.print(";");
  Serial.print(IMU_y_accel_data, 2);
  Serial.print(";");
  Serial.print(IMU_yaw_data, 2);
  Serial.print("/");

  pulse_val_1 = 0;
  pulse_val_2 = 0;
  pulse_val_3 = 0;
  pulse_val_4 = 0;
  // Serial.print("/");
  // Serial.print("50");
  // Serial.print(";");
  // Serial.print("50");
  // Serial.print(";");
  // Serial.print("50");
  // Serial.print(";");
  // Serial.print("50");
  // Serial.print(";");
  // Serial.print("0");
  // Serial.print(";");
  // Serial.print("0");
  // Serial.print(";");
  // Serial.print("0");
  // Serial.print("/");

  PrevT = CurrentT;
  }

  // control velocity from the rasberry pi
  if (Serial.available() >0)
  {

  String message = Serial.readStringUntil('\n');

  String_decode(message,vel_array);
  vel_array[0] = Check_input_value(vel_array[0]);
  vel_array[1] = Check_input_value(vel_array[1]);
  vel_array[2] = Check_input_value(vel_array[2]); 
  }


  int CurrentT2 = millis();
  if(CurrentT2 - PrevT2 > 20)
  {
    motor_control(vel_array[0],vel_array[1],vel_array[2]);
    PrevT2 = CurrentT2;
  }

}

float Check_input_value(float input_number)
{
   if(input_number <= 0.08 && input_number > 0 )
   {
      return 0.08;
   }
   else if(input_number >= -0.08 && input_number < 0 )
   {
      return -0.08;
   }
   return input_number;
}


void String_decode(String data,float* in_arr)
{
  String str="";
  int counter = 0;
  for(int i=0;i<data.length();i++)
  {
    if(data[i] == '/')
    {
      
      in_arr[counter] = str.toFloat();
      counter++;
      str = "";
      continue;
    }
    str = str + data[i];
  }


}




/*************************************************************************************************************************/
// MOTOR CONTROL function

void motor_control(float x_raw,float y_raw,float rot_raw)
{

 
    

    // Serial.print(x);
    // Serial.print(" ");
    // Serial.print(y);
    // Serial.print(" ");
    // Serial.println(rot);

    // Then calculate the speed for each wheel and set the motor to that speed
    int front_left  =   (812 * (x_raw + y_raw + rot_raw));    // motor 1
    int front_right =   (812 * (x_raw - y_raw - rot_raw*));    // motor 2
    int back_left   =   (812 * (x_raw - y_raw + rot_raw));    // motor 3 
    int back_right  =   (812 * (x_raw + y_raw - rot_raw));    // motor 4  
    
    front_left  = Check_motor_value(front_left);
    front_left  = Check_motor_value(front_right);
    front_left  = Check_motor_value(back_left);
    front_left  = Check_motor_value(back_right);
    // Serial.print(front_left);
    // Serial.print(" ");
    // Serial.print(front_right);
    // Serial.print(" ");
    // Serial.print(back_left);
    // Serial.print(" ");
    // Serial.println(back_right);
    
    Set_Single_Motor(front_left ,PWM_1,IN1_1,IN2_1);
    Set_Single_Motor(front_right,PWM_2,IN1_2,IN2_2);
    Set_Single_Motor(back_left  ,PWM_3,IN1_3,IN2_3);
    Set_Single_Motor(back_right ,PWM_4,IN1_4,IN2_4);

}

int Check_motor_value(int input_number)
{
   if(input_number > 254 )
   {
      return 254;
   }
   else if(input_number < -254 )
   {
      return -254;
   }
   return input_number;
}

void Set_Single_Motor( int pwmVal, int pwm, int in1, int in2){
  // Serial.println(pwmVal);
  if(pwmVal > 0)      // motor turn forward
  { 
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(pwmVal < 0) // motor turn backward
  {
    pwmVal = pwmVal*(-1);
    analogWrite(pwm,pwmVal);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else                // motor dont turn
  {
    analogWrite(pwm,0);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

/*****************************************************************************************************/
// READ ENCODER PULSE

void readEncoder_A1(){
  // Read encoder B when ENC1 rises
  int b = digitalRead(ENC2_1);
  if(b>0){
    // If B is high, increase the value of pulse
    pulse_val_1++;
    return;
  }
  // else decrease pulse value of encoder 1
  pulse_val_1--;

}

void readEncoder_A2(){
  // Read encoder B when ENC1 rises
  int b = digitalRead(ENC2_2);
  if(b>0){
    // If B is high, increase the value of pulse
    pulse_val_2--;
    return;
  }
  // else decrease pulse value of encoder 2
  pulse_val_2++;
}

void readEncoder_A3(){
  // Read encoder B when ENC1 rises
  int b = digitalRead(ENC2_3);
  if(b>0){
    // If B is high, increase the value of pulse
    pulse_val_3++;
    return;
  }
  // else decrease pulse value of encoder 3
  pulse_val_3--;
}

void readEncoder_A4(){
  // Read encoder B when ENC1 rises
  int b = digitalRead(ENC2_4);
  if(b>0){
    // If B is high, increase the value of pulse
    pulse_val_4--;
    return;
  }
  // else decrease pulse value of encoder 2
  pulse_val_4++;
}


//**********************************************************************************************
// IMU_get_data function
void IMU_Get_Data_func()
{
       /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  IMU_x_accel_data= a.acceleration.x - x_accel_calib_rate;
  IMU_y_accel_data= a.acceleration.y - y_accel_calib_rate;
  IMU_yaw_data= g.gyro.z - yaw_vel_calib_rate ;
  /* Print out the values */
}

