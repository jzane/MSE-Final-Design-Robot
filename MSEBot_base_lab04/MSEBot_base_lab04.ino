#include <uSTimer2.h>

#include <I2CEncoder.h>

#include <CharliePlexM.h>

/*

  MSE 2202 MSEBot base code for Labs 3 and 4
  Language: Arduino
  Authors: Michael Naish and Eugen Porter
  Date: 15/01/18

  Rev 1 - Initial version

*/

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;
Servo servo_GripMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_GripMotor;

// Uncomment keywords to enable debugging output

#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 10;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

//used for line tracking code (mode 1)
boolean pauseHere;

int stageCounter = 0;
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 180;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Zero = 80;          //  "
const int ci_Grip_Motor_Closed = 80;       //  "
const int ci_Arm_Servo_Retracted = 55;      //  "
const int ci_Arm_Servo_Extended = 120;      //  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 100; // May need to adjust this
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned long ul_Left_Motor_Position;
unsigned long ul_Right_Motor_Position;
unsigned long ul_Grip_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
boolean grab_object_mode = false; //used for lab04, sets robot into target aquisition mode
boolean target_aquired = false; //usef for lab04, to determine which stage of target aquisition bot is in
boolean coarse_target_direction = false; //used for lab04, coarse grain direction to target

int x_degrees_turn_position = 10; //value associated with reading a 90 degree turn
int light_sensor_data = 0; //globabl variable to hold light sensor output data
int light_tolerance = 0; //might need to change this
int light_sensor_bright = 300;
int distance_to_object = 10; //EXPERIMENTAL VALUE
int arm_target_length = 20; //value of arm length
unsigned long max_light_position = 0;
int which_case = 0;
unsigned long current_position = 0;

void setup() {

  pauseHere = true;


  Wire.begin();	      // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);
  servo_GripMotor.write(ci_Grip_Motor_Zero);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);
  digitalWrite(ci_Motor_Enable_Switch, HIGH); //pullup resistor

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_GripMotor.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  // set up line tracking sensors
  pinMode(ci_Right_Line_Tracker, INPUT);
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        readLineTrackers();
        Ping();
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Stop);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        encoder_GripMotor.zero();
        ui_Mode_Indicator_Index = 0;
        Serial.print("light: ");
        Serial.println(analogRead(A3));
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          readLineTrackers();

#ifdef DEBUG_ENCODERS
          ul_Left_Motor_Position = encoder_LeftMotor.getPosition();
          ul_Right_Motor_Position = encoder_RightMotor.getPosition();
          ul_Grip_Motor_Position = encoder_GripMotor.getPosition();

          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getPosition());
          Serial.print(", R: ");
          Serial.print(encoder_RightMotor.getPosition());
          Serial.print(", G: ");
          Serial.println(ul_Grip_Motor_Position, DEC);
#endif

          // set motor speeds
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed - ui_Left_Motor_Offset, 1600, 2100);
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed - ui_Right_Motor_Offset, 1600, 2100);

          /***************************************************************************************
             Add line tracking code here.
             Adjust motor speed according to information from line tracking sensors and
             possibly encoder counts.

             Line tracking code added Jan 19, 2016 by Julian Zane
            /*************************************************************************************/


          if (bt_Motors_Enabled)
          {

            if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
            {
              servo_LeftMotor.writeMicroseconds(1700);
              servo_RightMotor.writeMicroseconds(1600);
            }

            if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
            {
              servo_LeftMotor.writeMicroseconds(1700);
              servo_RightMotor.writeMicroseconds(1700);
            }

            if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
            {
              servo_LeftMotor.writeMicroseconds(1600);
              servo_RightMotor.writeMicroseconds(1700);
            }

            //if left and middle see the line
            if ((ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)))
            {
              servo_LeftMotor.writeMicroseconds(1700);//1900 is default
              servo_RightMotor.writeMicroseconds(1700);
            }


            //if right and middle see the line
            if ((ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance) && (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))))
            {
              servo_LeftMotor.writeMicroseconds(1700);
              servo_RightMotor.writeMicroseconds(1700);
            }

            if ((ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance) && (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))))
            {
              //all three sensors are reading light
              servo_LeftMotor.writeMicroseconds(200); //stop motors
              servo_RightMotor.writeMicroseconds(200);
              //ui_Robot_State_Index = 0; //resets the robot to mode 0 (used for lab03)
              if (which_case == 0)
              {
                ui_Robot_State_Index = 5; //bumbs into case 5
                break;
              }
              else if (which_case == 1)
              {
                ui_Robot_State_Index = 7;
              }

            }

            if (!(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && !(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && !(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)))
            {
              //all three sensors are reading light
              servo_LeftMotor.writeMicroseconds(200); //stop motors
              servo_RightMotor.writeMicroseconds(200);
              //ui_Robot_State_Index = 0; //resets the robot to mode 0 (used for lab03)

            }


          }//end if motors enabled

#ifdef DEBUG_MOTORS
          Serial.print("Motors: Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(" , Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(" . Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
          // ui_Mode_Indicator_Index = 1; // remember to chage this back to 1 for logiacal flow
          which_case = 0;
        }
        break;
      }

    case 2:    //Calibrate line tracker light levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Light = 0;
            ui_Middle_Line_Tracker_Light = 0;
            ui_Right_Line_Tracker_Light = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Light Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Light, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Light, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Light, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
            EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 2;
        }
        break;
      }

    case 3:    // Calibrate line tracker dark levels after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            ui_Left_Line_Tracker_Dark = 0;
            ui_Middle_Line_Tracker_Dark = 0;
            ui_Right_Line_Tracker_Dark = 0;
            ul_Calibration_Time = millis();
            ui_Cal_Count = 0;
          }
          else if ((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
          {
            ul_Calibration_Time = millis();
            readLineTrackers();
            ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
            ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
            ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
            ui_Cal_Count++;
          }
          if (ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
          {
            ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
            ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
            Serial.print("Dark Levels: Left = ");
            Serial.print(ui_Left_Line_Tracker_Dark, DEC);
            Serial.print(", Middle = ");
            Serial.print(ui_Middle_Line_Tracker_Dark, DEC);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Line_Tracker_Dark, DEC);
#endif
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
            EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
          ui_Mode_Indicator_Index = 3;
        }
        break;
      }

    case 4:    //Calibrate motor straightness after 3 seconds.
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            ul_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            ul_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (ul_Left_Motor_Position > ul_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (ul_Left_Motor_Position - ul_Right_Motor_Position) / 3.43;
              ui_Left_Motor_Offset = 0;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (ul_Right_Motor_Position - ul_Left_Motor_Position) / 3.43;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Right = ");
            Serial.print(ui_Right_Motor_Offset);
            Serial.print(", Left = ");
            Serial.println(ui_Left_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_ENCODERS
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
          ui_Mode_Indicator_Index = 4;
        }
        break;
      }



    /*
       Case 5 gets triggered as soon as the 3 light sensors all read light the first time
      turns 90 degrees left
      pop into case 1 again to follow line until bot sees 3 'light' sensors again
      then pops into case 7
      NOTE: use encoder.getRawPosition() b/c it reads the raw ticks, doesn't multiply by a factor > 1 --
      like encoder.getPosition() does
    */


    /*
      TO DO: use false zeros and global variables instead of zero()
       encoders stop sending data when code bumps into case 5
       this results in getting stuck in the while loop indefinetely
       b/c value of right encoder is stuck at 0 and is always less than the target value
    */
    case 5:    //Light Sensor mode
      {
        /*
           experimenting if using false zeros will keep encoders working when control is passed to case 5 code
           this means not using encoder.zero()
           list of variables used in case 5:
           start_case5_position
        */
        //encoder_RightMotor.zero(); //zeros the tick count of right encoder
        delay(100);

        unsigned long start_case5_position = encoder_RightMotor.getRawPosition; //start_case5_position holds current encoder pos
        /*
          while loop to hold code here to make full left turn
          holds until right encoder reads
          looks at relative position between current pos and starting pos (false zero)
        */
        while ((encoder_RightMotor.getRawPosition() - start_case5_position) <= 1.37)
        {
          servo_LeftMotor.writeMicroseconds(1600); //left turn
          servo_RightMotor.writeMicroseconds(200);
        }
        //if here, now pointing left
        servo_LeftMotor.writeMicroseconds(200); //stop
        servo_RightMotor.writeMicroseconds(200);
        
        which_case = 1;//set to 1 so next time breaks out of case 1, goes into case 7
        ui_Robot_State_Index = 1;  //when breaks from this case, it will go back into 1
        break;
      }








    case 6:
      //case 6 turns left at end of course, locates the platform and drops it off
      {
        ul_Right_Motor_Position = 0; //set right encoder to zero
        //might have to change the way I used ul_Right_Motor_Position
        while (ul_Right_Motor_Position <= x_degrees_turn_position)
        {
          //turn left until right encoder registers 90 degree turn
          // turnLeft();
          ul_Right_Motor_Position = encoder_RightMotor.getPosition(); //this should be cumulative
        }
        //now generally facing platform
        //stop_motors(); //stops motors from spinning contimuously

        while ((ul_Echo_Time / 58) >= arm_target_length)
        { //while driving towards target platform
          Ping();
          servo_LeftMotor.writeMicroseconds(1600); //driving straight
          servo_RightMotor.writeMicroseconds(1600);
        }//end while
        //  stop_motors();

        //extend arm our towards platform
        for (int pos = 60; pos < 121; pos++)
        {
          servo_ArmMotor.write(pos);
          delay(10);
        }

        //let go of grip motor slowly
        for (int pos = 40; pos <= 180; pos++)
        {
          servo_ArmMotor.write(pos);
          delay(10);
        }
        break;
      }//end case 5

    /*
       case 7 starts when bot is on the yellow line, we know the target is somewhere in front of us
      start by moving bot to the extreme right edge

    */
    case 7:
      {
        /*
            left_turn_while_scanning();
          servo_ArmMotor.write(ci_Arm_Servo_Retracted);
          delay(1000);
          servo_GripMotor.write(ci_Grip_Motor_Open);
          delay(1000);
          for (int ArmMotorAngle = ci_Arm_Servo_Retracted; ArmMotorAngle < 120; ArmMotorAngle = ArmMotorAngle + 1)
          {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(100);
          }
          servo_ArmMotor.write(ci_Arm_Servo_Extended);
          delay(1000);
          servo_GripMotor.write(ci_Grip_Motor_Closed);
          delay(1000);
          for (int ArmMotorAngle = ci_Arm_Servo_Extended; ArmMotorAngle > 55; ArmMotorAngle = ArmMotorAngle - 1)
          {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(100);
          }
        */

        //start by moving to the right
        encoder_RightMotor.zero();
        while (encoder_RightMotor.getRawPosition() >= -0.15) //arbitrary value to get robot to turn right to some value
        {
          servo_LeftMotor.writeMicroseconds(200); //right turn
          servo_RightMotor.writeMicroseconds(1350);
        }
        servo_LeftMotor.writeMicroseconds(200); //full stop
        servo_RightMotor.writeMicroseconds(200);
        //now robot in "full right" position

        /*
             extend the arm a little so the light sensor is at the same height as the target
          initially using angle of 60 MAY HAVE TO CHANGE THIS
        */
        for (int ArmMotorAngle = ci_Arm_Servo_Retracted; ArmMotorAngle < 60; ArmMotorAngle++)
        {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(10);
        }

        //arm is half extended and ready to sweep for light sensor values

        encoder_RightMotor.zero(); //zero encoder again
        light_sensor_data = analogRead(A3); //read value from light sensor, store it in global variable


        while (encoder_RightMotor.getRawPosition() <= 0.3) //arbitrary value to get robot to turn to full left position
        {
          current_position = encoder_RightMotor.getRawPosition(); //sets current_position as the current encoder 'tick reading'
          while (encoder_RightMotor.getRawPosition() - current_position <= 0.1) //moves the bot a little bit to the left each time
          {
            servo_LeftMotor.writeMicroseconds(1000); //full speed left
            servo_RightMotor.writeMicroseconds(2000);
          }
          //stop motors to take another reading of light sensor and compare it with the current max value
          servo_LeftMotor.writeMicroseconds(200); //full stop
          servo_RightMotor.writeMicroseconds(200);
          //bot has rotated a little bit to the left, corresponding to a number of encoder ticks

          //this 'if statement' finds at what encoder position corresponds to a maximum light sensor reading (ie: on target)
          if (analogRead(A3) < light_sensor_data) //if current light sensor reading is greater than the
          {
            max_light_position = encoder_RightMotor.getRawPosition(); //stores this encoder position
          }
        }

        //here, robot is in full left position, with encoder values quite high, and the position of
        //max light sensor value stored in 'max_light_position'
        //turn right until the encoder is set to max_light_position
        //this corresponds to the arm beingon target
        while (encoder_RightMotor.getRawPosition() >= max_light_position) //this is assuming the encoders decrease when moving in reverse
        {
          servo_LeftMotor.writeMicroseconds(2000); //full speed right
          servo_RightMotor.writeMicroseconds(1000);
        }

        servo_LeftMotor.writeMicroseconds(200); //full stop
        servo_RightMotor.writeMicroseconds(200);
        //now robot is pointing at the target

        /*
             JUSTIN'S CODE FOR GRABBING THE TARGET
        */
        servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        delay(1000);
        servo_GripMotor.write(ci_Grip_Motor_Open);
        delay(1000);
        for (int ArmMotorAngle = ci_Arm_Servo_Retracted; ArmMotorAngle < 120; ArmMotorAngle = ArmMotorAngle + 1)
        {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(10);
        }
        servo_ArmMotor.write(ci_Arm_Servo_Extended);
        delay(1000);
        servo_GripMotor.write(ci_Grip_Motor_Closed);
        delay(1000);
        for (int ArmMotorAngle = ci_Arm_Servo_Extended; ArmMotorAngle > 55; ArmMotorAngle = ArmMotorAngle - 1)
        {
          servo_ArmMotor.write(ArmMotorAngle);
          delay(20);
        }


        //now we have the target in the grip
        //then we have to back up, turn to the right, then bump back into case 1
        //servo_LeftMotor.writeMicroseconds(1000); //full reverse
        //servo_RightMotor.writeMicroseconds(1000);

        /*
             NOTE: MIGHT NOT BE REVERSING STRAIGHT, TO SOLVE THIS WE COULD
          RECORD ENCODER POSITION WHEN WE INITIALLY GET UP TO THE LINE, THEN USE FALSE ZEROS INSTEAD OF ZEROING THE ENCODERS A LOT
          FIRST MAKE SURE WE CAN GET THE ABOVE CODE TO WORK BEFORE WE START USING FALSE ZEROS
        */

        encoder_RightMotor.zero(); //zero encoders again

        /*
             reverse a small amount
        */
        while (encoder_RightMotor.getRawPosition() <= 1.5) //NEED TO CHANGE 1.5 TO A SUITABLE VALUE
        {
          servo_LeftMotor.writeMicroseconds(1000); //full speed reverse
          servo_RightMotor.writeMicroseconds(1000);
        }

        encoder_RightMotor.zero(); //zero encoders again

        /*
             turn right 90 degrees
          use the same encoder value from above (when we turned left 90 degrees
          but it will have to be the negative of that value b/c right motor is reversing)
        */
        while (encoder_RightMotor.getRawPosition() >= -1.5) //NEED TO CHANGE 1.5 TO A SUITABLE VALUE
        {
          servo_LeftMotor.writeMicroseconds(1000); //full speed right
          servo_RightMotor.writeMicroseconds(2000);
        }


        /*
             here we want to be pointed to the left of the track
           sweeping from left to right until the right line tracking sensor registers light
           then we pop back into case 1 to follow the path until the drop off point
        */

        encoder_RightMotor.zero();
        while (encoder_RightMotor.getRawPosition() <= 0.2) //MAY NEED TO CHANGE THIS VALUE
        {
          servo_LeftMotor.writeMicroseconds(1000); //full speed right
          servo_RightMotor.writeMicroseconds(2000);
        }

        servo_LeftMotor.writeMicroseconds(200); //full stop
        servo_RightMotor.writeMicroseconds(200);

        //read line trackers
        readLineTrackers();

        /*
          holds code here until the right line tracker reads light
          sweeps bot from left to right searching for light line
        */
        while (!(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)))
        {
          readLineTrackers();

          encoder_RightMotor.zero(); //zero encoder again
          current_position = encoder_RightMotor.getRawPosition();
          while (encoder_RightMotor.getRawPosition() - current_position <= 0.05) //moves the bot a little bit to the left each time
          {
            servo_LeftMotor.writeMicroseconds(2000); //full speed right
            servo_RightMotor.writeMicroseconds(1000);
          }
          //stop motors to take another reading of line trackers
          servo_LeftMotor.writeMicroseconds(200); //full stop
          servo_RightMotor.writeMicroseconds(200);

        }
        //breaks into case 1 again to drive the rest of the course

        ui_Robot_State_Index = 1;
        break;
      }




  }//end switch

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Robot_State_Index, DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
                                          (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

  if (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data, DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data, DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif

}

// measure distance to target using ultrasonic sensor
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}

void turnLeft()
{
  servo_LeftMotor.writeMicroseconds(200);
  servo_RightMotor.writeMicroseconds(1600);
}

void turnRight()
{
  servo_LeftMotor.writeMicroseconds(1600);
  servo_RightMotor.writeMicroseconds(200);
}

void stop_motors()
{
  servo_LeftMotor.writeMicroseconds(200);
  servo_RightMotor.writeMicroseconds(200);
}

void read_light_sensor()
{
  light_sensor_data = analogRead(ci_Light_Sensor); //reads pin A3
}


/*
  turns left slowly while scanning the light sensor
  stops when light sensor gets close to a known value
  corresponding to pointing at target
*/

void left_turn_while_scanning()
{
  stop_motors();
  servo_LeftMotor.writeMicroseconds(2000);
  servo_RightMotor.writeMicroseconds(1000);
  delay(100);
  stop_motors();
  //temp_right_encoder = 0;
  //light_sensor_data();
  while (analogRead(ci_Light_Sensor) > (light_sensor_bright - light_tolerance))
  { //light sensor is reading darker than known target

    //turns until right encoder increases
    turnLeft();
    delay(10);
    //temp_right_encoder = encoder_RightMotor.getPosition()
    //end while
  }//end if
}


void TurnLeft()
{

  ul_Left_Motor_Position = 0; //set left encoder to 0
  ul_Right_Motor_Position = 0; //set right encoder to 0
  /*
    while(encoder_RightMotor.getPosition() <= 2)//90 degree value)
    {
    servo_LeftMotor.writeMicroseconds(2000);
    servo_RightMotor.writeMicroseconds(1000);
    ul_Right_Motor_Position = encoder_RightMotor.getPosition();
    Serial.println(ul_Right_Motor_Position);
    }
  */
  // {
  servo_LeftMotor.writeMicroseconds(2000);
  servo_RightMotor.writeMicroseconds(1000);
  delay(500);
  ui_Mode_Indicator_Index = 1;
}












