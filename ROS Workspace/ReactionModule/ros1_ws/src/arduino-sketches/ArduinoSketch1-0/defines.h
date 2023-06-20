#ifndef defines_h
#define defines_h


//Arduino Pins

/*
//Battery Pins
#define BUZZER_PIN  8
#define BAT_PROBE_LED 13
#define BATTERY_PIN A0
*/

//Eyes Pins
#define R_LOW_SERVO_PIN 4
#define R_HIGH_SERVO_PIN 5
#define L_LOW_SERVO_PIN 7
#define L_HIGH_SERVO_PIN 2

//Body Pins

#define BODY_RIGH_SERVO_PIN 10
#define BODY_LEFT_SERVO_PIN 11
#define BODY_BACK_SERVO_PIN 12
#define BODY_BACK_DOWN_SERVO_PIN 9

//Hardware Parts defines


#define SPEED_MULTIPLIER 5

/*
//battery defines
#define BAT_CHK_TIMEOUT_TRH 10000    // publishes battery data every 10 secs.
#define BEEPING_INTERVAL 5000
#define LOW_BATTERY_VOLTAGE 21
#define VOLTAGE_TRH 3.70  //3.70 correspond to battery level at 21V.
*/

#define NOTE_TO_PLAY 262 //corresponds to C4
#define NOTE_DURATION 1000

//body defines
// back down
#define NORMAL_B_D 110
#define FRONT_B_D 90
#define BACK_B_D 170
#define RIGHT_B_D 90
#define LEFT_B_D 40
// front right
#define NORMAL_FR 120
#define FRONT_FR 60
#define BACK_FR 180
#define RIGHT_FR 50
#define LEFT_FR 180
// front left
#define NORMAL_FL 80
#define FRONT_FL 70
#define BACK_FL 40
#define RIGHT_FL 0
#define LEFT_FL 180
// back
#define NORMAL_B 60
#define FRONT_B 0
#define BACK_B 60
#define RIGHT_B 110
#define LEFT_B 110


/*
//servo angles
#define FR_OPEN 180
#define FR_MIDDLE 110
#define FR_CLOSE  0
#define FL_OPEN 0
#define FL_MIDDLE 70
#define FL_CLOSE 180
#define B_OPEN 180
#define B_MIDDLE 110
#define B_CLOSE 0
#define B_D_OPEN 0
#define B_D_CLOSE 180
#define B_D_MIDDLE 90
*/

#define NORMAL 0
#define BOW 1
#define BOW_RIGHT 2
#define BOW_LEFT 3
#define BOW_BACK 4

//eyes defines
//right eye angles
#define STANDARD_LOW_R 80
#define STANDARD_HIGH_R 45//85

#define LOOK_RIGH_LOW_R 130
#define LOOK_RIGH_HIGH_R 60

#define LOOK_LEFT_LOW_R 60
#define LOOK_LEFT_HIGH_R 0

#define LOOK_DOWN_LOW_R 50
#define LOOK_DOWN_HIGH_R 90

#define LOOK_DOWN_LEFT_LOW_R 0
#define LOOK_DOWN_LEFT_HIGH_R 90

#define LOOK_DOWN_RIGHT_LOW_R 80
#define LOOK_DOWN_RIGHT_HIGH_R 120

#define LOOK_UP_LOW_R 135
#define LOOK_UP_HIGH_R 15

#define LOOK_UP_LEFT_LOW_R 100
#define LOOK_UP_LEFT_HIGH_R 0

#define LOOK_UP_RIGHT_LOW_R 180
#define LOOK_UP_RIGHT_HIGH_R 15

#define CROSS_LOW_R 50
#define CROSS_HIGH_R 20

#define DIVIDE_LOW_R 150
#define DIVIDE_HIGH_R 60


//left eye angles
#define STANDARD_LOW_L 60
#define STANDARD_HIGH_L 80

#define LOOK_RIGH_LOW_L  90
#define LOOK_RIGH_HIGH_L 120

#define LOOK_LEFT_LOW_L 0
#define LOOK_LEFT_HIGH_L 50

#define LOOK_DOWN_LOW_L 100
#define LOOK_DOWN_HIGH_L 30

#define LOOK_DOWN_LEFT_LOW_L 0
#define LOOK_DOWN_LEFT_HIGH_L 0

#define LOOK_DOWN_RIGHT_LOW_L 130
#define LOOK_DOWN_RIGHT_HIGH_L 100

#define LOOK_UP_LOW_L 30
#define LOOK_UP_HIGH_L 100

#define LOOK_UP_LEFT_LOW_L 0
#define LOOK_UP_LEFT_HIGH_L 100

#define LOOK_UP_RIGHT_LOW_L 50
#define LOOK_UP_RIGHT_HIGH_L 150

#define CROSS_LOW_L 90
#define CROSS_HIGH_L 90

#define DIVIDE_LOW_L 0
#define DIVIDE_HIGH_L 60

// eyes positions code
#define EYES_STANDARD 0
#define EYES_RIGH 1
#define EYES_LEFT 2
#define EYES_UP 3
#define EYES_DOWN 4
#define EYES_CROSS 5
#define EYES_DIVIDE 6
#define EYES_L_UP_R_DOWN 7
#define EYES_R_UP_L_DOWN 8
#define EYES_DANCE_STYLE1 9
#define EYES_DANCE_STYLE2 10
#define RIGHT_EYE_UP 11
#define RIGHT_EYE_DOWN 12
#define EYES_UP_DIVIDE 13

#endif
