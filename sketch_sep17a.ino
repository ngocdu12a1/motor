#ifndef ARDPRINTF
#define ARDPRINTF
#define ARDBUFFER 16
#include <stdarg.h>
#include <Arduino.h>

int  sensor1 = 14;
int  sensorIR1  = 2;
int  sensorIR2  = 3;
int  sensorIR3  = 4;
int  sensorIR4  = 5;
int  sensorIR5  = 12;

int ENA = 11;
int motorInput1 = 10;
int motorInput2 = 9;
int motorInput3 = 8;
int motorInput4 = 7;
int ENB = 6;
bool first = false;
int sensor[5] = {0, 0, 0, 0, 0}; // giữ giá trị cảm biến đọc vào

//
float  error = 0, P = 0, I = 0, D = 0, PID_value = 0, previous_error = 0;
float  Kp = 25, Ki = 0, Kd = 6;
int speed_motor = 95, left_speed = 0, right_speed = 0;

int ardprintf(char *, ...);


void setup() {
  Serial.begin(9600);
  pinMode(sensor1, INPUT);
   
  pinMode(sensorIR1, INPUT);
  pinMode(sensorIR2, INPUT);
  pinMode(sensorIR3, INPUT);
  pinMode(sensorIR4, INPUT);
  pinMode(sensorIR5, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);

  
  // set direction
  digitalWrite(motorInput1, 1);
  digitalWrite(motorInput2, 0);
  digitalWrite(motorInput3, 1);
  digitalWrite(motorInput4, 0);

  
}

void loop() {
  calculate_error();
  calculate_PID();
  control_motors();

  delay(5);
}
 

void calculate_error(){
    sensor[0] = digitalRead(sensorIR1);
    sensor[1] = digitalRead(sensorIR2);
    sensor[2] = digitalRead(sensorIR3);
    sensor[3] = digitalRead(sensorIR4);
    sensor[4] = digitalRead(sensorIR5);

    if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) error = 4;       // 0 0 0 0 1
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) error = 3;  // 0 0 0 1 1
    else if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) error = 2;  // 0 0 0 1 0
    else if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) error = 1;  // 0 0 1 1 0
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) error = 0;  // 0 0 1 0 0
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0) error = -1;  // 0 1 1 0 0
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 0) error = -2;  // 0 1 0 0 0
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1) error = -3;  // 1 1 0 0 0
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1) error = -4;  // 1 0 0 0 0
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) error = previous_error;  // 0 0 0 0 0

    if(sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) error = 4;       // 0 0 0 0 1
    else if(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) error = 3;  // 0 0 0 1 1
    else if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) error = 2;  // 0 0 0 1 0
    else if(sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1) error = 1;  // 0 0 1 1 0
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1) error = 0;  // 0 0 1 0 0
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1) error = -1;  // 0 1 1 0 0
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 1) error = -2;  // 0 1 0 0 0
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) error = -3;  // 1 1 0 0 0
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0) error = -4;  // 1 0 0 0 0
    else if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) error = previous_error;  // 0 0 0 0 0

}

void calculate_PID(){
    P = error;
    I = I + error;
    D = error - previous_error;

    PID_value = Kp * P + Ki * I + Kd * D;
    previous_error = error;
}

void control_motors(){
  left_speed = speed_motor - PID_value;
  right_speed = speed_motor + PID_value;

  left_speed = constrain(left_speed, 0, 225);
  right_speed = constrain(right_speed, 0, 225);

  analogWrite(ENA, right_speed);
  analogWrite(ENB, left_speed);
}


int ardprintf(char *str, ...)
{
  int i, count=0, j=0, flag=0;
  char temp[ARDBUFFER+1];
  for(i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++;

  va_list argv;
  va_start(argv, count);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j=0;
      temp[0] = '\0';

      switch(str[++i])
      {
        case 'd': Serial.print(va_arg(argv, int));
                  break;
        case 'l': Serial.print(va_arg(argv, long));
                  break;
        case 'f': Serial.print(va_arg(argv, double));
                  break;
        case 'c': Serial.print((char)va_arg(argv, int));
                  break;
        case 's': Serial.print(va_arg(argv, char *));
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0) 
      {
        temp[ARDBUFFER] = '\0';
        Serial.print(temp);
        temp[0]='\0';
      }
    }
  };
  Serial.println();
  return count + 1;
}
#undef ARDBUFFER
#endif
