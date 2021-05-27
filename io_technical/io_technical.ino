#include <Wire.h>
#include <PixyI2C.h>
PixyI2C pixy;
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

#define x_robot 115
#define y_robot 131

int x_ball, y_ball, x_goal, y_goal, ball_angle, goal_angle, ball_direction, goal_direction;
int ball_distance, goal_distance;
int is_ball, is_goal;
int spin_cnt = 0, shoot_cnt = 0;
int buff[8];
int counter = 0;
int GY, speed = 255;
bool shoot_sens = false;
bool ball_shooted = false;
bool look_front = true;
bool game_started = true;

void setup() {
  initiate();
  display.begin(0x2, 0x3C);
  pixy.init();
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();
  
  Serial.begin(115200);
  Serial.write(0XA5);
  Serial.write(0X54);
  delay(500);
  Serial.write(0XA5);
  Serial.write(0X51);
  
}

void loop() {
  readSensors();
  read_pixy();
  readGY();
  print_all();
  look_front = true;
  if(shoot_sens) 
  {
    spin(true);
    if(goal_distance > 60) move(goal_direction);
    else
    {
      look_front = false;
      if(goal_direction == 0) {stop();shoot();}
      else if(goal_direction < 8) motor(70, 70, 70, 70);
      else if(goal_direction >=8) motor(-70, -70, -70, -70);
    }
  }
  else if(is_ball) 
  {
    ball_shooted = false;
    if(ball_distance < 50 && (ball_direction < 4 || ball_direction > 12)) spin(true);
    else spin(false);
    shift(ball_direction);
  }
  else        stop();
}

void shoot()
{
  if(!ball_shooted && game_started)
  {
    digitalWrite(PC15, 1);
    delay(300);
    digitalWrite(PC15, 0);
    ball_shooted = true;
  }
}
void spin(bool state)
{
  if(state && game_started) digitalWrite(PC14, 1);
  else      digitalWrite(PC14, 0);
}
void readSensors()
{
  if(digitalRead(PB5))
  {
    digitalWrite(PC13,0);
    Serial.write(0XA5);
    Serial.write(0X55);
    while(digitalRead(PB5));
  }
  if(digitalRead(PA15))
  {
    digitalWrite(PC13,0);
    game_started = !game_started;
    while(digitalRead(PA15));
  }
  shoot_sens = !digitalRead(PB0);
  if(shoot_sens) 
  {
    if(spin_cnt < 4)
      digitalWrite(PA11, 1);
    else if(spin_cnt < 8)
      digitalWrite(PA11, 0);
    else
      spin_cnt = 0;
    spin_cnt++;
  }
  else           digitalWrite(PA11, 0);
}
void stop()
{
  if(look_front)  motor(GY, GY, GY, GY);
  else            motor(0,0,0,0);
}
void shift(int direction)
{
  if(direction == 0)       move(direction);
  else if(direction < 2)   move(direction+1);
  else if(direction > 14)  move(direction-1);
  else if(direction <= 8)  move(direction+3);
  else if(direction > 8)   move(direction-3);
}
void motor(int L1, int L2, int R2, int R1)
{
  if(look_front)
  {
    L1 += GY;
    L2 += GY;
    R1 += GY;
    R2 += GY;
  }
  if(!game_started)
  {
    L1 = 0;
    L2 = 0;
    R1 = 0;
    R2 = 0;
  }
   
  L1 = L1*65535/255;
  L2 = L2*65535/255;
  R1 = R1*65535/255;
  R2 = R2*65535/255;
  if(L1 > 65535) L1 = 65535;
  if(L2 > 65535) L2 = 65535;
  if(R1 > 65535) R1 = 65535;
  if(R2 > 65535) R2 = 65535;
  if(L1 < -65535) L1 = -65535;
  if(L2 < -65535) L2 = -65535;
  if(R1 < -65535) R1 = -65535;
  if(R2 < -65535) R2 = -65535;
  
  if(L1 >= 0){
    digitalWrite(PB15,0);
    pwmWrite(PA8,L1);
  }
  else{
    digitalWrite(PB15,1);
    pwmWrite(PA8,L1+65535);
  }
  if(L2 >= 0){
    digitalWrite(PB14,0);
    pwmWrite(PB8,L2);
  }
  else{
    digitalWrite(PB14,1);
    pwmWrite(PB8,L2+65535);
  }
  if(R2 >= 0){
    digitalWrite(PB13,0);
    pwmWrite(PB7,R2);
  }
  else{
    digitalWrite(PB13,1);
    pwmWrite(PB7,R2+65535);
  }
  if(R1 >= 0){
    digitalWrite(PB12,0);
    pwmWrite(PB6,R1);
  }
  else{
    digitalWrite(PB12,1);
    pwmWrite(PB6,R1+65535);
  }
}
void move(int direction)
{
  if(direction == 0)      motor(speed   , speed   , -speed  , -speed   );
  if(direction == 1)      motor(speed   , speed/2 , -speed  , -speed/2 );
  if(direction == 2)      motor(speed   , 0       , -speed  , 0        );
  if(direction == 3)      motor(speed   , -speed/2, -speed  , speed/2  );
  if(direction == 4)      motor(speed   , -speed  , -speed  , speed    );
  if(direction == 5)      motor(speed/2 , -speed  , -speed/2, speed    );
  if(direction == 6)      motor(0       , -speed  , 0       , speed    );
  if(direction == 7)      motor(-speed/2, -speed  , speed/2 , speed    );    
  
  if(direction == 8)      motor(-speed  , -speed  , speed   , speed    );
  
  if(direction == 9)      motor(-speed   , -speed/2, speed   , speed/2 );
  if(direction == 10)     motor(-speed   , 0       , speed   , 0       );
  if(direction == 11)     motor(-speed   , speed/2 , speed   , -speed/2);
  if(direction == 12)     motor(-speed   , speed   , speed   , -speed  );
  if(direction == 13)     motor(-speed/2 , speed   , speed/2 , -speed  );
  if(direction == 14)     motor(0        , speed   , 0       , -speed  );
  if(direction == 15)     motor(speed/2  , speed   , -speed/2, -speed  );        
}
void print_all()
{
  display.clearDisplay();
  display.drawCircle(80, 32, 12, WHITE);
  if(is_ball)
    display.fillCircle(80 + sin(ball_angle * PI/180) * 20,32 - cos(ball_angle * PI/180) * 20 , 3, WHITE);
  if(is_goal)
    display.drawCircle(80 + sin(goal_angle * PI/180) * 20,32 - cos(goal_angle * PI/180) * 20 , 5, WHITE);
  display.drawLine(80 + sin(GY * PI/180) * 9,32 - cos(GY * PI/180) * 9, 
                   80 - sin(GY * PI/180) * 9,32 + cos(GY * PI/180) * 9, WHITE);
  display.fillCircle(80 - sin(GY * PI/180) * 9,32 + cos(GY * PI/180) * 9, 2, WHITE);
  display.setCursor(0,0);
  display.print("x:");
  display.println(x_ball);
  display.print("y:");
  display.println(y_ball);
  display.print("a:");
  display.println(ball_angle);
  display.print("GY:");
  display.println(GY);
  display.print("dir:");
  display.println(ball_direction);
  display.print("dis:");
  display.println(ball_distance);
  display.display();
}
void readGY()
{
  if(digitalRead(PB5)){
    Serial.write(0XA5);
    Serial.write(0X55);
    while(digitalRead(PB5));
  }
  Serial.write(0XA5);
  Serial.write(0X51);
  while (true) {   
    buff[counter] = Serial.read();
    if(counter == 0 && buff[0] != 0xAA) break;
    counter++;       
    if(counter==8)
    {   
    counter=0;                 
    if(buff[0]==0xAA && buff[7]==0x55)
      {         
       GY=(int16_t)(buff[1]<<8|buff[2])/100.00; 
      } 
    }    
  }
}
void read_pixy()
{
  uint16_t blocks;
  blocks = pixy.getBlocks();
  is_ball = false;
  is_goal = false;
  if(blocks)
  {
    
    for(int i=0; i<blocks; i++)
    {
      if(pixy.blocks[i].signature == 1) // Ball
      {
        y_ball = pixy.blocks[i].x;
        x_ball = pixy.blocks[i].y;
        ball_angle = get_angle(x_ball, y_ball);
        ball_direction = get_direction(ball_angle);
        ball_distance = sqrt(pow(x_ball - x_robot, 2) + pow(y_ball - y_robot, 2));
        is_ball = true;
      }
    }
    
    for(int i=0; i<blocks; i++)
    {
      if(pixy.blocks[i].signature == 2) // Ball
      {
        y_goal = pixy.blocks[i].x;
        x_goal = pixy.blocks[i].y;
        goal_angle = get_angle(x_goal, y_goal);
        goal_direction = get_direction(goal_angle);
        goal_distance = sqrt(pow(x_goal - x_robot, 2) + pow(y_goal - y_robot, 2));
        is_goal = true;
      }
    }
  }
}
int get_angle(int x, int y)
{
  int angle = atan2(y - y_robot, x - x_robot)*180/PI;
  angle += 90;
  if(angle < 0) angle += 360;
  return angle;
}
int get_direction(int angle)
{
  int direction;
  for(int i = 0; i < 16; i++)
  {         
    if(angle <= 11.25) direction = 0; 
    else if(angle >= 348.5) direction = 0;
    else if((angle - 11.25 >= i * 22.5) && (angle-11.25 < (i+1) * 22.5))  direction = i + 1;
  }
  return direction;
}
void initiate()
{
  pinMode(PA12, OUTPUT); // LED
  pinMode(PA11, OUTPUT); // LED
  pinMode(PC14, OUTPUT); // Spin
  pinMode(PC15, OUTPUT); // Shoot
  
  pinMode(PB12,OUTPUT);
  pinMode(PB13,OUTPUT);
  pinMode(PB14,OUTPUT);
  pinMode(PB15,OUTPUT);

  pinMode(PA8,PWM);
  pinMode(PB8,PWM);
  pinMode(PB7,PWM);
  pinMode(PB6,PWM);
  motor(0,0,0,0);
  for(int i=0; i<3; i++)
  {
    digitalWrite(PA12, 1);
    digitalWrite(PA11, 0);
    delay(100);
    digitalWrite(PA12, 0);
    digitalWrite(PA11, 1);
    delay(100);
  }
  digitalWrite(PA12, 0);
  digitalWrite(PA11, 0);
}
