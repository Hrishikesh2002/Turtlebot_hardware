#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define A1 26 // Left
#define B_1 27
#define P1 25
#define A2 14 // Right
#define B2 12
#define P2 13

#define pi 3.1416
#define DIAMETER 0.067
#define wheel_dist 0.195

#define LEFT 0
#define RIGHT 1
#define ClkWise 0
#define AClkWise 1

#define outputA_R 39
#define outputB_R 36
#define outputA_L 35
#define outputB_L 34

double counter_R = 0;
double counter_L = 0;
hw_timer_t* timer = NULL;
double Rcounter_cur = 0;
double Lcounter_cur = 0;
double Rcounter_prev = 0;
double Lcounter_prev = 0;
double rps_R = 0;
double rps_L = 0;
double counts_per_rev = 206;
double count_diff_L = 0;
double count_diff_R = 0;
double R_revs = 0;
double L_revs = 0; //The revolutions made in each interval between two consecutive timers for Left wheel
double R_dist = 0;
double L_dist = 0; //The distance travelled in each interval between two consecutive timers for Left wheel

double distance_travelled = 0;
double revs_total_R = 0;
double revs_total_L = 0;
double dist_left = 0;
double dist_right = 0;
double Theta_bot = 0;
double Omega_bot = 0;
double Omega_R = 0;
double Omega_L =0;
double Velocity_bot = 0;
double x_bot = 0;
double y_bot = 0;
double phi = 0;
double x=0, z=0;

void Differential_drive();



void IRAM_ATTR onTimer() {   // 100 ms timer

 
  Rcounter_cur = counter_R;    
  count_diff_R = Rcounter_cur - Rcounter_prev;
  R_revs = count_diff_R/counts_per_rev;
  rps_R = R_revs*10;
  Omega_R = rps_R * pi * 2;
  R_dist = R_revs * pi * DIAMETER;
  Rcounter_prev = Rcounter_cur;
  

  Lcounter_cur = counter_L;
  count_diff_L = Lcounter_cur - Lcounter_prev;
  L_revs = count_diff_L/counts_per_rev;
  rps_L = L_revs*10;
  Omega_L = rps_L * pi * 2;
  L_dist = L_revs * pi * DIAMETER;
  Lcounter_prev = Lcounter_cur;



  Differential_drive();



}

void updateCounter_R(){
   if(digitalRead(outputA_R)==HIGH)
  {
    counter_R++;
  }

  else
  {
    counter_R--;
  }

}

void updateCounter_L(){
   if(digitalRead(outputA_L)==LOW)
  {
    counter_L++;
  }

  else
  {
    counter_L--;
  }
  
}


void SetPins() {
  pinMode(A1, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(P1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(P2, OUTPUT);

  pinMode(outputA_R, INPUT);
  pinMode(outputB_R, INPUT);
  pinMode(outputA_L, INPUT);
  pinMode(outputB_L, INPUT); 
}

void SetTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);

  attachInterrupt(digitalPinToInterrupt(outputB_R), updateCounter_R, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB_L), updateCounter_L, RISING);
}

double pwm_L=0;
double pwm_R=0;

void driveMotor(int Location, double speed_curr) {

// speed_curr = map(speed_curr, -100, +100, -255, +255);

if(speed_curr>=0)
{
  if(Location){
    pwm_R = speed_curr;
    digitalWrite(A2, 1);
    digitalWrite(B2, 0);
    analogWrite(P2,pwm_R);
  }


  else{
    pwm_L = speed_curr;
    digitalWrite(A1, 1);
    digitalWrite(B_1,0);
    analogWrite(P1, pwm_L);
  }
}

if(speed_curr<0)
{
    if(Location){
    pwm_R = speed_curr;
    digitalWrite(A2, 0);
    digitalWrite(B2, 1);
    analogWrite(P2,(-1)*pwm_R);
  }


  else{
    pwm_L = speed_curr;
    digitalWrite(A1, 0);
    digitalWrite(B_1,1);
    analogWrite(P1, (-1)*pwm_L);
  }
}


}

void Differential_drive()
{
//  revs_total_L = counter_L/counts_per_rev;
//  revs_total_R = counter_R/counts_per_rev;
//
//  dist_left = revs_total_L * pi * DIAMETER;
//  dist_right = revs_total_R * pi * DIAMETER;
//
//  distance_travelled = (dist_left + dist_right)/2;

  Omega_bot = DIAMETER * (Omega_R - Omega_L)/(2 * wheel_dist );
  Velocity_bot = DIAMETER * (Omega_R + Omega_L)/(4);

  phi = ((R_dist - L_dist)/wheel_dist);

  double avg_dist = (R_dist + L_dist)/2;

  x_bot = x_bot + avg_dist * cos(Theta_bot + phi/2);
  y_bot = y_bot + avg_dist * sin(Theta_bot + phi/2);

  Theta_bot += phi;

  if(Theta_bot >= 2*pi)
  {
    Theta_bot -= 2*pi;
  }

  if(Theta_bot <= (-1)*2*pi)
  {
    Theta_bot += 2*pi;
  }

}
