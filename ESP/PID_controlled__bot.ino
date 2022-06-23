#define TCP true
#include "utils.h"
#include <PID_v1.h>
//#if TCP
//#include "ArduinoTcpHardware.h"
//#else
//#include "ArduinoHardware.h"
//#endif    

#include "ArduinoTcpHardware.h"

#include <geometry_msgs/Twist.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#if TCP

#include <WiFi.h>
const char *ssid = "Galaxy M31A78E";
const char *password = "rutuhrishi4565";
// Set the rosserial socket server IP address
IPAddress server(192,168,238,26);
//Set the rosserial socket server port
const uint16_t serverPort = 11411;

#endif

void velCallback(const geometry_msgs::Twist &vel)
{
  x = vel.linear.x;
  z = vel.angular.z;
}

//initializing handle

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);
tf::TransformBroadcaster broadcaster;

//tf variables to be
char base_link[] = "/base_link";
char odom[] = "/odom";

double SetRPS_R =5;
double SetRPS_L =5;
double Kp = 50; double Ki = 300 ; double Kd = 0;
// The values of Kp, Ki, Kd for rpm are 1,1,0 (for 100 ms timer)
// 1.2, 9.5, 0 for rps 100ms timer

double input_L = 0;
double input_R = 0;
double output_L = 0;
double output_R = 0;

PID PID_R(&rps_R, &output_R, &SetRPS_R, Kp, Ki, Kd, DIRECT);
PID PID_L(&rps_L, &output_L, &SetRPS_L, Kp, Ki, Kd, DIRECT);


void setup() {
  PID_R.SetMode(AUTOMATIC);
  PID_L.SetMode(AUTOMATIC);
  PID_R.SetOutputLimits(-255, 255);
  PID_L.SetOutputLimits(-255, 255);
  
  SetPins();

  SetTimer();

  #if TCP

    Serial.begin(115200);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    //Connect the ESP to the wifi AP
    WiFi.begin(ssid, password);

    while(WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Set the connection to rosserial socket server
    nh.getHardware() -> setConnection(server, serverPort);

    //Another way to get IP
    Serial.print("IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

  #endif

  #if !TCP
    nh.getHardware()->setBaud(115200);
  #endif

  
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(sub);
  broadcaster.init(nh);




  // Serial.begin(9600);
}

double currentMillis=0, previousMillis=0;

void loop() {
    nh.spinOnce();

    SetRPS_L = (x - z*wheel_dist/2)/(pi * DIAMETER);
    SetRPS_R = (x + z*wheel_dist/2)/(pi * DIAMETER);

    PID_R.Compute();
    PID_L.Compute();

    driveMotor(LEFT, output_L);
    driveMotor(RIGHT, output_R);
//
//      Serial.println(distance_travelled);   


// *** broadcast odom->base_link transform with tf ***
   currentMillis = millis();
    if (currentMillis - previousMillis >= 10)
    {
    geometry_msgs::TransformStamped t;

    t.header.stamp = nh.now();
    t.header.frame_id = odom;
    t.child_frame_id = base_link;

    t.transform.translation.x = x_bot;
    t.transform.translation.y = y_bot;
    t.transform.translation.z = 0;

    t.transform.rotation = tf::createQuaternionFromYaw(Theta_bot);

    broadcaster.sendTransform(t);

 // *** broadcast odom message ***

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_bot;
    odom_msg.pose.pose.position.y = y_bot;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(Theta_bot);

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = Velocity_bot;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = Omega_bot;

    odom_pub.publish(&odom_msg);


    }

}
