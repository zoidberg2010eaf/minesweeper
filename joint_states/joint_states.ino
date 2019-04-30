
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

const int I = 2;
int l_ticks, r_ticks;
const int maxTicks = 60;
double l_pos, r_pos;
double l_past_pos, r_past_pos;
double l_vel, r_vel;
bool lock = false;
int l_sign = 1, r_sign = 1;

void SignCb(const std_msgs::Int8MultiArray &data){
  l_sign = data.data[0];
  r_sign = data.data[1];
}

ros::Subscriber<std_msgs::Int8MultiArray> sub("sign", &SignCb);
std_msgs::Float32 l_jointPoistion;
std_msgs::Float32 r_jointPoistion;
std_msgs::Float32MultiArray joint_states;
std_msgs::Float32MultiArray joint_vel_states;
//ros::Publisher fl_pub("front_left_wheel", &l_jointPoistion);
//ros::Publisher fr_pub("front_right_wheel", &r_jointPoistion);
//ros::Publisher rl_pub("rear_left_wheel", &l_jointPoistion);
//ros::Publisher rr_pub("rear_right_wheel", &r_jointPoistion);
ros::Publisher js_pub("joint_positions", &joint_states);
ros::Publisher js_vel_pub("joint_velocities", &joint_vel_states);

void setup() {
  pinMode(I, INPUT);
  nh.initNode();
  nh.subscribe(sub);
  joint_states.data_length = 4;
  joint_vel_states.data_length = 4;
  nh.advertise(js_pub);
  //nh.advertise(fl_pub);
  //nh.advertise(fr_pub);
  //nh.advertise(rl_pub);
  //nh.advertise(rr_pub);
  nh.advertise(js_vel_pub);
}
void loop() {
  if(!digitalRead(I)){
    if(!lock){
      l_ticks += l_sign;
      r_ticks += r_sign;
      lock = true;
    }
  }else{
    lock = false;
  }
  double interval = 0.020;
  l_past_pos = l_pos;
  r_past_pos = r_pos;
  l_pos = ((double)l_ticks/maxTicks) * 2 * PI;
  r_pos = ((double)r_ticks/maxTicks) * 2 * PI;
  l_jointPoistion.data = l_pos;
  r_jointPoistion.data = r_pos;
  l_vel = (l_pos - l_past_pos) / interval;
  r_vel = (r_pos - r_past_pos) /interval;
  float positions[4] = {l_pos, r_pos, l_pos, r_pos};
  float velocities[4] = {l_vel, r_vel, l_vel, r_vel};
  joint_states.data = positions;
  joint_vel_states.data = velocities;
  //fl_pub.publish(&l_jointPoistion);
  //fr_pub.publish(&r_jointPoistion);
  //rl_pub.publish(&l_jointPoistion);
  //rr_pub.publish(&r_jointPoistion);
  js_pub.publish(&joint_states);
  js_vel_pub.publish(&joint_vel_states);
  nh.spinOnce();
  delay(20);
}
