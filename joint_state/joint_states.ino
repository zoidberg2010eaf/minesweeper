
#include <ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

const int I = 2;
int l_ticks, r_ticks;
const int maxTicks = 60;
double l_pos, r_pos;
bool lock = false;
int l_sign = 1, r_sign = 1;

void SignCb(const std_msgs::Int8MultiArray &data){
  l_sign = data.data[0];
  r_sign = data.data[1];
}

ros::Subscriber<std_msgs::Int8MultiArray> sub("sign", &SignCb);
std_msgs::Float32 l_jointPoistion;
std_msgs::Float32 r_jointPoistion;
ros::Publisher fl_pub("front_left_wheel", &l_jointPoistion);
ros::Publisher fr_pub("front_right_wheel", &r_jointPoistion);
ros::Publisher rl_pub("rear_left_wheel", &l_jointPoistion);
ros::Publisher rr_pub("rear_right_wheel", &r_jointPoistion);


void setup() {
  pinMode(I, INPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(fl_pub);
  nh.advertise(fr_pub);
  nh.advertise(rl_pub);
  nh.advertise(rr_pub);
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
  l_pos = ((double)l_ticks/maxTicks) * 2 * PI;
  r_pos = ((double)r_ticks/maxTicks) * 2 * PI;
  l_jointPoistion.data = l_pos;
  r_jointPoistion.data = r_pos;
  fl_pub.publish(&l_jointPoistion);
  fr_pub.publish(&r_jointPoistion);
  rl_pub.publish(&l_jointPoistion);
  rr_pub.publish(&r_jointPoistion);
  nh.spinOnce();
  delay(20);
}
