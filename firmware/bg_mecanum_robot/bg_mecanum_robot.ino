

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <AFMotor.h>

ros::NodeHandle  nh;

uint8_t _speed_fl = 0;   
uint8_t _speed_fr = 0;
uint8_t _speed_rl = 0;   
uint8_t _speed_rr = 0; 

uint8_t _forward_fl = FORWARD;  
uint8_t _forward_fr = FORWARD; 
uint8_t _forward_rl = FORWARD;  
uint8_t _forward_rr = FORWARD; 


uint8_t _id_fl = 0; 
uint8_t _id_fr = 1; 
uint8_t _id_rl = 2; 
uint8_t _id_rr = 3; 


AF_DCMotor _motor_fl(3); 
AF_DCMotor _motor_fr(2);
AF_DCMotor _motor_rl(4); 
AF_DCMotor _motor_rr(1);


void messageCb( const std_msgs::Int16MultiArray& drive_msg)
{
  
  
  // save directions
  if(drive_msg.data[_id_fl] < 0)
    _forward_fl = FORWARD;
  else
    _forward_fl = BACKWARD;
    
  // save directions
  if(drive_msg.data[_id_fr] < 0)
    _forward_fr = FORWARD;
  else
    _forward_fr = BACKWARD;

  // save directions
  if(drive_msg.data[_id_rl] > 0)
    _forward_rl = FORWARD;
  else
    _forward_rl = BACKWARD;

  // save directions
  if(drive_msg.data[_id_rr] > 0)
    _forward_rr = FORWARD;
  else
    _forward_rr = BACKWARD;

    

  // save speed
  _speed_fl = abs(drive_msg.data[_id_fl]);   
  _speed_fr = abs(drive_msg.data[_id_fr]);
  _speed_rl = abs(drive_msg.data[_id_rl]);   
  _speed_rr = abs(drive_msg.data[_id_rr]); 

  
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("cmd_drives", messageCb );





void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();

  // set direction of drive
  _motor_fl.run(_forward_fl);
  _motor_fr.run(_forward_fr); 
  _motor_rl.run(_forward_rl);
  _motor_rr.run(_forward_rr);



  // set speed 
  _motor_fl.setSpeed(_speed_fl);  
  _motor_fr.setSpeed(_speed_fr);
  _motor_rl.setSpeed(_speed_rl);  
  _motor_rr.setSpeed(_speed_rr);
  
  delay(100);
}
