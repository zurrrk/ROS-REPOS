#include "ros/ros.h"  // rosで必要はヘッダーファイル
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot"); 

  ros::NodeHandle nh;
  // ノードハンドラの作成。ハンドラは必要になったら起動される。

  ros::Publisher  pub_joint1, pub_joint2;
  // パブリッシャの作成。トピックに対してデータを送信。

  ros::Rate rate(0.25);
  

  std_msgs::Float64 target_joint1, target_joint2;


  pub_joint1 = nh.advertise<std_msgs::Float64>("/my_robot/arm1_joint_effort_controller/command", 10);
  pub_joint2 = nh.advertise<std_msgs::Float64>("/my_robot/arm2_joint_effort_controller/command", 10);
 
  float knee = 7;
  float hip = -2;
  
  target_joint1.data = -knee/3; //knee
  target_joint2.data = -hip;  //hip

  while (ros::ok()) { // このノードが使える間は無限ループ
    if(target_joint1.data==knee){
      target_joint1.data=-knee/3;
    }
    else if(target_joint1.data==-knee/3){
      target_joint1.data=knee;
    }

    if(target_joint2.data==hip){
      target_joint2.data=-hip;
    }
    else if(target_joint2.data==-hip){
      target_joint2.data=hip;
    }
      
    pub_joint1.publish(target_joint1);    
    pub_joint2.publish(target_joint2);

    
    ros::spinOnce(); // コールバック関数を呼ぶ
    rate.sleep();     // 指定した周期でループするよう寝て待つ
  }
  
  return 0;
}
