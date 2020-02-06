#include "ros/ros.h"  // rosで必要はヘッダーファイル
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot"); 
  // initでROSを初期化して、my_teleopという名前をノードにつける                        
  // 同じ名前のノードが複数あってはいけないので、ユニークな名前をつける

  ros::NodeHandle nh;
  // ノードハンドラの作成。ハンドラは必要になったら起動される。

  ros::Publisher  pub_joint1, pub_joint2;
  // パブリッシャの作成。トピックに対してデータを送信。

  ros::Rate rate(2.15);
  // ループの頻度を設定。この場合は10Hz、1秒間に10回数、1ループ100ms。

  std_msgs::Float64 target_joint1, target_joint2;


  pub_joint1 = nh.advertise<std_msgs::Float64>("/my_robot/arm1_joint_position_controller/command", 10);
  pub_joint2 = nh.advertise<std_msgs::Float64>("/my_robot/arm2_joint_position_controller/command", 10);
 

  target_joint1.data = 1.4;
  target_joint2.data = -0.7;

  while (ros::ok()) { // このノードが使える間は無限ループ
    if(target_joint1.data==1.4){
      target_joint1.data=0.0;
    }
    else if(target_joint1.data==0.0){
      target_joint1.data=1.4;
    }

    if(target_joint2.data==-0.7){
      target_joint2.data=0.0;
    }
    else if(target_joint2.data==0.0){
      target_joint2.data=-0.7;
    }
      
    pub_joint1.publish(target_joint1); // 角度を送信    
    pub_joint2.publish(target_joint2);

    
    ros::spinOnce(); // コールバック関数を呼ぶ
    rate.sleep();     // 指定した周期でループするよう寝て待つ
  }
  
  return 0;
}
