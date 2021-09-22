#include "movebase_client.h"

using namespace BT;

NodeStatus MoveBase::tick() {
   // if no server is present, fail after 2 seconds
   // 如果伺服器沒有回應，在兩秒後回傳失敗
   if(!_client.waitForServer(ros::Duration(2.0))) {
      ROS_ERROR("Can't contact move_base server");
      return NodeStatus::FAILURE;
   }

   // Take the goal from the InputPort of the Node
   // 從節點中的輸入端口拿取目標數值
   Position2D goal;
   if(!getInput<Position2D>("goal",goal)) {
      // if I can't get this, there is something wrong with your BT.
      // For this reason throw an exception instead of returning FAILURE
      // 若無法拿到goal，代表行為樹有問題
      // 因此在此拋出異常訊息而不是返回FAILURE
      throw RuntimeError("missing required input [goal]");
   }

   // Reset this flag
   // 將中止旗標重製
   _aborted = false;

   ROS_INFO("Sending goal %f %f %f %f", goal.x, goal.y, goal.quaternion_z, goal.quaternion_w);

   // Bulid the message from Position2D
   // 建立來自Position2D的訊息
   move_base_msgs::MoveBaseGoal msg;
   msg.target_pose.header.frame_id = "map";
   msg.target_pose.header.stamp = ros::Time::now();
   msg.target_pose.pose.position.x = goal.x;
   msg.target_pose.pose.position.y = goal.y;
   msg.target_pose.pose.orientation.z = goal.quaternion_z;
   msg.target_pose.pose.orientation.w = goal.quaternion_w;

   _client.sendGoal(msg);

   while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
      // polling at 50 Hz. No big deal in terms of CPU
      // 以50hz的頻率輪詢，對cpu來說不是太大的負擔
   }

   if(_aborted) {
      // this happens only if method halt() was invoked
      // _client.cancelAllGoals();
      // 只有在halt()被調用的時候才會觸發
      // cancelAllGoals();會一鍵暫停所有任務
      ROS_ERROR("MoveBase aborted");
      return BT::NodeStatus::FAILURE;
   }

   if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR("MoveBase failed");
      return NodeStatus::FAILURE;
   }

   ROS_INFO("Target reached");
   return NodeStatus::SUCCESS;
}