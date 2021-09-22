#pragma once

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>


class WaitingEvent : public AsyncActionNode{
public:
   WaitingEvent(const std::string& name, const BT::NodeConfiguration& config) : BT:: AsyncActionNode(name, config)
   {}

   static BT::PortsList providedPorts()
   {
      return{BT::InputPort<int>("waiting_time")};
   }

   virtual BT::NodeStatus tick() override{
      int expect_waiting_time;
      if(!getInput<int>("waiting_time",expect_waiting_time)){
         throw BT::RuntimeError("missing required input [waiting_time]");
      }
      ros::Duration(expect_waiting_time).sleep();
      return BT::NodeStatus::SUCCESS;
   }
};