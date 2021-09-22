#include "movebase_client.h"
#include "interrupt_event.h"
#include "waiting_event.h"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

using namespace BT;

int main(int argc, char **argv) {
   ros::init(argc, argv, "test_bt");

   ros::NodeHandle nh("~");
   std::string xml_filename;
   nh.param<std::string>("file",xml_filename,"~/kubot_behavior_tree/xml/kubot_bt_test.xml");
   ROS_INFO("Loading XML : %s" , xml_filename.c_str());

   BehaviorTreeFactory factory;

   factory.registerNodeType<MoveBase>("MoveBase");
   factory.registerNodeType<InterruptEvent>("InterruptEvent");
   factory.registerNodeType<WaitingEvent>("WaitingEvent");

   auto tree = factory.createTreeFromFile(xml_filename);

   StdCoutLogger logger_cout(tree);

   NodeStatus status = NodeStatus::RUNNING;

   while(ros::ok() && status == NodeStatus::RUNNING) {
      status = tree.rootNode()->executeTick();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }

   return 0;
}