#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <dvrk_control/DVRKControl.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<dvrk_control::DVRKControl> node(argc, argv, "dvrk_control", 1);
  return static_cast<int>(!node.execute());
}


