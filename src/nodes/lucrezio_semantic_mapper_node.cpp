#include <iostream>
#include "semantic_mapper_node.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "semantic_mapper_node");
  ros::NodeHandle nh;

  SemanticMapperNode mapper(nh);

//  ros::spin();
  ros::Rate rate(1);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
