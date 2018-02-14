#include <ros/ros.h>
#include <move_along_roads/RoadPlannerAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_along_roads::RoadPlannerAction> MoveClient;

// this program waits for an input, and then sends a goal to road_planner

// """ this could be updated or adapted to work with road_planner.py to
//     direct the robot to non-node locations, by doing stuff like pythagoras or something,
//     but for now it works okay like this """

bool move_to_node(std::string goalname) {
  move_along_roads::RoadPlannerGoal goal;
  goal.end_node = goalname;
  ac.sendGoal(goal);
  std::cout << "Moving to " << goalname << "..." << '\n';

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Node reached");
    return true;
  else {
    ROS_INFO("Node NOT reached");
    return false;
  }
}

int main(int argc, char** argv){

  std::cout << "Initialising node..." << '\n';
  ros::init(argc, argv, "welcome_visitors");

  //tell the action client that we want to spin a thread by default
  std::cout << "Initialising action client..." << '\n';
  MoveClient ac("road_planner", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the road_planner action server to come up");
  }

  // initial goal for the end nodes
  move_along_roads::RoadPlannerGoal goal;

  while (ros::ok) {
    std::cout << "Doorbell rung, moving to door..." << '\n';
    move_to_node("door");


    std::cout << "Input visitor type: " << '\n';
  	std::string input;
  	std::cin >> input;

    if (input == "exit")
      break;

    if (input == "deli_worker") {
      std::cout << "follow me" << '\n';
      ROS_INFO("Sending location goal to kitchen table");
      move_to_node("kitchen_table");
      std::cout << "please place the breakfast on the table" << '\n';
      time.sleep(2);
      std::cout << "thank you, please follow me to the door" << '\n';
      move_to_node("door");
      std::cout << "bye!" << '\n';
    }
    else if (input == "postal worker") {
      std::cout << "follow me" << '\n';
      ROS_INFO("Sending location goal to kitchen table");
      move_to_node("kitchen_table");
      std::cout << "please place the package on the table" << '\n';
      time.sleep(2);
      std::cout << "thank you, please follow me to the door" << '\n';
      move_to_node("door");
      std::cout << "bye!" << '\n';
    }
    else if (input == "doctor") {
      std::cout << "follow me" << '\n';
      ROS_INFO("Sending location goal to granny's bed");
      move_to_node("bed");
      std::cout << "I shall wait for you" << '\n';
      move_to_node("living_room");
      time.sleep(2);
      std::cout << "thank you, please follow me to the door" << '\n';
      move_to_node("door");
      std::cout << "bye!" << '\n';
    }
    else {
      std::cout << "sorry, I don't know you" << '\n';
    }
    return 0;
  }
  return 0;
}
