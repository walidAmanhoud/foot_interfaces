#include "FootMouseInterface.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "footMouseInterface");

  ros::NodeHandle n;
  float frequency = 500.0f;

  FootMouseInterface footMouseInterface(n,frequency);
 
  if (!footMouseInterface.init()) 
  {
    return -1;
  }
  else
  {
    footMouseInterface.run();
  }

  return 0;

}