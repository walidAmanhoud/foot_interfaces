#include "FootMouseSharedControl.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "footMouseSharedControl");
  ros::NodeHandle n;
  float frequency = 500.0f;

  FootMouseSharedControl footMouseSharedControl(n,frequency);
 
  if (!footMouseSharedControl.init()) 
  {
    return -1;
  }
  else
  {
    bool ready = false;
    if(n.hasParam("ready"))
    {
      while(!ready)
      {

        n.getParam("ready", ready);
      }
    
      ROS_INFO("Start foot mouse shared control");
      footMouseSharedControl.run();

      n.deleteParam("ready");
    }
    else
    {
      footMouseSharedControl.run();
    }  
  }


  return 0;
}

