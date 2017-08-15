#include "FootMouseController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "footMouseController");
  ros::NodeHandle n;
  float frequency = 500.0f;

  FootMouseController footMouseController(n,frequency);
 
  if (!footMouseController.init()) 
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
    
      ROS_INFO("Start foot mouse controller");
      footMouseController.run();

      n.deleteParam("ready");
    }
    else
    {
      footMouseController.run();
    }  
  }


  return 0;
}

