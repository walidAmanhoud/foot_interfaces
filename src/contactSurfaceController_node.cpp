#include "ContactSurfaceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contactSurfaceController");
  ros::NodeHandle n;
  float frequency = 500.0f;

  ContactSurfaceController contactSurfaceController(n,frequency);
 
  if (!contactSurfaceController.init()) 
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
    
      ROS_INFO("Start contact task shared control");
      contactSurfaceController.run();

      n.deleteParam("ready");
    }
    else
    {
      contactSurfaceController.run();
    }  
  }


  return 0;
}

