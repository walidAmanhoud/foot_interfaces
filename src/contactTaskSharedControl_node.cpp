#include "ContactTaskSharedControl.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "contactTaskSharedControl");
  ros::NodeHandle n;
  float frequency = 500.0f;

  ContactTaskSharedControl contactTaskSharedControl(n,frequency);
 
  if (!contactTaskSharedControl.init()) 
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
      contactTaskSharedControl.run();

      n.deleteParam("ready");
    }
    else
    {
      contactTaskSharedControl.run();
    }  
  }


  return 0;
}

