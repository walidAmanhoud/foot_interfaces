#include "FootMouseInterface.h"
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

FootMouseInterface::FootMouseInterface(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency),
  _dt(1/frequency)
{

	ROS_INFO_STREAM("The foot mouse interface node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool FootMouseInterface::init(std::string eventPath) 
{

  // Pulibsher definition
  _pubFootMouseData = _n.advertise<foot_interfaces::FootMouseMsg>("foot_mouse", 1);

  if((_fd = open(eventPath.c_str(), O_RDONLY)) == -1) 
  {
    return false;
  }

  // Set flags for non blocking read
  int flags = fcntl(_fd, F_GETFL, 0);
  fcntl(_fd, F_SETFL, flags | O_NONBLOCK);

  // Initialize state variables
  _synReceived = false;
  _relReceived = false;

  // Initialize foot mouse message
  _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_NONE;
  _footMouseMessage.buttonState = 0;
  _footMouseMessage.relX = 0;
  _footMouseMessage.relY = 0;
  _footMouseMessage.relZ = 0;
  _footMouseMessage.filteredRelX = 0.0f;
  _footMouseMessage.filteredRelY = 0.0f;
  _footMouseMessage.filteredRelZ = 0.0f;

  // Initialize filtered x,y relative motion 
  _filteredRelX = 0.0f;
  _filteredRelY = 0.0f;
  _filteredRelZ = 0.0f;

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&FootMouseInterface::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);


	if (_n.ok())
	{ 
  	// Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void FootMouseInterface::run() 
{
  while (_n.ok()) 
  {

    // Read foot mouse data;
    // Publish data to topics
    readFootMouse();

    publishData();

    ros::spinOnce();
    _loopRate.sleep();

  }
}


void FootMouseInterface::publishData()
{
  if(_synReceived)
  {
    _synReceived = false;
    _relReceived = false;

    if(_footMouseMessage.event == foot_interfaces::FootMouseMsg::FM_CURSOR)
    {
      // Update windows for moving average filtering
      if(_winX.size()>_windowSize)
      {
        _winX.pop_front();
      }
      else if(_winX.size() < _windowSize)
      {
        _winX.push_back(_footMouseMessage.relX);
      }
      else
      {
        _winX.pop_front();
        _winX.push_back(_footMouseMessage.relX);

      }
      
      if(_winY.size()>_windowSize)
      {
        _winY.pop_front();
      }
      else if(_winY.size() < _windowSize)
      {
        _winY.push_back(_footMouseMessage.relY);
      }
      else
      {
        _winY.pop_front();
        _winY.push_back(_footMouseMessage.relY);

      }
    }

    if(_useMovingAverage) // If moving average is used, compute average relative motion from the windows
    {
      std::cerr << _winX.size() << std::endl;
      if(_winX.size() == 0)
      {
        _filteredRelX = _footMouseMessage.relX;
      }
      else
      {
        float temp = 0;
        for(int k = 0; k < _winX.size(); k++)
        {
          temp += (float) _winX[k]; 
        }
        temp/= _winX.size();
        _filteredRelX = temp;
      }

      if(_winY.size() == 0)
      {
        _filteredRelY = _footMouseMessage.relX;
      }
      else
      {
        float temp = 0;
        for(int k = 0; k < _winY.size(); k++)
        {
          temp += (float) _winY[k]; 
        }
        temp/= _winY.size();
        _filteredRelY = temp;
      }

      _filteredRelZ = 0.0f;
    }
    else // Use simple 1D low pass filter
    {
      _filteredRelX = _alpha*_filteredRelX+(1.0f-_alpha)*_footMouseMessage.relX;
      _filteredRelY = _alpha*_filteredRelY+(1.0f-_alpha)*_footMouseMessage.relY;
      _filteredRelZ = _alpha*_filteredRelZ+(1.0f-_alpha)*_footMouseMessage.relZ;
    }

    _footMouseMessage.filteredRelX = _filteredRelX;
    _footMouseMessage.filteredRelY = _filteredRelY;
    _footMouseMessage.filteredRelZ = _filteredRelZ;

    std::cerr << "cursor: " << _footMouseMessage.relX << " " << _footMouseMessage.relY << " " << _footMouseMessage.relZ << std::endl;

    // Publish foot mouse message
    _pubFootMouseData.publish(_footMouseMessage);
  }
}

void FootMouseInterface::readFootMouse() 
{

  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(_fd, &readset);
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = int(5e4);
  int result = select(_fd+1, &readset, NULL, NULL, &tv);

  if(result > 0 && FD_ISSET(_fd,&readset)) // Check if data available
  {

    // Read data from input device
    result = read(_fd,&_ie,sizeof(struct input_event));

    // Get event type
    switch (_ie.type)
    {
      case EV_KEY:
      {
        // Get KEY event code
        if(_ie.code == BTN_RIGHT)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK;
        }
        else if(_ie.code == BTN_LEFT)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_LEFT_CLICK;
        }
        else if(_ie.code == KEY_KPMINUS)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_BTN_B;
        }
        else if(_ie.code == KEY_KPPLUS)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_BTN_A;
        }

        _footMouseMessage.buttonState = _ie.value;
        // std::cerr << (int) _ie.code << " " << (int) _ie.value << std::endl;  

        break;
      }
      case EV_REL:
      {     
        // std::cerr << (int) _ie.code << " " << (int) _ie.value << std::endl;  
        // Get REL event code   
        if(_ie.code == REL_X)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_CURSOR;
          _footMouseMessage.relX = _ie.value;
          _footMouseMessage.relY = 0;

          if(!_relReceived)
          {
            _relReceived = true;
          }

        }
        else if(_ie.code == REL_Y)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_CURSOR;
          _footMouseMessage.relY = _ie.value;

          if(!_relReceived)
          {
            _footMouseMessage.relX = 0;
            _relReceived = true;
          }
        }
        else if(_ie.code == REL_Z)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_CURSOR;
          _footMouseMessage.relZ = _ie.value;
          if(!_relReceived)
          {
            _footMouseMessage.relX = 0;
            _footMouseMessage.relY = 0;
            _relReceived = true;
          }
        }
        else if(_ie.code == REL_WHEEL)
        {
          _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_WHEEL;
          _footMouseMessage.relWheel = _ie.value;
        }

        break;
      }
      case EV_SYN:
      {   
        // A SYN event is received between event, it separates them       
        _synReceived = true;
        break;
      }
      default:
      {
        break;
      }
    }
  }
  else // No events happened
  {
    _synReceived = true;
    _footMouseMessage.event = foot_interfaces::FootMouseMsg::FM_NONE;
  }

  // std::cerr << (int) _ie.type << " " << (int) _ie.code << " " << (int) _ie.value << std::endl;
}


void FootMouseInterface::dynamicReconfigureCallback(foot_interfaces::footMouseInterface_paramsConfig &config, uint32_t level) 
{

  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _alpha = config.alpha;
  _useMovingAverage = config.useMovingAverage;
  _windowSize = config.windowSize;

}