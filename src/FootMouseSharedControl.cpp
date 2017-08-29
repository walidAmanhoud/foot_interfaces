#include "FootMouseSharedControl.h"
// #include <tf/transform_datatypes.h>


FootMouseSharedControl::FootMouseSharedControl(ros::NodeHandle &n, double frequency): FootMouseController(n,frequency),
	_n(n),
	_loopRate(frequency),
	_dt(1 / frequency)
{
	_vuser.setConstant(0.0f);
	_vtask.setConstant(0.0f);

  _taskAttractor << -0.34f, 0.3f, 0.44f;
}


void FootMouseSharedControl::processRightClickEvent(int value, bool newEvent)
{

	std::cerr << "bou" << std::endl;
}

void  FootMouseSharedControl::computeCommand()
{
	uint8_t event;
	int buttonState, relX, relY, relWheel;
	float filteredRelX = 0.0f, filteredRelY = 0.0f;
	bool newEvent = false;

	// If new event received update last event
	// Otherwhise keep the last one
	if(_msgFootMouse.event > 0)
	{
		_lastEvent = _msgFootMouse.event;
		buttonState = _msgFootMouse.buttonState;
		relX = _msgFootMouse.relX;
		relY = _msgFootMouse.relY;
		relWheel = _msgFootMouse.relWheel;
		filteredRelX = _msgFootMouse.filteredRelX;
		filteredRelY = _msgFootMouse.filteredRelY;
		newEvent = true;
	}
	else
	{
		buttonState = 0;
		relX = 0;
		relY = 0;
		relWheel = 0;
		filteredRelX = 0;
		filteredRelY = 0;
		newEvent = false;
	}

	event = _lastEvent;

	// Process corresponding event
	switch(event)
	{
		case foot_interfaces::FootMouseMsg::FM_BTN_A:
		{
			// processABButtonEvent(buttonState,newEvent,-1.0f);
			break;

		}
		case foot_interfaces::FootMouseMsg::FM_BTN_B:
		{
			// processABButtonEvent(buttonState,newEvent,1.0f);
			break;
		}
		case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
		{
			// processRightClickEvent(buttonState,newEvent);
			break;
		}
		case foot_interfaces::FootMouseMsg::FM_CURSOR:
		{
			processCursorEvent(filteredRelX,filteredRelY,newEvent);
			break;
		}
		default:
		{
			break;
		}

	}


  computeTaskVelocity();

	computeAutonomy();
  std::cerr << "alpha:" << _alpha << std::endl;
  std::cerr << "vuser:" << _vuser.transpose() << std::endl;
  std::cerr << "vtask:" << _vtask.transpose() << std::endl;
  _vdes = (1.0f-_alpha)*_vuser+_alpha*_vtask;
}

void FootMouseSharedControl::computeAutonomy()
{

  float d = (_taskAttractor-_pcur).norm();
  _alpha = std::exp(-(d-_d1)/_d2-2.0f*_vuser.norm());
  
  if(_alpha>1.0f)
  {
    _alpha = 1.0f;
  }

}

void FootMouseSharedControl::computeTaskVelocity()
{

    float alpha = 4.0f;
    float omega = M_PI;
    float r = 0.05f;

    Eigen::Vector3f x = _pcur-_taskAttractor;
    float R = sqrt(x(0) * x(0) + x(1) * x(1));
    float T = atan2(x(1), x(0));
    float vx = -alpha*(R-r) * cos(T) - R * omega * sin(T);
    float vy = -alpha*(R-r) * sin(T) + R * omega * cos(T);
    float vz = -alpha*x(2);

    _vtask << vx, vy, vz;

    if (_vtask.norm() > 0.15f) 
    {
      _vtask = _vtask / _vtask.norm()*0.15f;
    }

}


void FootMouseSharedControl::processCursorEvent(float relX, float relY, bool newEvent)
{
	if(!newEvent) // No new event received
	{
		// Track desired position
		// _vuser = -_convergenceRate*(_pcur-_pdes);
    _vuser.setConstant(0.0f);
	}
	else
	{
		// Update desired x,y position
		_pdes(1) = _pcur(1);
		_pdes(2) = _pcur(2);

		// Compute desired x, y velocities along x, y axis of world frame
		if(relX>MAX_XY_REL)
		{
			_vuser(1) = -_linearVelocityLimit;
		}
		else if(relX < -MAX_XY_REL)
		{
			_vuser(1) = _linearVelocityLimit;
		}
		else
		{
			_vuser(1) = -_linearVelocityLimit*relX/MAX_XY_REL;
		}

		if(relY>MAX_XY_REL)
		{
			_vuser(2) = -_linearVelocityLimit;
		}
		else if(relY < -MAX_XY_REL)
		{
			_vuser(2) = _linearVelocityLimit;
		}
		else
		{
			_vuser(2) = -_linearVelocityLimit*relY/MAX_XY_REL;
		}

		if(!_firstButton || !_buttonPressed)
		{
			// If buttons not pressed track desired z position
			// _vuser(0) = -_convergenceRate*(_pcur(0)-_pdes(0));
      _vuser(0) = 0.0f;
		}
	}	
}