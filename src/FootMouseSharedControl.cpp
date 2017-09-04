#include "FootMouseSharedControl.h"
// #include <tf/transform_datatypes.h>


FootMouseSharedControl::FootMouseSharedControl(ros::NodeHandle &n, double frequency): FootMouseController(n,frequency),
	_n(n),
	_loopRate(frequency),
	_dt(1 / frequency)
{
	_vuser.setConstant(0.0f);
	
  for(int k = 0; k < NB_TASKS; k++)
  {
    _vtask[k].setConstant(0.0f);
  }

  _taskId = 0;

  _taskAttractor[0] << -0.34f, 0.3f, 0.35f;
  //_taskAttractor[1] << -0.34f, -0.3f, 0.35f;

  _pubtaskAttractor[0] = _n.advertise<geometry_msgs::PointStamped>("footMouseSharedControl/taskAttractor1", 1);
  //_pubtaskAttractor[1] = _n.advertise<geometry_msgs::PointStamped>("footMouseSharedControl/taskAttractor2", 1);
  _pubArbitration = _n.advertise<std_msgs::Float32>("footMouseSharedControl/arbitration", 1);

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
  std::cerr << "vuser: " << _vuser.transpose() << std::endl;

  std::cerr << "vtask " <<  _taskId << ": "<<  _vtask[_taskId].transpose() << std::endl;

  _vdes = (1.0f-_alpha)*_vuser+_alpha*_vtask[_taskId];
}

void FootMouseSharedControl::computeAutonomy()
{

  
  Eigen::VectorXf d, c, ct, ch;
  d.resize(NB_TASKS);
  c.resize(NB_TASKS);
  ct.resize(NB_TASKS);
  ch.resize(NB_TASKS);

  for(int k =0; k < NB_TASKS; k++)
  {

    d(k) = (_taskAttractor[k]-_pcur).norm();
    Eigen::Vector3f vu, vt;

    if(_vuser.norm()> 1e-6f)
    {
      vu = _vuser/_vuser.norm();
    }
    else
    {
      vu.setConstant(0.0f);
    }

    if(_vtask[k].norm()> 1e-6f)
    {
      vt = _vtask[k]/_vtask[k].norm();
    }
    else
    {
      vt.setConstant(0.0f);
    }


    ch(k) = _agreementWeight*_vtask[k].dot(_vuser);
    ct(k) = _attractorWeight*std::exp(-(d(k)-_d1)/_d2);
    
    c(k) = ch(k)+ct(k);
    std::cerr << "c" << k << ": " << c(k) << " ch: " << ch(k) <<" ct: " << ct(k) << std::endl;
    // _alpha = 
  }

  // _alpha = std::exp(-(d-_d1)/_d2-2.0f*_vuser.norm());

  // _alpha = (_vuser/_vuser.norm()).dot(_vtask/_vtask.norm())+std::exp(-(d-_d1)/_d2);

  ct.maxCoeff(&_taskId);
  _alpha = c(_taskId);

  if(_alpha>_arbitrationLimit)
  {
    _alpha = _arbitrationLimit;
  }
  else if(_alpha < 0.0f)
  {
    _alpha = 0.0f;
  }

}

void FootMouseSharedControl::computeTaskVelocity()
{

    float alpha = 4.0f;
    float omega = M_PI;
    float r = 0.05f;

    for(int k =0; k < NB_TASKS; k++)
    {
      Eigen::Vector3f x = _pcur-_taskAttractor[k];
      float R = sqrt(x(0) * x(0) + x(1) * x(1));
      float T = atan2(x(1), x(0));
      // float vx = -alpha*(R-r) * cos(T) - R * omega * sin(T);
      // float vy = -alpha*(R-r) * sin(T) + R * omega * cos(T);
      // float vz = -alpha*x(2);
      float vx = -alpha*x(0);
      float vy = -alpha*x(1);
      float vz = -alpha*x(2);
      _vtask[k] << vx, vy, vz;

      if (_vtask[k].norm() > 0.15f) 
      {
        _vtask[k] = _vtask[k] / _vtask[k].norm()*0.15f;
      }
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

		if(relX >-5 && relX <5)
		{
			relX = 0;
		}

		if(relY >-5 && relY <5)
		{
			relY = 0;
		}
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

void FootMouseSharedControl::publishData()
{
    _mutex.lock();

  // Publish desired pose
  _msgDesiredPose.position.x = _pdes(0);
  _msgDesiredPose.position.y = _pdes(1);
  _msgDesiredPose.position.z = _pdes(2);
  _msgDesiredPose.orientation.w = _qdes(0);
  _msgDesiredPose.orientation.x = _qdes(1);
  _msgDesiredPose.orientation.y = _qdes(2);
  _msgDesiredPose.orientation.z = _qdes(3);

  _pubDesiredPose.publish(_msgDesiredPose);

  // Publish desired twist (passive ds controller)
  _msgDesiredTwist.linear.x  = _vdes(0);
  _msgDesiredTwist.linear.y  = _vdes(1);
  _msgDesiredTwist.linear.z  = _vdes(2);

  // Convert desired end effector frame angular velocity to world frame
  Eigen::Matrix3f tempM; 
  Eigen::Vector3f tempV;
  tempM << _Rcur.data[0], _Rcur.data[1], _Rcur.data[2],
           _Rcur.data[3], _Rcur.data[4], _Rcur.data[5],
           _Rcur.data[6], _Rcur.data[7], _Rcur.data[8];

  tempV = tempM*_omegades;
  _msgDesiredTwist.angular.x = tempV(0);
  _msgDesiredTwist.angular.y = tempV(1);
  _msgDesiredTwist.angular.z = tempV(2);

  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qdes(0);
  _msgDesiredOrientation.x = _qdes(1);
  _msgDesiredOrientation.y = _qdes(2);
  _msgDesiredOrientation.z = _qdes(3);

  _pubDesiredOrientation.publish(_msgDesiredOrientation);


  for(int k =0; k < NB_TASKS; k++)
  {
  _msgTaskAttractor[k].header.frame_id = "world";
  _msgTaskAttractor[k].header.stamp = ros::Time::now();
  _msgTaskAttractor[k].point.x = _taskAttractor[k](0);
  _msgTaskAttractor[k].point.y = _taskAttractor[k](1);
  _msgTaskAttractor[k].point.z = _taskAttractor[k](2);
  _pubtaskAttractor[k].publish(_msgTaskAttractor[k]);
    
  }

  std_msgs::Float32 msg;

  msg.data = _alpha;
  _pubArbitration.publish(msg);

  _mutex.unlock();


}

void FootMouseSharedControl::dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure request bou. Updatig the parameters ...");

	_arbitrationLimit = config.arbitrationLimit;
	_agreementWeight = config.agreementWeight;
	_attractorWeight = config.attractorWeight;
	_d1 = config.d1;
	_d2 = config.d2;
	_convergenceRate = config.convergenceRate;
	_zVelocity = config.zVelocity;
	_linearVelocityLimit = config.linearVelocityLimit;
	_angularVelocityLimit = config.angularVelocityLimit;
	_modeThreeTranslation = config.modeThreeTranslation;
	// if (_arbitrationLimit < 0)
	// {
	// 	ROS_ERROR("RECONFIGURE: The convergence rate cannot be negative!");
	// }

	// if (_zVelocity < 0)
	// {
	// 	ROS_ERROR("RECONFIGURE: The z velocity cannot be negative!");
	// }

	// if (_linearVelocityLimit < 0) 
	// {
	// 	ROS_ERROR("RECONFIGURE: The limit for linear velocity cannot be negative!");
	// }

	// if (_angularVelocityLimit < 0) 
	// {
	// 	ROS_ERROR("RECONFIGURE: The limit for angular velocity cannot be negative!");
	// }
}
