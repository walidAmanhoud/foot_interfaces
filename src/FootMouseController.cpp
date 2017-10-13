#include "FootMouseController.h"
// #include <tf/transform_datatypes.h>
#include <signal.h>

FootMouseController* FootMouseController::me = NULL;

FootMouseController::FootMouseController(ros::NodeHandle &n, double frequency): 
	_n(n),
	_loopRate(frequency),
	_dt(1 / frequency)
{
	me = this;

	ROS_INFO_STREAM("Motion generator node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz");
}


bool FootMouseController::init() 
{
	// Init state variables
	_firstRealPoseReceived = false;
	_firstEventReceived = false;
	_msgFootMouse.event = foot_interfaces::FootMouseMsg::FM_NONE;
	_lastEvent = foot_interfaces::FootMouseMsg::FM_NONE;
	_firstButton = false;
	_buttonPressed = false;

	// Init control variables
	_pcur.setConstant(0.0f);
	_pdes.setConstant(0.0f);
	_vdes.setConstant(0.0f);
	_omegades.setConstant(0.0f);
	_qcur.setConstant(0.0f);
	_qdes.setConstant(0.0f);
	_attractorPosition.setConstant(0.0f);

	if (!initROS()) 
	{
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}


bool FootMouseController::initROS() 
{
	// Subscriber definitions
	_subRealPose = _n.subscribe("/lwr/ee_pose", 1, &FootMouseController::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
	_subFootMouse= _n.subscribe("/foot_mouse", 100, &FootMouseController::updateFootMouseData, this, ros::TransportHints().reliable().tcpNoDelay());

	// Publisher definitions
	_pubDesiredPose = _n.advertise<geometry_msgs::Pose>("fm", 1);	
	_pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
	_pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);

	_pubAttractorPosition = _n.advertise<geometry_msgs::PointStamped>("ds/attractorPosition", 1);
	_pubDesiredPath = _n.advertise<nav_msgs::Path>("ds/desiredPath", 1);

		
	// Dynamic reconfigure definition
	_dynRecCallback = boost::bind(&FootMouseController::dynamicReconfigureCallback, this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);


	// nav_msgs::Path msgDesiredPath;
	_msgDesiredPath.poses.resize(MAX_FRAME);

	_start = true;
	_rightClick = false;
	_count = 0;

  if(pthread_create(&_thread, NULL, &FootMouseController::startPathPublishingLoop, this))
  {
      throw std::runtime_error("Cannot create reception thread");   
  }

	signal(SIGINT,FootMouseController::stopNode);
	
	if (_n.ok()) 
	{ 
		// Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The foot mouse controller is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}


}

void FootMouseController::run()
{
	while (!_stop) 
	{
		if(_firstEventReceived && _firstRealPoseReceived)
		{
			// Compute control command
			computeCommand();

			// Publish data to topics
			publishData();
		}

		ros::spinOnce();

		_loopRate.sleep();
	}

	_vdes.setConstant(0.0f);
	_omegades.setConstant(0.0f);
	_qdes = _qcur;

	publishData();
	ros::spinOnce();
	_loopRate.sleep();

	_start = false;
  pthread_join(_thread,NULL);

  ros::shutdown();
}

void FootMouseController::stopNode(int sig)
{
	me->_stop = true;
}

void  FootMouseController::computeCommand()
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
			processABButtonEvent(buttonState,newEvent,-1.0f);
			break;
		}
		case foot_interfaces::FootMouseMsg::FM_BTN_B:
		{
			processABButtonEvent(buttonState,newEvent,1.0f);
			break;
		}
		case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
		{
			processRightClickEvent(buttonState,newEvent);
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
}

void FootMouseController::publishData()
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


	_mutex.unlock();
}


void FootMouseController::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
	_msgRealPose = *msg;

	// Update end effecotr pose (position+orientation)
	_pcur << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
	_qcur << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
	_Rcur =  KDL::Rotation::Quaternion(_msgRealPose.orientation.x,_msgRealPose.orientation.y,_msgRealPose.orientation.z,_msgRealPose.orientation.w);

  _wRb << _Rcur.UnitX().x(), _Rcur.UnitY().x(), _Rcur.UnitZ().x(),
       _Rcur.UnitX().y(), _Rcur.UnitY().y(), _Rcur.UnitZ().y(), 
       _Rcur.UnitX().z(), _Rcur.UnitY().z(), _Rcur.UnitZ().z();

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		_pdes = _pcur;
		_qdes = _qcur;
		_vdes.setConstant(0.0f);
	}
}


void FootMouseController::updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg)
{
	_msgFootMouse = *msg;

	if(!_firstEventReceived && _msgFootMouse.event > 0)
	{
		_firstEventReceived = true;
	}
}


void FootMouseController::processABButtonEvent(int value, bool newEvent, int direction)
{
	if(_modeThreeTranslation) // Control translation along z axis of world frame
	{
		if(!_firstButton)
		{
			_firstButton = true;
		}

		if(!newEvent) // No new event received
		{
			// Track desired position
			_vdes = -_convergenceRate*(_pcur-_pdes);
		}
		else
		{
			if(value>0) // Button pressed
			{
				// Update desired z velocity and position
				_count++;
				if(_count>MAX_XY_REL)
				{
					_count = MAX_XY_REL;
				}
				_buttonPressed = true;
				_vdes(2) = direction*_zVelocity*_count/MAX_XY_REL;
				_pdes(2) = _pcur(2);
			}
			else // Button released
			{
				// Track desired position
				_count = 0;
				_buttonPressed = false;
				_vdes(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
			}
		}
	}
	else // Control translation along z axis of end effector frame
	{
		if(!_firstButton)
		{
			_firstButton = true;
		}
		
		if(!newEvent) // No new event received
		{
				// Track desired position
			_vdes = -_convergenceRate*(_pcur-_pdes);
		}
		else
		{
			// Compute desired orientation matrix from desired quaternion
			_Rdes = KDL::Rotation::Quaternion(_qdes(1),_qdes(2),_qdes(3),_qdes(0));
			
			// Extract the z vector from the orientation matrix
			KDL::Vector temp = _Rdes.UnitZ();
			Eigen::Vector3f zAxis;
			zAxis << temp.x(),temp.y(),temp.z();

			if(value>0)
			{
				_count++;
				if(_count>MAX_XY_REL)
				{
					_count = MAX_XY_REL;
				}
				// Update desired velocity and position
				_buttonPressed = true;
				_vdes = direction*_zVelocity*zAxis;
				_pdes = _pcur;
			}
			else
			{
				_count = 0;
				// Track desired position
				_buttonPressed = false;
				_vdes= -_convergenceRate*(_pcur-_pdes);
			}	
		}
	}
}


void FootMouseController::processCursorEvent(float relX, float relY, bool newEvent)
{
	if(_modeThreeTranslation) // Control translations along x,x axis of world frame
	{
		if(!newEvent) // No new event received
		{
			// Track desired position
			_vdes = -_convergenceRate*(_pcur-_pdes);
		}
		else
		{
			// Update desired x,y position
			_pdes(0) = _pcur(0);
			_pdes(1) = _pcur(1);

			// Compute desired x, y velocities along x, y axis of world frame
			if(relX>MAX_XY_REL)
			{
				_vdes(1) = -_linearVelocityLimit;
			}
			else if(relX < -MAX_XY_REL)
			{
				_vdes(1) = _linearVelocityLimit;
			}
			else
			{
				_vdes(1) = -_linearVelocityLimit*relX/MAX_XY_REL;
			}

			if(relY>MAX_XY_REL)
			{
				_vdes(0) = -_linearVelocityLimit;
			}
			else if(relY < -MAX_XY_REL)
			{
				_vdes(0) = _linearVelocityLimit;
			}
			else
			{
				_vdes(0) = -_linearVelocityLimit*relY/MAX_XY_REL;
			}

			if(!_firstButton || !_buttonPressed)
			{
				// If buttons not pressed track desired z position
				_vdes(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
			}
		}	
	}
	else // Control rotation along x, y axis of end effector frame
	{
		if(!newEvent) // No new event received
		{
			// Track desired position
			_vdes = -_convergenceRate*(_pcur-_pdes);
			// Track desired orientation
			_qdes = _qcur;
		}
		else
		{
			// Compute desired angular velocities along x, y axis of end
			// effector frame
			if(relX>MAX_XY_REL)
			{
				_omegades(0) = _angularVelocityLimit;
			}
			else if(relX < -MAX_XY_REL)
			{
				_omegades(0) = -_angularVelocityLimit;
			}
			else
			{
				_omegades(0) = _angularVelocityLimit*relX/MAX_XY_REL;
			}

			if(relY>MAX_XY_REL)
			{
				_omegades(1) = _angularVelocityLimit;
			}
			else if(relY < -MAX_XY_REL)
			{
				_omegades(1) = -_angularVelocityLimit;
			}
			else
			{
				_omegades(1) = _angularVelocityLimit*relY/MAX_XY_REL;
			}

			// Set desired angular velocity along z axos to 0
			_omegades(2) = 0.0f;

			// Update desired quaternion based on desired end effector frame
			// angular velocity
			Eigen::Vector4f q;
			q = _qdes;
			Eigen:: Vector4f wq;
			wq << 0, _omegades;
			Eigen::Vector4f dq = quaternionProduct(q,wq);
			q += 0.5f*_dt*dq;
			q /= q.norm();
			_qdes = q;

			if(!_firstButton || !_buttonPressed)
			{
				// If buttons not pressed track desired position
				_vdes = -_convergenceRate*(_pcur-_pdes);
			}
		}		
	}
}

void FootMouseController::processRightClickEvent(int value, bool newEvent)
{

	if(!_firstButton)
	{
		_firstButton = true;
	}

	if(!newEvent) // No new event received
	{
		// Track desired position
		_vdes = -_convergenceRate*(_pcur-_pdes);
		_rightClick = false;
	}
	else
	{
		float alpha = 4.0f;
		float omega = M_PI;
		float r = 0.05f;

		if(value == 1) // Button pressed
		{
			// Update desired z velocity and position
			_buttonPressed = true;
			_attractorPosition = _pcur;
			_pdes = _pcur;
			_vdes = -_convergenceRate*(_pcur-_attractorPosition);
			_rightClick = true;
		}
		else if(value == 2)
		{
			Eigen::Vector3f x = _pcur-_attractorPosition;
			float R = sqrt(x(0) * x(0) + x(1) * x(1));
			float T = atan2(x(1), x(0));
			float vx = -alpha*(R-r) * cos(T) - R * omega * sin(T);
			float vy = -alpha*(R-r) * sin(T) + R * omega * cos(T);
			float vz = -alpha*x(2);

			_vdes << vx, vy, vz;

			if (_vdes.norm() > 0.15f) 
			{
				_vdes = _vdes / _vdes.norm()*0.15f;
			}

			_rightClick = true;
		}
		else // Button released
		{
			// Track desired position
			_buttonPressed = false;
			_pdes = _pcur;
			_vdes = -_convergenceRate*(_pcur-_pdes);
			_rightClick = false;
		}
	}
}


Eigen::Vector4f FootMouseController::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  	Eigen::Vector4f q;
  	q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  	Eigen::Vector3f q1Im = (q1.segment(1,3));
  	Eigen::Vector3f q2Im = (q2.segment(1,3));
  	q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  	return q;
}

void FootMouseController::callback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
{
	this->dynamicReconfigureCallback(config,level);
}

void FootMouseController::dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure request. Updatig the parameters ...");

	_convergenceRate = config.convergenceRate;
	_zVelocity = config.zVelocity;
	_linearVelocityLimit = config.linearVelocityLimit;
	_angularVelocityLimit = config.angularVelocityLimit;
	_modeThreeTranslation = config.modeThreeTranslation;

	if (_convergenceRate < 0)
	{
		ROS_ERROR("RECONFIGURE: The convergence rate cannot be negative!");
	}

	if (_zVelocity < 0)
	{
		ROS_ERROR("RECONFIGURE: The z velocity cannot be negative!");
	}

	if (_linearVelocityLimit < 0) 
	{
		ROS_ERROR("RECONFIGURE: The limit for linear velocity cannot be negative!");
	}

	if (_angularVelocityLimit < 0) 
	{
		ROS_ERROR("RECONFIGURE: The limit for angular velocity cannot be negative!");
	}
}


void* FootMouseController::startPathPublishingLoop(void* ptr)
{
    reinterpret_cast<FootMouseController *>(ptr)->pathPublishingLoop();  
}


void FootMouseController::pathPublishingLoop()
{
	while(_start)
	{
		if(_rightClick)
		{
			publishFuturePath();	
		}
	}
	std::cerr << "END path publishing thread" << std::endl;
}

void FootMouseController::publishFuturePath() 
{
	geometry_msgs::PointStamped msg;

	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = _attractorPosition(0);
	msg.point.y = _attractorPosition(1);
	msg.point.z = _attractorPosition(2);

	_pubAttractorPosition.publish(msg);



	_msgDesiredPath.header.stamp = ros::Time::now();
	_msgDesiredPath.header.frame_id = "world";

	Eigen::Vector3f simulatedPose = _pcur;
	Eigen::Vector3f vsim;

	vsim.setConstant(0.0f);

	float alpha = 4.0f;
	float omega = M_PI;
	float r = 0.05f;

	for (int frame = 0; frame < MAX_FRAME; frame++)
	{

		Eigen::Vector3f x = simulatedPose-_attractorPosition;
		float R = sqrt(x(0) * x(0) + x(1) * x(1));
		float T = atan2(x(1), x(0));
		float vx = -alpha*(R-r) * cos(T) - R * omega * sin(T);
		float vy = -alpha*(R-r) * sin(T) + R * omega * cos(T);
		float vz = -alpha*x(2);

		vsim << vx, vy, vz;

		if (vsim.norm() > 0.15f) 
		{
			vsim = vsim / vsim.norm()*0.15f;
		}

		simulatedPose +=  vsim* _dt * 20;

		_msgDesiredPath.poses[frame].header.stamp = ros::Time::now();
		_msgDesiredPath.poses[frame].header.frame_id = "world";
		_msgDesiredPath.poses[frame].pose.position.x = simulatedPose(0);
		_msgDesiredPath.poses[frame].pose.position.y = simulatedPose(1);
		_msgDesiredPath.poses[frame].pose.position.z = simulatedPose(2);

		_pubDesiredPath.publish(_msgDesiredPath);

	}
}