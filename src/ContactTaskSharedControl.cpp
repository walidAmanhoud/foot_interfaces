#include "ContactTaskSharedControl.h"
// #include <tf/transform_datatypes.h>


ContactTaskSharedControl::ContactTaskSharedControl(ros::NodeHandle &n, double frequency): FootMouseController(n,frequency),
	_n(n),
	_loopRate(frequency),
	_dt(1 / frequency)
{
	_vuser.setConstant(0.0f);
	
    _vtask.setConstant(0.0f);

  _taskId = 0;

  // _taskAttractor << -0.34f, 0.3f, 0.35f;
  _taskAttractor << -0.34f, 0.0f, 0.2f;

  _msgMarker.header.frame_id = "world";
	_msgMarker.header.stamp = ros::Time();
	_msgMarker.ns = "marker_test_triangle_list";
	_msgMarker.id = 0;
	_msgMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	_msgMarker.action = visualization_msgs::Marker::ADD;
	_msgMarker.pose.position.x = _taskAttractor(0);
	_msgMarker.pose.position.y = _taskAttractor(1);
	_msgMarker.pose.position.z = _taskAttractor(2);
	_msgMarker.pose.orientation.x = 0.0;
	_msgMarker.pose.orientation.y = 1.0;
	_msgMarker.pose.orientation.z = 0.0;
  _msgMarker.pose.orientation.w = 0.0;
  // _msgMarker.pose.orientation.x = 0.0;
  // _msgMarker.pose.orientation.y = 0.9239f;
  // _msgMarker.pose.orientation.z = 0.3827f;
  // _msgMarker.pose.orientation.w = 0.0;
  _Rplane= KDL::Rotation::Quaternion( _msgMarker.pose.orientation.x, _msgMarker.pose.orientation.y, _msgMarker.pose.orientation.z, _msgMarker.pose.orientation.w);
	_msgMarker.scale.x = 1.0;
	_msgMarker.scale.y = 1.0;
	_msgMarker.scale.z = 1.0;
	_msgMarker.color.a = 1.0;

 	geometry_msgs::Point p1,p2,p3,p4,p5,p6;
 	float objectWidth = 0.1f;
 	float objectLength = 0.1f;

 	p1.x = objectWidth/2.0f;
	p1.y = -objectLength/2.0f;
	p1.z = 0.0f;
 	p2.x = -objectWidth/2.0f;
	p2.y = -objectLength/2.0f;
	p2.z = 0.0f;
 	p3.x = -objectWidth/2.0f;
	p3.y = objectLength/2.0f;
	p3.z = 0.0f;
 	p4.x = -objectWidth/2.0f;
	p4.y = objectLength/2.0f;
	p4.z = 0.0f;
 	p5.x = objectWidth/2.0f;
	p5.y = objectLength/2.0f;
	p5.z = 0.0f;
 	p6.x = objectWidth/2.0f;
	p6.y = -objectLength/2.0f;
	p6.z = 0.0f;

	std_msgs::ColorRGBA c;
	c.r = 0.7;
	c.g = 0.7;
	c.b = 0.7;
	c.a = 1.0;

	for(int k = 0; k < 6; k++)
	{
		_msgMarker.colors.push_back(c);
	}

	_msgMarker.points.push_back(p1);
	_msgMarker.points.push_back(p2);
	_msgMarker.points.push_back(p3);
	_msgMarker.points.push_back(p4);
	_msgMarker.points.push_back(p5);
	_msgMarker.points.push_back(p6);

  _pubtaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ContactTaskSharedControl/taskAttractor", 1);
  _pubArbitration = _n.advertise<std_msgs::Float32>("ContactTaskSharedControl/arbitration", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ContactTaskSharedControl/plane", 1);

}


void ContactTaskSharedControl::processRightClickEvent(int value, bool newEvent)
{

	std::cerr << "bou" << std::endl;
}

void  ContactTaskSharedControl::computeCommand()
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

  // std::cerr << "alpha:" << _alpha << std::endl;
  // std::cerr << "vuser: " << _vuser.transpose() << std::endl;

  std::cerr << "vtask: "<<  _vtask.transpose() << std::endl;
  std::cerr << "omegades : "<<  _omegades.transpose() << std::endl;

  // _vdes = (1.0f-_alpha)*_vuser+_alpha*_vtask;
  _vdes = _vtask;
}

void ContactTaskSharedControl::computeAutonomy()
{

  
  float d, c, ct, ch;

    d = (_taskAttractor-_pcur).norm();
    Eigen::Vector3f vu, vt;

    if(_vuser.norm()> 1e-6f)
    {
      vu = _vuser/_vuser.norm();
    }
    else
    {
      vu.setConstant(0.0f);
    }

    if(_vtask.norm()> 1e-6f)
    {
      vt = _vtask/_vtask.norm();
    }
    else
    {
      vt.setConstant(0.0f);
    }


    ch = _agreementWeight*_vtask.dot(_vuser);
    ct = _attractorWeight*std::exp(-(d-_d1)/_d2);
    
    c = ch+ct;
    // std::cerr << "c: " << c << " ch: " << ch <<" ct: " << ct << std::endl;
    // _alpha = 

  // _alpha = std::exp(-(d-_d1)/_d2-2.0f*_vuser.norm());

  // _alpha = (_vuser/_vuser.norm()).dot(_vtask/_vtask.norm())+std::exp(-(d-_d1)/_d2);

  // ct.maxCoeff(&_taskId);
  _alpha = c;

  if(_alpha>_arbitrationLimit)
  {
    _alpha = _arbitrationLimit;
  }
  else if(_alpha < 0.0f)
  {
    _alpha = 0.0f;
  }

}

void ContactTaskSharedControl::computeTaskVelocity()
{
    float alpha = 4.0f;
    float omega = M_PI;
    float r = 0.05f;

  Eigen::Vector3f x = _pcur-_taskAttractor;
  float R = sqrt(x(0) * x(0) + x(1) * x(1));
  float T = atan2(x(1), x(0));
  // float vx = -alpha*(R-r) * cos(T) - R * omega * sin(T);
  // float vy = -alpha*(R-r) * sin(T) + R * omega * cos(T);
  // float vz = -alpha*x(2);
  float vx = -alpha*x(0);
  float vy = -alpha*x(1);
  float vz = -alpha*x(2);
  _vtask << vx, vy, vz;

  if (_vtask.norm() > 0.15f) 
  {
  _vtask = _vtask / _vtask.norm()*0.15f;
  }
  primaryTask();

}


void ContactTaskSharedControl::primaryTask()
{
  Eigen::Matrix3f wRb;
  wRb << _Rcur.UnitX().x(), _Rcur.UnitY().x(), _Rcur.UnitZ().x(),
         _Rcur.UnitX().y(), _Rcur.UnitY().y(), _Rcur.UnitZ().y(), 
         _Rcur.UnitX().z(), _Rcur.UnitY().z(), _Rcur.UnitZ().z();

  Eigen::Vector3f direction = wRb.transpose()*(_taskAttractor-_pcur);
  float distance = direction.norm();

  direction /= distance;

  Eigen::Matrix3f orthogonalProjector = Eigen::Matrix3f::Identity()-direction*direction.transpose();
  Eigen::Matrix<float,3,6> L;
  L.block(0,0,3,3) = -orthogonalProjector/distance;
  Eigen::Matrix3f S;

  S << 0.0f, -direction(2), direction(1),
       direction(2), 0.0f, -direction(0),
       -direction(1), direction(0), 0.0f;
  L.block(0,3,3,3) = S;

  Eigen::Matrix<float,6,3> W;
  W.setConstant(0.0f);
  Eigen::JacobiSVD<Eigen::MatrixXf> svd;
  Eigen::MatrixXf singularValues;
  svd = Eigen::JacobiSVD<Eigen::MatrixXf>(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
  singularValues = svd.singularValues();
  // Compute pseudo inverse jacobian matrix
  float tolerance = 1e-6f*std::max(L.rows(),L.cols())*singularValues.array().abs().maxCoeff();

  for(int m = 0; m < std::min(W.rows(),W.cols()); m++)
  {
    if(singularValues(m,0)>tolerance)
    {
      W(m,m) = 1.0f/singularValues(m,0);
    }
    else
    {
      // If a singular value is close to zero, it usually means we are close to a singularity, which might gives big values in the computation
      // of the pseudo inverse matrix => we ignore this value by setting the inverse to zero
      W(m,m) = 0.0f;
    }
  }
  Eigen::Matrix<float,6,3> Linv;
  Linv = svd.matrixV()*W*svd.matrixU().adjoint();

  Eigen::Vector3f sd;
  sd << 0.0f,0.0f,1.0f;

  // Eigen::Vector3f zPlane;
  // zPlane << _Rplane.UnitZ().x(),_Rplane.UnitZ().y(),_Rplane.UnitZ().z();
  // sd += wRb.transpose()*zPlane;
  // sd /= sd.norm();

  Eigen::Matrix<float,6,1> v;
  Eigen::Matrix<float,6,1> gains;
  gains << 5.0f*Eigen::Vector3f::Ones(), 5.0f*Eigen::Vector3f::Ones();

  v = gains.cwiseProduct(Linv*sd);


  Eigen::Vector3f ex, ey;
  ex << 1.0f, 0.0f, 0.0f;
  ey << 0.0f, 1.0f, 0.0f;
  Eigen::Matrix<float, 6,1> n1,n2,n3,n4;
  n1.setConstant(0.0f);
  n2.setConstant(0.0f);
  n3.setConstant(0.0f);
  n4.setConstant(0.0f);

  n1.segment(0,3) = direction;
  n2.segment(3,3) = direction;
  n3.segment(0,3) = -S*ey;
  n3.segment(3,3) = -orthogonalProjector*ey/distance;
  n4.segment(0,3) = S*ex;
  n4.segment(3,3) = orthogonalProjector*ex/distance;

  v+= _vuser(2)*n1+_vuser(1)*n4;

  // Eigen::Vector4f q;
  // q = _qdes;
  // Eigen:: Vector4f wq;
  // wq << 0, v.segment(3,3);
  // Eigen::Vector4f dq = quaternionProduct(q,wq);
  // q += 0.5f*_dt*dq;
  // q /= q.norm();
  // _qdes = q;
  


  _vtask = wRb*(v.segment(0,3));
  // _vtask.setConstant(0.0f);
  _omegades = wRb*(v.segment(3,3));


  std::cerr << direction.transpose() << std::endl;
  _qdes = _qcur;

}

void ContactTaskSharedControl::processCursorEvent(float relX, float relY, bool newEvent)
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
		_pdes(0) = _pcur(0);
    _pdes(1) = _pcur(1);

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
			_vuser(0) = -_linearVelocityLimit;
		}
		else if(relY < -MAX_XY_REL)
		{
			_vuser(0) = _linearVelocityLimit;
		}
		else
		{
			_vuser(0) = -_linearVelocityLimit*relY/MAX_XY_REL;
		}

		if(!_firstButton || !_buttonPressed)
		{
			// If buttons not pressed track desired z position
			// _vuser(0) = -_convergenceRate*(_pcur(0)-_pdes(0));
      _vuser(2) = 0.0f;
		}
	}	
}

void ContactTaskSharedControl::processABButtonEvent(int value, bool newEvent, int direction)
{
  if(!_firstButton)
  {
    _firstButton = true;
  }

  if(!newEvent) // No new event received
  {
    // Track desired position
    // _vuser = -_convergenceRate*(_pcur-_pdes);
    _vuser.setConstant(0.0f);
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
      _vuser(2) = direction*_zVelocity*_count/MAX_XY_REL;
      _pdes(2) = _pcur(2);
    }
    else // Button released
    {
      // Track desired position
      _count = 0;
      _buttonPressed = false;
      _vuser(2) = -_convergenceRate*(_pcur(2)-_pdes(2));
    }
  }
}

void ContactTaskSharedControl::publishData()
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
  _msgDesiredTwist.angular.x = _omegades(0);
  _msgDesiredTwist.angular.y = _omegades(1);
  _msgDesiredTwist.angular.z = _omegades(2);

  _pubDesiredTwist.publish(_msgDesiredTwist);

  // Publish desired orientation
  _msgDesiredOrientation.w = _qdes(0);
  _msgDesiredOrientation.x = _qdes(1);
  _msgDesiredOrientation.y = _qdes(2);
  _msgDesiredOrientation.z = _qdes(3);

  _pubDesiredOrientation.publish(_msgDesiredOrientation);


  _msgTaskAttractor.header.frame_id = "world";
  _msgTaskAttractor.header.stamp = ros::Time::now();
  _msgTaskAttractor.point.x = _taskAttractor(0);
  _msgTaskAttractor.point.y = _taskAttractor(1);
  _msgTaskAttractor.point.z = _taskAttractor(2);
  _pubtaskAttractor.publish(_msgTaskAttractor);
    
  _pubMarker.publish(_msgMarker);

  std_msgs::Float32 msg;

  msg.data = _alpha;
  _pubArbitration.publish(msg);


  _mutex.unlock();


}

void ContactTaskSharedControl::dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
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