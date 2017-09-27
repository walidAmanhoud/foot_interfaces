#include "ContactSurfaceController.h"

ContactSurfaceController* ContactSurfaceController::me = NULL;

ContactSurfaceController::ContactSurfaceController(ros::NodeHandle &n, double frequency):
	_n(n),
	_loopRate(frequency),
	_dt(1.0f/frequency)
{
    me = this;


  _firstRealPoseReceived = false;
  _firstEventReceived = false;
  _lastEvent = foot_interfaces::FootMouseMsg::FM_NONE;
  _firstButton = false;
  _buttonPressed = false;

  // Init control variables
  _pcur.setConstant(0.0f);
	_vuser.setConstant(0.0f);
  _vtask.setConstant(0.0f);
  _pdes.setConstant(0.0f);
  _vdes.setConstant(0.0f);
  _omegades.setConstant(0.0f);
  _qcur.setConstant(0.0f);
  _qdes.setConstant(0.0f);
  _attractorPosition.setConstant(0.0f);
  _taskAttractor << -0.34f, 0.0f, 0.2f;
  _planeNormal << 0.0f, 0.0f, 1.0f;
  _planeTangent << 0.0f, -1.0f, 0.0f;
  _p <<0.0f,0.0f,0.2f;

  _stop = false;

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.ns = "marker_test_triangle_list";
  _msgMarker.id = 0;
  _msgMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  _msgMarker.action = visualization_msgs::Marker::ADD;
  _msgMarker.pose.position.x = _taskAttractor(0);
  _msgMarker.pose.position.y = _taskAttractor(1);
  _msgMarker.pose.position.z = _taskAttractor(2);
  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f R;
  float angle = -M_PI/6.0f;

  R(0,0) = 1.0f;
  R(1,0) = 0.0f;
  R(2,0) = 0.0f;
  R(0,1) = 0.0f;
  R(1,1) = cos(angle);
  R(2,1) = sin(angle);
  R(0,2) = 0.0f;
  R(1,2) = -sin(angle);
  R(2,2) = cos(angle);
  A = A*R;

  _planeNormal = R.col(2);
  _planeTangent = -R.col(1);

  KDL::Rotation Rkdl(A(0,0),A(0,1),A(0,2),
                     A(1,0),A(1,1),A(1,2),
                     A(2,0),A(2,1),A(2,2)); 
  double x,y,z,w;
  Rkdl.GetQuaternion(x,y,z,w); 

  // _msgMarker.pose.orientation.x = 0.0;
  // _msgMarker.pose.orientation.y = 1.0;
  // _msgMarker.pose.orientation.z = 0.0;
  // _msgMarker.pose.orientation.w = 0.0;

  _msgMarker.pose.orientation.x = x;
  _msgMarker.pose.orientation.y = y;
  _msgMarker.pose.orientation.z = z;
  _msgMarker.pose.orientation.w = w;


  _msgMarker.scale.x = 1.0;
  _msgMarker.scale.y = 1.0;
  _msgMarker.scale.z = 1.0;
  _msgMarker.color.a = 1.0;

  geometry_msgs::Point p1,p2,p3,p4,p5,p6;
  float objectWidth = 1.0f;
  float objectLength = 1.0f;
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

}


bool ContactSurfaceController::init() 
{
  // Subscriber definitions
  _subFootMouse= _n.subscribe("/foot_mouse", 100, &ContactSurfaceController::updateFootMouseData, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &ContactSurfaceController::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &ContactSurfaceController::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &ContactSurfaceController::updateMeasuredWrench, this, ros::TransportHints().reliable().tcpNoDelay());


  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredPose = _n.advertise<geometry_msgs::Pose>("fm", 1); 
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ContactSurfaceController/taskAttractor", 1);
  _pubArbitration = _n.advertise<std_msgs::Float32>("ContactSurfaceController/arbitration", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ContactSurfaceController/plane", 1);
  _pubForceNorm = _n.advertise<std_msgs::Float32>("ContactSurfaceController/forceNorm", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("ContactSurfaceController/filteredWrench", 1);

  // Dynamic reconfigure definition
  // _dynRecCallback = boost::bind(&ContactSurfaceController::dynamicReconfigureCallback, this, _1, _2);
  // _dynRecServer.setCallback(_dynRecCallback);

  signal(SIGINT,ContactSurfaceController::stopNode);

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


void ContactSurfaceController::run()
{
  while (!_stop) 
  {
    if(_firstRealPoseReceived)
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

  ros::shutdown();
}


void ContactSurfaceController::stopNode(int sig)
{
  me->_stop = true;
}


void  ContactSurfaceController::computeCommand()
{
  // uint8_t event;
  // int buttonState, relX, relY, relWheel;
  // float filteredRelX = 0.0f, filteredRelY = 0.0f;
  // bool newEvent = false;

  // // If new event received update last event
  // // Otherwhise keep the last one
  // if(_msgFootMouse.event > 0)
  // {
  //   _lastEvent = _msgFootMouse.event;
  //   buttonState = _msgFootMouse.buttonState;
  //   relX = _msgFootMouse.relX;
  //   relY = _msgFootMouse.relY;
  //   relWheel = _msgFootMouse.relWheel;
  //   filteredRelX = _msgFootMouse.filteredRelX;
  //   filteredRelY = _msgFootMouse.filteredRelY;
  //   newEvent = true;
  // }
  // else
  // {
  //   buttonState = 0;
  //   relX = 0;
  //   relY = 0;
  //   relWheel = 0;
  //   filteredRelX = 0;
  //   filteredRelY = 0;
  //   newEvent = false;
  // }

  // event = _lastEvent;

  // // Process corresponding event
  // switch(event)
  // {
  //   case foot_interfaces::FootMouseMsg::FM_BTN_A:
  //   {
  //     processABButtonEvent(buttonState,newEvent,-1.0f);
  //     break;

  //   }
  //   case foot_interfaces::FootMouseMsg::FM_BTN_B:
  //   {
  //     processABButtonEvent(buttonState,newEvent,1.0f);
  //     break;
  //   }
  //   case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
  //   {
  //     // processRightClickEvent(buttonState,newEvent);
  //     break;
  //   }
  //   case foot_interfaces::FootMouseMsg::FM_CURSOR:
  //   {
  //     processCursorEvent(filteredRelX,filteredRelY,newEvent);
  //     break;
  //   }
  //   default:
  //   {
  //     break;
  //   }
  // }

  float delta = 0.01f;

  _dir = -(_pcur-_taskAttractor);
  _dir /= _dir.norm();
  float normalDistance = (_planeNormal*_planeNormal.transpose()*(_pcur-_taskAttractor)).norm();

  Eigen::Vector3f u = _dir.cross(_planeTangent);
  float c = _dir.dot(_planeTangent);
  float s = u.norm();
  u/=s;

  float theta = std::atan2(delta,normalDistance)*std::acos(c)/(M_PI/2.0f);

  Eigen::Matrix3f K = getSkewSymmetricMatrix(u);
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity()+std::sin(theta)*K+(1.0f-cos(theta))*K*K;

  Eigen::Matrix3f D;
  D.col(0) = -_planeNormal;
  D.col(1) = _planeTangent;
  D.col(2) = (D.col(0)).cross(D.col(1));
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  L(0,0) = 2.0f*normalDistance;
  L(1,1) = 0.25f;
  L(2,2) = 0.25f;

  _vdes = (D*L*D.transpose())*(R*_dir);

  if(_vdes.norm()>0.2f)
  {
    _vdes *= 0.2f/_vdes.norm();
  }

  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  s = k.norm();
  k /= s;
  K << 0.0f, -k(2), k(1),
       k(2), 0.0f, -k(0),
       -k(1), k(0), 0.0f;

  c = (-_wRb.col(2)).transpose()*_planeNormal;

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Get axis angle representation
  KDL::Vector vec;


  // Convert eigen matrix intp kdl
  KDL::Rotation Rkdl(Re(0,0),Re(0,1),Re(0,2),
                     Re(1,0),Re(1,1),Re(1,2),
                     Re(2,0),Re(2,1),Re(2,2));  


  float angle = Rkdl.GetRotAngle(vec);

  Eigen::Vector3f omega, damping;
  // Compute angular velocity direction in desired frame
  omega << vec.x(), vec.y(), vec.z();
  omega *= angle;

  // std::cerr << omega.transpose() << std::endl;


  // Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  // quaternionToAxisAngle(qtemp,omega,angle);
  // std::cerr << (angle*omega).transpose() << std::endl;
  // std::cerr << (acos(c)*k).transpose() << std::endl;

  // _omegades = acos(c)*k;

  Eigen::Matrix3f Rdes = Re*_wRb;
  _qdes = rotationMatrixToQuaternion(Rdes);
  // _qdes << w,x,y,z;
  // std::cerr << _taskAttractor.transpose() << std::endl;
  std::cerr << "d: " << normalDistance << " vdes: " << _vdes.transpose() << " norm: " << _vdes.norm() << std::endl;
}


void ContactSurfaceController::processCursorEvent(float relX, float relY, bool newEvent)
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


void ContactSurfaceController::processABButtonEvent(int value, bool newEvent, int direction)
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


void ContactSurfaceController::processRightClickEvent(int value, bool newEvent)
{
  std::cerr << "bou" << std::endl;
}


void ContactSurfaceController::publishData()
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
  _pubTaskAttractor.publish(_msgTaskAttractor);
    

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.pose.position.x = _taskAttractor(0);
  _msgMarker.pose.position.y = _taskAttractor(1);
  _msgMarker.pose.position.z = _taskAttractor(2);
  _pubMarker.publish(_msgMarker);

  std_msgs::Float32 msg;

  msg.data = _alpha;
  _pubArbitration.publish(msg);

  msg.data = _forceNorm;
  _pubForceNorm.publish(msg);

  _msgFilteredWrench.header.frame_id = "world";
  _msgFilteredWrench.header.stamp = ros::Time::now();
  _msgFilteredWrench.wrench.force.x = _filteredWrench(0);
  _msgFilteredWrench.wrench.force.y = _filteredWrench(1);
  _msgFilteredWrench.wrench.force.z = _filteredWrench(2);
  _msgFilteredWrench.wrench.torque.x = _filteredWrench(3);
  _msgFilteredWrench.wrench.torque.y = _filteredWrench(4);
  _msgFilteredWrench.wrench.torque.z = _filteredWrench(5);
  _pubFilteredWrench.publish(_msgFilteredWrench);

  _mutex.unlock();
}


void ContactSurfaceController::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
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
    _dir = _wRb.col(2);
    float s = -(_planeNormal.dot(_pcur-_p))/(_planeNormal.dot(_dir));
    _taskAttractor = _pcur+s*_dir;
    _angle = std::acos(_planeTangent.dot(_dir));
  }
}


void ContactSurfaceController::updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg)
{
  _msgFootMouse = *msg;

  if(!_firstEventReceived && _msgFootMouse.event > 0)
  {
    _firstEventReceived = true;
  }
}

 
void ContactSurfaceController::updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Matrix<float,6,1> raw;
  raw(0) = msg->wrench.force.x;
  raw(1) = msg->wrench.force.y;
  raw(2) = msg->wrench.force.z;
  raw(3) = msg->wrench.torque.x;
  raw(4) = msg->wrench.torque.y;
  raw(5) = msg->wrench.torque.z;

  if(!_wrenchBiasOK && _firstRealPoseReceived)
  {
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrenchBias.segment(0,3) -= loadForce;
    _wrenchBias.segment(3,3) -= _loadOffset.cross(loadForce);
    _wrenchBias += raw; 
    _wrenchCount++;
    if(_wrenchCount==NB_SAMPLES)
    {
      _wrenchBias /= NB_SAMPLES;
      _wrenchBiasOK = true;
      std::cerr << "Bias: " << _wrenchBias.transpose() << std::endl;
    }
  }

  if(_wrenchBiasOK && _firstRealPoseReceived)
  {
    _wrench = raw-_wrenchBias;
    Eigen::Vector3f loadForce = _wRb.transpose()*_loadMass*_gravity;
    _wrench.segment(0,3) -= loadForce;
    _wrench.segment(3,3) -= _loadOffset.cross(loadForce);
    _filteredWrench = _filteredGain*_filteredWrench+(1.0f-_filteredGain)*_wrench;
  }

}


void ContactSurfaceController::updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _twist(0) = msg->linear.x;
  _twist(1) = msg->linear.y;
  _twist(2) = msg->linear.z;
  _twist(3) = msg->angular.x;
  _twist(4) = msg->angular.y;
  _twist(5) = msg->angular.z;
}


// void ContactSurfaceController::dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
// {
//   ROS_INFO("Reconfigure request bou. Updatig the parameters ...");

//   _arbitrationLimit = config.arbitrationLimit;
//   _agreementWeight = config.agreementWeight;
//   _attractorWeight = config.attractorWeight;
//   _d1 = config.d1;
//   _d2 = config.d2;
//   _linearSpeedGain = config.linearSpeedGain;
//   _angularSpeedGain = config.angularSpeedGain;
//   _filteredGain = config.filteredGain;
//   _contactForceThreshold = config.contactForceThreshold;
//   _usePid = config.usePid;
//   _targetForce = config.targetForce;
//   _pidLimit = config.pidLimit;
//   _kp = config.kp;
//   _ki = config.ki;
//   _kd = config.kd;

//   _convergenceRate = config.convergenceRate;
//   _zVelocity = config.zVelocity;
//   _linearVelocityLimit = config.linearVelocityLimit;
//   _angularVelocityLimit = config.angularVelocityLimit;
//   _modeThreeTranslation = config.modeThreeTranslation;

// }


Eigen::Vector4f ContactSurfaceController::quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
  Eigen::Vector4f q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Vector3f q1Im = (q1.segment(1,3));
  Eigen::Vector3f q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


Eigen::Matrix3f ContactSurfaceController::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}

Eigen::Vector4f ContactSurfaceController::rotationMatrixToQuaternion(Eigen::Matrix3f R)
{
  Eigen::Vector4f q;

  float r11 = R(0,0);
  float r12 = R(0,1);
  float r13 = R(0,2);
  float r21 = R(1,0);
  float r22 = R(1,1);
  float r23 = R(1,2);
  float r31 = R(2,0);
  float r32 = R(2,1);
  float r33 = R(2,2);


  float tr = r11+r22+r33;
  float tr1 = r11-r22-r33;
  float tr2 = -r11+r22-r33;
  float tr3 = -r11-r22+r33;

  if(tr>0)
  {  
    q(0) = sqrt(1.0f+tr)/2.0f;
    q(1) = (r32-r23)/(4.0f*q(0));
    q(2) = (r13-r31)/(4.0f*q(0));
    q(3) = (r21-r12)/(4.0f*q(0));
  }
  else if((tr1>tr2) && (tr1>tr3))
  {
    q(1) = sqrt(1.0f+tr1)/2.0f;
    q(0) = (r32-r23)/(4.0f*q(1));
    q(2) = (r21+r12)/(4.0f*q(1));
    q(3) = (r31+r13)/(4.0f*q(1));
  }     
  else if((tr2>tr1) && (tr2>tr3))
  {   
    q(2) = sqrt(1.0f+tr2)/2.0f;
    q(0) = (r13-r31)/(4.0f*q(2));
    q(1) = (r21+r12)/(4.0f*q(2));
    q(3) = (r32+r23)/(4.0f*q(2));
  }
  else
  {
    q(3) = sqrt(1.0f+tr3)/2.0f;
    q(0) = (r21-r12)/(4.0f*q(3));
    q(1) = (r31+r13)/(4.0f*q(3));
    q(2) = (r32+r23)/(4.0f*q(3));        
  }

  return q;
}


Eigen::Matrix3f ContactSurfaceController::quaternionToRotationMatrix(Eigen::Vector4f q)
{
  Eigen::Matrix3f R;

  float q0 = q(0);
  float q1 = q(1);
  float q2 = q(2);
  float q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


void ContactSurfaceController::quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle)
{
  if((q.segment(1,3)).norm() < 1e-3f)
  {
    axis = q.segment(1,3);
  }
  else
  {
    axis = q.segment(1,3)/(q.segment(1,3)).norm();
    
  }

  angle = 2*std::acos(q(0));

}