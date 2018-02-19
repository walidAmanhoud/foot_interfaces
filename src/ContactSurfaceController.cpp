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
  _taskAttractor << -0.6f, -0.3f, 0.186f;
  // _taskAttractor << -0.74f, -0.170f, 0.186f;
  _planeNormal << 0.0f, 0.0f, 1.0f;
  _planeTangent << 0.0f, -1.0f, 0.0f;
  _p <<0.0f,0.0f,0.186f;

  _stop = false;

  _msgMarker.header.frame_id = "world";
  _msgMarker.header.stamp = ros::Time();
  _msgMarker.ns = "marker_test_triangle_list";
  _msgMarker.id = 0;
  _msgMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  _msgMarker.action = visualization_msgs::Marker::ADD;
  _msgMarker.pose.position.x = _p(0);
  _msgMarker.pose.position.y = _p(1);
  _msgMarker.pose.position.z = _p(2);
  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f R;
  // float angle = -M_PI/6.0f;
  // angle = 0.0f;

  // R(0,0) = 1.0f;
  // R(1,0) = 0.0f;
  // R(2,0) = 0.0f;
  // R(0,1) = 0.0f;
  // R(1,1) = cos(angle);
  // R(2,1) = sin(angle);
  // R(0,2) = 0.0f;
  // R(1,2) = -sin(angle);
  // R(2,2) = cos(angle);
  // A = A*R;

  // _planeNormal = R.col(2);
  // _desiredDir << 0.0f,-1.0f,0.0f;
  // _planeTangent = -R.col(1);

  // KDL::Rotation Rkdl(A(0,0),A(0,1),A(0,2),
  //                    A(1,0),A(1,1),A(1,2),
  //                    A(2,0),A(2,1),A(2,2)); 
  // double x,y,z,w;
  // Rkdl.GetQuaternion(x,y,z,w); 

  _msgMarker.pose.orientation.x = 0.0;
  _msgMarker.pose.orientation.y = 1.0;
  _msgMarker.pose.orientation.z = 0.0;
  _msgMarker.pose.orientation.w = 0.0;

  // _msgMarker.pose.orientation.x = x;
  // _msgMarker.pose.orientation.y = y;
  // _msgMarker.pose.orientation.z = z;
  // _msgMarker.pose.orientation.w = w;


  _msgMarker.scale.x = 1.0;
  _msgMarker.scale.y = 1.0;
  _msgMarker.scale.z = 1.0;
  _msgMarker.color.a = 1.0;

  geometry_msgs::Point p1,p2,p3,p4,p5,p6;
  // float objectWidth = 1.0f;
  // float objectLength = 1.0f;
  float objectWidth = 0.59f;
  float objectLength = 0.82f;
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

  _wrenchBiasOK = false;
  _wrenchCount = 0;
  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.046f;
  _wrenchBias.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);
  _contactForce.setConstant(0.0f);
  _fc = 0.0f;
  _fdis = 0.0f;
  _xf = 0.0f;
  _xOffset = 0.0f;
  _yOffset = 0.0f;

  _Ma.setIdentity();
  _Ma *= 2.0f;
  _Da.setIdentity();
  _Da *= 8.0f;
  _maxAcc = 2.0f;
  _xaDot.setConstant(0.0f);

  _inertiaGain = 0.0f;
  _dampingGain = 0.0f;

  _fm = 0.0f;
}


bool ContactSurfaceController::init() 
{
  // Subscriber definitions
  _subFootMouse= _n.subscribe("/foot_mouse", 100, &ContactSurfaceController::updateFootMouseData, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealPose = _n.subscribe("/lwr/ee_pose", 1, &ContactSurfaceController::updateRealPose, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &ContactSurfaceController::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &ContactSurfaceController::updateMeasuredWrench, this, ros::TransportHints().reliable().tcpNoDelay());

  _subOptitrackRobotBasisPose = _n.subscribe("/optitrack/robot/pose", 1, &ContactSurfaceController::updateRobotBasisPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane1Pose = _n.subscribe("/optitrack/plane1/pose", 1, &ContactSurfaceController::updatePlane1Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane2Pose = _n.subscribe("/optitrack/plane2/pose", 1, &ContactSurfaceController::updatePlane2Pose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackPlane3Pose = _n.subscribe("/optitrack/plane3/pose", 1, &ContactSurfaceController::updatePlane3Pose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publisher definitions
  _pubDesiredTwist = _n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 1);
  _pubDesiredWrench = _n.advertise<geometry_msgs::Wrench>("/lwr/joint_controllers/passive_ds_command_force", 1);
  _pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
  _pubDesiredPose = _n.advertise<geometry_msgs::Pose>("fm", 1); 
  _pubTaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ContactSurfaceController/taskAttractor", 1);
  _pubArbitration = _n.advertise<std_msgs::Float32>("ContactSurfaceController/arbitration", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ContactSurfaceController/plane", 1);
  _pubForceNorm = _n.advertise<std_msgs::Float32>("ContactSurfaceController/forceNorm", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("ContactSurfaceController/filteredWrench", 1);

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&ContactSurfaceController::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

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
    // if(_firstRealPoseReceived && _wrenchBiasOK)
    if(_firstRealPoseReceived && _wrenchBiasOK && _firstEventReceived &&//)// &&
       _firstRobotBasisPose && _firstPlane1Pose &&
       _firstPlane2Pose && _firstPlane3Pose)
    {

      _mutex.lock();
      // Compute control command
      computeCommand();

      // Publish data to topics
      publishData();

      _mutex.unlock();
    }

    ros::spinOnce();

    _loopRate.sleep();
  }

  _vdes.setConstant(0.0f);
  _omegades.setConstant(0.0f);
  _qdes = _qcur;
  _contactForce.setConstant(0.0f);

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
    // case foot_interfaces::FootMouseMsg::FM_BTN_A:
    // {
    //   processABButtonEvent(buttonState,newEvent,-1.0f);
    //   break;

    // }
    // case foot_interfaces::FootMouseMsg::FM_BTN_B:
    // {
    //   processABButtonEvent(buttonState,newEvent,1.0f);
    //   break;
    // }
    // case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
    // {
    //   // processRightClickEvent(buttonState,newEvent);
    //   break;
    // }
    // // case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
    // // {
    // //   processABButtonEvent(buttonState,newEvent,1.0f);
    // //   // processRightClickEvent(buttonState,newEvent);
    // //   break;
    // // }
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

  // autonomousControl();
  admittanceControl();
}

void  ContactSurfaceController::autonomousControl2()
{

 // Extract linear speed, force and torque data
  Eigen::Vector3f vcur = _twist.segment(0,3);
  Eigen::Vector3f force = _filteredWrench.segment(0,3);  
  Eigen::Vector3f torque = _filteredWrench.segment(3,3);

  Eigen::Vector3f fn = (_wRb.col(2)*_wRb.col(2).transpose())*(_wRb*force);
  Eigen::Matrix3f orthogonalProjector = Eigen::Matrix3f::Identity()-_wRb.col(2)*_wRb.col(2).transpose();

  Eigen::Vector3f v = orthogonalProjector*vcur;

  Eigen::Vector3f frictionComponent;
  // Eigen::Vector3f vb = _vuser(1)*_wRb.col(1);
  Eigen::Vector3f tangentialForce;
  Eigen::Vector3f vmes = _twist.segment(0,3);

  if(v.norm()>1e-2f)// || (force.norm() > _targetForce && _twist.segment(0,3).norm() > 1e-3f))
  {
    // frictionComponent = force.dot(vb/vb.norm())*vb/vb.norm(); 
    frictionComponent = orthogonalProjector*force;
  }
  else
  {
    frictionComponent.setConstant(0.0f);
  }
  fn = _wRb*(force-frictionComponent);
  Eigen::Vector3f normalDirection = fn/fn.norm();

  if(force.norm()> _contactForceThreshold)
  { 
    // if(v.norm()>1e-2)
    // {
    //   _planeNormal = fn/fn.norm(); 
    //       // std::cerr << "fn: " <<  (fn/fn.norm()).transpose() << std::endl;
    // }
    // else
    {
      // _planeNormal = _wRb*force/force.norm();
      _planeNormal = normalDirection;
    }

  }
  // else
  // {
  //   _planeNormal << 0.0f,0.0f,1.0f;
  // }
  std::cerr << "plane normal: " << _planeNormal.transpose() << " v: " << v.norm() <<std::endl;
  std::cerr << "plane normal 2: " << normalDirection.transpose() <<std::endl;

  // Compute projected force and speed along plane normal
  // fn = (_planeNormal*_planeNormal.transpose())*force;
  Eigen::Vector3f vn = (_planeNormal*_planeNormal.transpose())*vcur;

  // Compute norm of the projected force along plane normal
  _forceNorm = fn.norm();

  _p1 = _plane1Position-_robotBasisPosition;
  _p2 = _plane2Position-_robotBasisPosition;
  _p3 = _plane3Position-_robotBasisPosition;

  // Compute normal distance to the plane
  float normalDistance = 0.0f;

  // Compute canonical basis used to decompose the desired dynamics along the plane frame
  // D = [-n o p]
  Eigen::Vector3f temp;
  temp << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B;
  B.col(0) = -_planeNormal;
  B.col(1) = (Eigen::Matrix3f::Identity()-_planeNormal*_planeNormal.transpose())*temp;
  B.col(1) = B.col(1)/(B.col(1).norm());
  B.col(2) = (B.col(0)).cross(B.col(1));

  // Compute Weighted diagonal matrix
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  L(0,0) = _convergenceRate;
  L(1,1) = _convergenceRate*(1-std::tanh(50*normalDistance));
  L(2,2) = _convergenceRate*(1-std::tanh(50*normalDistance));
  // L(1,1) = _convergenceRate*(1-std::tanh(50*vn.norm()));
  // L(2,2) = _convergenceRate*(1-std::tanh(50*vn.norm()));
  std::cerr << "normalDistance: " << normalDistance << " L: "<< L.diagonal().transpose() << std::endl;

  // Compute fixed attractor on plane
  Eigen::Vector3f xa;
  // xa = _p1+0.8*(_p2-_p1)+0.5*(_p3-_p1);
  xa = _taskAttractor;
  // xa(0) += _xOffset;
  // xa(1) += _yOffset;
  // xa(2) = (-_planeNormal(0)*(xa(0)-_p1(0))-_planeNormal(1)*(xa(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  // xa(2) = (-_planeNormal(0)*(xa(0)-_p(0))-_planeNormal(1)*(xa(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  std::cerr << "xa: " << xa.transpose() << std::endl;


  // Check if polishing motion is activated and compute desired velocity based on motion dynamics
  // = sum of linear dynamics coming from moving and fixed attractors
  if(_polishing)
  {
    // _vdes = (B*L*B.transpose())*((Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*(_vuser(1)*_wRb.col(0)+_vuser(0)*_wRb.col(1)));
    _vdes = (B*L*B.transpose())*((Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*getDesiredVelocity(_pcur,xa));
    // _vdes.setConstant(0.0f);
    std::cerr << _vuser(1) <<  " " << _vuser(0) << std::endl;
  }
  else
  {
    // _vdes = (B*L*B.transpose())*((_xp-_pcur));
    _vdes.setConstant(0.0f);
    // _vdes = (B*L*B.transpose())*((_xp-_pcur)-_xf*_planeNormal);
  }
  // _vdes.setConstant(0.0f);

  // Check if force control activated and compute contact force following the specified dynamics
  if(_controlForce)
  {
    // Compute force error
    float error = _targetForce-_forceNorm;
    
    // Compute force stiffness rate gain from measured speed
    float alpha = 1-std::tanh(50*vn.norm());

    // Compute force damping damping rate gain from measured speed
    float beta = 1-alpha;

    // Integrate contact force dynamics
    _fc += _dt*(_forceStiffnessRateGain*alpha*error-_forceDampingRateGain*beta*_targetForce);

    // Saturate contact force
    // std::tanh(0.4*_forceNorm)
    // if(_fc < -20*(alpha))
    // {
    //   _fc = -20*(alpha);    
    // if(_fc < -20)
    // {
    //   _fc = -20;
    if(_fc < -15*(alpha+std::tanh(0.4*_forceNorm)))
    {
      _fc = -15*(alpha+std::tanh(0.4*_forceNorm));
    }
    else if(_fc > 30.0f)
    {
      _fc = 30.0f;
    }

    // float alpha = 1-std::tanh(50*vn.norm());
    // float beta = -std::tanh(50*vn.norm());

    // _xf += _dt*(_A*alpha*error+_C*beta*_targetForce);

    // if(_xf < -0.1*alpha)
    // {
    //   _xf = -0.1*alpha;
    // }
    // else if(_xf > 0.3f)
    // {
    //   _xf = 0.3f;
    // }
    
    // std::cerr << "d: " << normalDistance << " xf: " << _xf << " alpha: " << alpha << std::endl;
    // std::cerr << "vn: " << vn.norm() << std::endl;

    std::cerr << "alpha: " << alpha << " fc: " << _fc << " vn: " << vn.norm() << std::endl;
  }
  else
  {
    // _xf = 0.0f;
    _fc = 0.0f;
    _contactForce.setConstant(0.0f);
  }



  if(_splitForceFromMotion)
  {
    // _contactForce = _fc*B.col(0);
    _contactForce = _fc*_wRb.col(2);
  }
  else
  {
    // Reform the damping matrix for passive ds control
    // float lambda1 = 80;
    // float lambda2 = 50;
    // Eigen::Matrix3f D;
    // Eigen::Vector3f dir;

    // if(_vdes.norm()> 1e-6)
    // {
    //   // if(!_firstControl)
    //   // {
    //   //   _firstControl = true;
    //   //   dir = _vdes/_vdes.norm();
    //   //   // _vdes/_vdes.norm();
    //   // }
    //   // else
    //   // {
    //   //   dir = _dir;
    //   // }
    //   dir = _vdes/_vdes.norm();
    //   D = lambda1*(dir*dir.transpose())+lambda2*(Eigen::Matrix3f::Identity()-dir*dir.transpose());
    // }
    // else
    // {
    //   D = lambda2*Eigen::Matrix3f::Identity();
    // }

    // // // Compute force speed offset using the inverse of the passive ds controller damping matrix
    Eigen::Vector3f vf;
    // vf = D.inverse()*(_fc*_wRb.col(2));
    vf = (_C*_fc*_wRb.col(2));
    _vdes += vf;

    // // Compute force attractor offset using the inverse of the motion dynamics
    // Eigen::Vector3f xf;
    // xf = (B*L*B.transpose()).inverse()*vf;
    // _vdes += _C*_fc*_wRb.col(2);
    // _vdes += (B*L*B.transpose()*xf);

    std::cerr << "vf: " << vf.transpose() << std::endl;
    _contactForce.setConstant(0.0f);
  }

  // Bound speed  
  if(_vdes.norm()>0.3f)
  {
    _vdes *= 0.3f/_vdes.norm();
  }

  // if(_vdes.norm()>1e-6)
  // {
  //   _dir = _vdes/_vdes.norm();
  // }


  // Compute rotation error between current orientation and plane orientation
  // use Rodrigues' law
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  Eigen::Matrix3f K;
  K << 0.0f, -k(2), k(1),
       k(2), 0.0f, -k(0),
       -k(1), k(0), 0.0f;

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Convert rotation error into axis angle representation
  float angle;
  Eigen::Vector3f omega, damping;
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  quaternionToAxisAngle(qtemp,omega,angle);

  std::cerr << angle << std::endl;
  if(fabs(angle)<M_PI/3.0f)
  {
    omega *= angle;

    // Compute damping in world frame
    damping = -fabs(std::acos(c))*_twist.segment(3,3);

    float stiffnessGain = 2.0f;
    float dampingGain = 0.0f;
    float torqueGain = 0.6f;
    _omegades = stiffnessGain*omega + dampingGain*damping+torqueGain*_wRb*torque;
    _qdes = _qcur;    
    
  }
  else
  {
    _contactForce.setConstant(0.0f);
    _fc = 0.0f;
    _omegades.setConstant(0.0f);
    _qdes = _qcur;
    std::cerr <<"bou" << std::endl;
  }

  // _omegades.setConstant(0.0f);
  // _qdes = _qcur;

}

void  ContactSurfaceController::autonomousControl()
{
  
  // Extract linear speed, force and torque data
  Eigen::Vector3f vcur = _twist.segment(0,3);
  Eigen::Vector3f force = _filteredWrench.segment(0,3);  
  Eigen::Vector3f torque = _filteredWrench.segment(3,3);

  // Compute plane normal form markers position
  _p1 = _plane1Position-_robotBasisPosition;
  _p2 = _plane2Position-_robotBasisPosition;
  _p3 = _plane3Position-_robotBasisPosition;
  Eigen::Vector3f p13,p12;
  p13 = _p3-_p1;
  p12 = _p2-_p1;
  p13 /= p13.norm();
  p12 /= p12.norm();
  _planeNormal = p13.cross(p12);

  std::cerr << "plane normal: " << _planeNormal.transpose() << std::endl;

  // Compute projected force and speed along plane normal
  Eigen::Vector3f fn = (_planeNormal*_planeNormal.transpose())*(_wRb*force);
  Eigen::Vector3f vn = (_planeNormal*_planeNormal.transpose())*vcur;

  // Compute norm of the projected force along plane normal
  _forceNorm = fn.norm();

  // Compute vertical projection of the current position onto the plane
  _xp = _pcur;
  _xp(2) = (-_planeNormal(0)*(_xp(0)-_p3(0))-_planeNormal(1)*(_xp(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
  // _xp(2) = (-_planeNormal(0)*(_xp(0)-_p(0))-_planeNormal(1)*(_xp(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);

  // Compute normal distance to the plane
  float normalDistance = (_planeNormal*_planeNormal.transpose()*(_pcur-_xp)).norm();

  // Compute canonical basis used to decompose the desired dynamics along the plane frame
  // D = [-n o p]
  Eigen::Vector3f temp;
  temp << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B;
  B.col(0) = -_planeNormal;
  B.col(1) = (Eigen::Matrix3f::Identity()-_planeNormal*_planeNormal.transpose())*temp;
  B.col(1) = B.col(1)/(B.col(1).norm());
  B.col(2) = (B.col(0)).cross(B.col(1));

  // Compute Weighted diagonal matrix
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  L(0,0) = _convergenceRate*(1-std::tanh(10*normalDistance));
  // L(0,0) = 1.0f;
  L(1,1) = _convergenceRate*(1-std::tanh(50*normalDistance));
  L(2,2) = _convergenceRate*(1-std::tanh(50*normalDistance));
  // L(1,1) = _convergenceRate*(1-std::tanh(50*vn.norm()));
  // L(2,2) = _convergenceRate*(1-std::tanh(50*vn.norm()));
  std::cerr << "normalDistance: " << normalDistance << " L: "<< L.diagonal().transpose() << std::endl;

  // Compute fixed attractor on plane
  Eigen::Vector3f xa;
  xa = _p1+0.5*(_p2-_p1)+0.5*(_p3-_p1);

  if(_linear || _polishing)
  {
    _xOffset += _vuser(0)*_dt;
    _yOffset += _vuser(1)*_dt;
    std::cerr << _xOffset << " " << _yOffset << std::endl;
    xa(0) += _xOffset;
    xa(1) += _yOffset;
  }

  // xa = _taskAttractor;
  // xa(0) += _xOffset;
  // xa(1) += _yOffset;
  xa(2) = (-_planeNormal(0)*(xa(0)-_p1(0))-_planeNormal(1)*(xa(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  // xa(2) = (-_planeNormal(0)*(xa(0)-_p(0))-_planeNormal(1)*(xa(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  std::cerr << "xa: " << xa.transpose() << std::endl;


  // Check if polishing motion is activated and compute desired velocity based on motion dynamics
  // = sum of linear dynamics coming from moving and fixed attractors

  float ch = std::max(0.0f, 1-normalDistance/0.1f);
  if(_polishing)
  {
    _vdes = (B*L*B.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*getDesiredVelocity(_pcur,xa));
    // _vdes = (B*L*B.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*getDesiredVelocity(_pcur,xa)-_xf*_planeNormal);
  }
  else if(_linear)
  {
    _vdes = (B*L*B.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*(xa-_pcur));
  }
  else
  {
    _vdes = (B*L*B.transpose())*((_xp-_pcur));
    // _vdes = (B*L*B.transpose())*((_xp-_pcur)-_xf*_planeNormal);
  }

  std::cerr << "vdes: " << _vdes.transpose() << "ch: " << ch << std::endl;

  // Check if force control activated and compute contact force following the specified dynamics
  if(_controlForce)
  {
    // Compute force error
    float error = _targetForce-_forceNorm;
    
    // Compute force stiffness rate gain from measured speed
    float alpha = 1-std::tanh(50*vn.norm());

    // Compute force damping damping rate gain from measured speed
    float beta = 1-alpha;

    // Integrate contact force dynamics
    _fc += _dt*(_forceStiffnessRateGain*alpha*error-_forceDampingRateGain*beta*_targetForce);

    // Saturate contact force
    // std::tanh(0.4*_forceNorm)
    // if(_fc < -20*(alpha))
    // {
    //   _fc = -20*(alpha);    
    // if(_fc < -20)
    // {
    //   _fc = -20;

    float gamma = std::tanh(0.4*_forceNorm);
    if(_fc < -15*(alpha))
    {
      _fc = -15*(alpha);
    }
    else if(_fc > 30.0f)
    {
      _fc = 30.0f;
    }

    // float alpha = 1-std::tanh(50*vn.norm());
    // float beta = -std::tanh(50*vn.norm());

    // _xf += _dt*(_A*alpha*error+_C*beta*_targetForce);

    // if(_xf < -0.1*alpha)
    // {
    //   _xf = -0.1*alpha;
    // }
    // else if(_xf > 0.3f)
    // {
    //   _xf = 0.3f;
    // }
    
    // std::cerr << "d: " << normalDistance << " xf: " << _xf << " alpha: " << alpha << std::endl;
    // std::cerr << "vn: " << vn.norm() << std::endl;

    std::cerr << "alpha: " << alpha << " gamma: " << gamma << " fc: " << _fc << " vn: " << vn.norm() << std::endl;
  }
  else
  {
    // _xf = 0.0f;
    _fc = 0.0f;
    _contactForce.setConstant(0.0f);
  }



  if(_splitForceFromMotion)
  {
    // _contactForce = _fc*B.col(0);
    _contactForce = _fc*_wRb.col(2);
  }
  else
  {
    // Reform the damping matrix for passive ds control
    // float lambda1 = 80;
    // float lambda2 = 50;
    // Eigen::Matrix3f D;
    // Eigen::Vector3f dir;

    // if(_vdes.norm()> 1e-6)
    // {
    //   // if(!_firstControl)
    //   // {
    //   //   _firstControl = true;
    //   //   dir = _vdes/_vdes.norm();
    //   //   // _vdes/_vdes.norm();
    //   // }
    //   // else
    //   // {
    //   //   dir = _dir;
    //   // }
    //   dir = _vdes/_vdes.norm();
    //   D = lambda1*(dir*dir.transpose())+lambda2*(Eigen::Matrix3f::Identity()-dir*dir.transpose());
    // }
    // else
    // {
    //   D = lambda2*Eigen::Matrix3f::Identity();
    // }

    // // // Compute force speed offset using the inverse of the passive ds controller damping matrix
    Eigen::Vector3f vf;
    // vf = D.inverse()*(_fc*_wRb.col(2));
    // vf = (_C*_fc*_wRb.col(2));
    vf = (_C*_fc*B.col(2));
    _vdes += vf;

    // // Compute force attractor offset using the inverse of the motion dynamics
    // Eigen::Vector3f xf;
    // xf = (B*L*B.transpose()).inverse()*vf;
    // _vdes += _C*_fc*_wRb.col(2);
    // _vdes += (B*L*B.transpose()*xf);

    std::cerr << "vf: " << vf.transpose() << std::endl;
    _contactForce.setConstant(0.0f);
  }

  // Bound speed  
  if(_vdes.norm()>0.3f)
  {
    _vdes *= 0.3f/_vdes.norm();
  }

  // if(_vdes.norm()>1e-6)
  // {
  //   _dir = _vdes/_vdes.norm();
  // }


  // Compute rotation error between current orientation and plane orientation
  // use Rodrigues' law
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  Eigen::Matrix3f K;
  K << 0.0f, -k(2), k(1),
       k(2), 0.0f, -k(0),
       -k(1), k(0), 0.0f;

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Convert rotation error into axis angle representation
  float angle;
  Eigen::Vector3f omega;
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  quaternionToAxisAngle(qtemp,omega,angle);

  // _omegades = 2.0f*omega*angle;
  // _omegades = 2.0f*omega*angle+0.5f*_wRb*torque;
  // // _qdes = _qcur;

  // Eigen::Vector4f q;
  // q = _qdes;
  // Eigen:: Vector4f wq;
  // wq << 0.0f, _wRb.transpose()*_omegades;
  // std::cerr << wq.transpose() << std::endl; 
  // Eigen::Vector4f dq = quaternionProduct(q,wq);
  // q += 0.5f*_dt*dq;
  // q /= q.norm();
  // _qdes = q;   
  // _omegades = omega;

  // Compute final quaternion on plane
  Eigen::Vector4f qf = quaternionProduct(qtemp,_qcur);

  // Perform quaternion slerp interpolation to progressively orient the end effector while
  // approaching the plane
  _qdes = slerpQuaternion(_qcur,qf,1-std::tanh(5*normalDistance));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _qcur(0);
  qcurI.segment(1,3) = -_qcur.segment(1,3);
  wq = 5.0f*quaternionProduct(qcurI,_qdes-_qcur);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegades = omegaTemp;

  Eigen::Vector3f wt;
  wt = _A*torque;
  // Eigen::Vector4f q = _qdes;
  // wq << 0.0f, wt;
  // Eigen::Vector4f dq = quaternionProduct(q,wq);
  // q += 0.5f*_dt*dq;
  // q /= q.norm();
  // _qdes = q;   
  _omegades += _wRb*wt;
}


void  ContactSurfaceController::admittanceControl()
{
  
  // Extract linear speed, force and torque data
  Eigen::Vector3f vcur = _twist.segment(0,3);
  Eigen::Vector3f force = _filteredWrench.segment(0,3);  
  Eigen::Vector3f torque = _filteredWrench.segment(3,3);

  // Compute plane normal form markers position
  _p1 = _plane1Position-_robotBasisPosition;
  _p2 = _plane2Position-_robotBasisPosition;
  _p3 = _plane3Position-_robotBasisPosition;
  Eigen::Vector3f p13,p12;
  p13 = _p3-_p1;
  p12 = _p2-_p1;
  p13 /= p13.norm();
  p12 /= p12.norm();
  _planeNormal = p13.cross(p12);

  std::cerr << "plane normal: " << _planeNormal.transpose() << std::endl;

  // Compute projected force and speed along plane normal vector
  Eigen::Vector3f fn = (_planeNormal*_planeNormal.transpose())*(_wRb*force);
  Eigen::Vector3f vn = (_planeNormal*_planeNormal.transpose())*vcur;

  // Compute norm of the projected force along plane normal
  _forceNorm = fn.norm();

  // Compute vertical projection of the current position onto the plane
  _xp = _pcur;
  _xp(2) = (-_planeNormal(0)*(_xp(0)-_p3(0))-_planeNormal(1)*(_xp(1)-_p3(1))+_planeNormal(2)*_p3(2))/_planeNormal(2);
  // _xp(2) = (-_planeNormal(0)*(_xp(0)-_p(0))-_planeNormal(1)*(_xp(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);

  // Compute normal distance to the plane
  float normalDistance = (_planeNormal*_planeNormal.transpose()*(_pcur-_xp)).norm();

  // Compute canonical basis used to decompose the desired dynamics along the plane frame
  // D = [-n o p]
  Eigen::Vector3f temp;
  temp << 1.0f,0.0f,0.0f;
  Eigen::Matrix3f B;
  B.col(0) = -_planeNormal;
  B.col(1) = (Eigen::Matrix3f::Identity()-_planeNormal*_planeNormal.transpose())*temp;
  B.col(1) = B.col(1)/(B.col(1).norm());
  B.col(2) = (B.col(0)).cross(B.col(1));

  // Compute Weighted diagonal matrix
  Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  L(0,0) = _convergenceRate*(1-std::tanh(10*normalDistance));
  L(1,1) = _convergenceRate*(1-std::tanh(50*normalDistance));
  L(2,2) = _convergenceRate*(1-std::tanh(50*normalDistance));
  std::cerr << "normalDistance: " << normalDistance << " L: "<< L.diagonal().transpose() << std::endl;

  // Compute fixed attractor on plane
  Eigen::Vector3f xattractor;
  xattractor = _p1+0.5*(_p2-_p1)+0.5*(_p3-_p1);


  // Offset fixed attractor position based on mouse input
  if(_linear || _polishing)
  {
    _xOffset += _vuser(0)*_dt;
    _yOffset += _vuser(1)*_dt;
    std::cerr << _xOffset << " " << _yOffset << std::endl;
    xattractor(0) += _xOffset;
    xattractor(1) += _yOffset;
  }

  // xattractor = _taskAttractor;
  // xattractor(0) += _xOffset;
  // xattractor(1) += _yOffset;
  xattractor(2) = (-_planeNormal(0)*(xattractor(0)-_p1(0))-_planeNormal(1)*(xattractor(1)-_p1(1))+_planeNormal(2)*_p1(2))/_planeNormal(2);
  // xattractor(2) = (-_planeNormal(0)*(xattractor(0)-_p(0))-_planeNormal(1)*(xattractor(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);
  std::cerr << "xattractor: " << xattractor.transpose() << std::endl;


  // Check if polishing motion is activated and compute desired velocity based on motion dynamics
  // = sum of linear dynamics coming from moving and fixed attractors
  Eigen::Vector3f x0Dot;
  if(_polishing)
  {
    x0Dot = (B*L*B.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*getDesiredVelocity(_pcur,xattractor));
    // x0Dot = (B*L*B.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*getDesiredVelocity(_pcur,xattractor)-_xf*_planeNormal);
  }
  else if(_linear)
  {
    x0Dot = (B*L*B.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-B.col(0)*B.col(0).transpose())*(xattractor-_pcur));
  }
  else
  {
    x0Dot = (B*L*B.transpose())*((_xp-_pcur));
    // x0Dot = (B*L*B.transpose())*((_xp-_pcur)-_xf*_planeNormal);
  }


  // Check if force control activated and modify desired dynamics based on force error
  if(_controlForce)
  {

    _fm+= _dt*(_targetForce-_fm);
    _Ma.setIdentity();
    _Ma *= _inertiaGain;
    _Da.setIdentity();
    _Da *= _dampingGain;

    // Compute force error
    Eigen::Vector3f xaDotDot;
    // Eigen::Vector3f forceError = (_targetForce-_forceNorm)*B.col(0);
    Eigen::Vector3f forceError = (_fm-_forceNorm)*B.col(0);



    // Compute force stiffness rate gain from measured speed
    float alpha = 1-std::tanh(50*vn.norm());

    // Compute force damping damping rate gain from measured speed
    float beta = 1-alpha;

    // Integrate contact force dynamics
    // _fc += _dt*(_forceStiffnessRateGain*(_fm-_forceNorm)-_forceDampingRateGain*_fm);
    _fc += _dt*(_forceStiffnessRateGain*(_fm-_forceNorm));

    // if(_fc < -15)
    // {
    //   _fc = -15;
    // }
    // else if(_fc > 30.0f)
    // {
    //   _fc = 30.0f;
    // }

    //Compute admittance acceleration;
    // xaDotDot = _Ma.inverse()*(forceError-_Da*(_xaDot-x0Dot));
    xaDotDot = _Ma.inverse()*(_forceStiffnessRateGain*forceError-_Da*_xaDot);
    // xaDotDot = _Ma.inverse()*(_fc*B.col(0)-_Da*(_xaDot-x0Dot));
    // xaDotDot = _Ma.inverse()*(_fc*B.col(0)-_Da*_xaDot);

    // Bound admittance acceleration
    // if(xaDotDot.norm()>_maxAcc)
    // {
    //   xaDotDot *= _maxAcc/xaDotDot.norm();
    // }

    // Compute admittance velocity
    _xaDot += _dt*xaDotDot;
    _contactForce.setConstant(0.0f);

    // _vdes = _xaDot;
    _vdes = _xaDot+x0Dot;
    std::cerr << "xaDot: " << _xaDot.transpose() << " x0Dot: " << x0Dot.transpose() << std::endl;
    std::cerr << "fc: " << _fc << std::endl;
  }
  else
  {
    _xaDot.setConstant(0.0f);
    _vdes = x0Dot;
    _contactForce.setConstant(0.0f);
    _fm = 0.0f;
    _fc = 0.0f;
  }

  // // // Bound speed  
  // if(_xaDot.norm()>0.3f)
  // {
  //   _xaDot *= 0.3f/_xaDot.norm();
  // }



  // Bound speed  
  if(_vdes.norm()>0.3f)
  {
    _vdes *= 0.3f/_vdes.norm();
  }

  // Compute rotation error between current orientation and plane orientation based on Rodrigues' law
  Eigen::Vector3f k;
  k = (-_wRb.col(2)).cross(_planeNormal);
  float c = (-_wRb.col(2)).transpose()*_planeNormal;  
  float s = k.norm();
  k /= s;
  Eigen::Matrix3f K;
  K << 0.0f, -k(2), k(1),
       k(2), 0.0f, -k(0),
       -k(1), k(0), 0.0f;

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  
  // Compute desired final quaternion on plane
  float angle;
  Eigen::Vector3f omega;
  Eigen::Vector4f qtemp = rotationMatrixToQuaternion(Re);
  Eigen::Vector4f qf = quaternionProduct(qtemp,_qcur);

  // Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
  _qdes = slerpQuaternion(_qcur,qf,1-std::tanh(5*normalDistance));

  // Compute needed angular velocity to perform the desired quaternion
  Eigen::Vector4f qcurI, wq;
  qcurI(0) = _qcur(0);
  qcurI.segment(1,3) = -_qcur.segment(1,3);
  wq = 5.0f*quaternionProduct(qcurI,_qdes-_qcur);
  Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
  _omegades = omegaTemp;

  // Increase angular veloicty to correct angular torque
  Eigen::Vector3f wt;
  wt = _A*torque;  
  _omegades += _wRb*wt;
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
      _vuser(1) = -_userVelocityLimit;
    }
    else if(relX < -MAX_XY_REL)
    {
      _vuser(1) = _userVelocityLimit;
    }
    else
    {
      _vuser(1) = -_userVelocityLimit*relX/MAX_XY_REL;
    }

    if(relY>MAX_XY_REL)
    {
      _vuser(0) = -_userVelocityLimit;
    }
    else if(relY < -MAX_XY_REL)
    {
      _vuser(0) = _userVelocityLimit;
    }
    else
    {
      _vuser(0) = -_userVelocityLimit*relY/MAX_XY_REL;
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
  // _mutex.lock();

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
  Eigen::Vector3f center = _p1+0.5f*(_p2-_p1)+0.5f*(_p3-_p1); 
  _msgMarker.pose.position.x = center(0);
  _msgMarker.pose.position.y = center(1);
  _msgMarker.pose.position.z = center(2);
  Eigen::Vector3f u,v,n;
  u = _p3-_p1;
  v = _p2-_p1;
  u /= u.norm();
  v /= v.norm();
  n = u.cross(v);
  Eigen::Matrix3f R;
  R.col(0) = u;
  R.col(1) = v;
  R.col(2) = n;
  Eigen::Vector4f q = rotationMatrixToQuaternion(R);


  _msgMarker.pose.orientation.x = q(1);
  _msgMarker.pose.orientation.y = q(2);
  _msgMarker.pose.orientation.z = q(3);
  _msgMarker.pose.orientation.w = q(0);

  // _msgMarker.pose.position.x = _p(0);
  // _msgMarker.pose.position.y = _p(1);
  // _msgMarker.pose.position.z = _p(2);
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


  _msgDesiredWrench.force.x = _contactForce(0);
  _msgDesiredWrench.force.y = _contactForce(1);
  _msgDesiredWrench.force.z = _contactForce(2);
  _msgDesiredWrench.torque.x = 0.0f;
  _msgDesiredWrench.torque.y = 0.0f;
  _msgDesiredWrench.torque.z = 0.0f;
  _pubDesiredWrench.publish(_msgDesiredWrench);

  // _mutex.unlock();
}


void ContactSurfaceController::updateRealPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  _msgRealPose = *msg;

  // Update end effecotr pose (position+orientation)
  _pcur << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  _qcur << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
  _wRb = quaternionToRotationMatrix(_qcur);
  _pcur = _pcur+_toolOffset*_wRb.col(2);

  // _xp = _pcur;
  // _xp(2) = (-_planeNormal(0)*(_xp(0)-_p(0))-_planeNormal(1)*(_xp(1)-_p(1))+_planeNormal(2)*_p(2))/_planeNormal(2);

  if(!_firstRealPoseReceived)
  {
    _firstRealPoseReceived = true;
    _pdes = _pcur;
    _qdes = _qcur;
    _vdes.setConstant(0.0f);
    // _dir = _wRb.col(2);
    // float s = -(_planeNormal.dot(_pcur-_p))/(_planeNormal.dot(_dir));
    // _taskAttractor = _pcur+s*_dir;
    // _angle = std::acos(_planeTangent.dot(_dir));
    // xp = xcur;

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
    _filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench;
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


void ContactSurfaceController::updateRobotBasisPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _robotBasisPosition << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  _robotBasisPosition(2) -= 0.025f;
  if(!_firstRobotBasisPose)
  {
    _firstRobotBasisPose = true;
  }
}


void ContactSurfaceController::updatePlane1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane1Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane1Pose)
  {
    _firstPlane1Pose = true;
  }
}


void ContactSurfaceController::updatePlane2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane2Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane2Pose)
  {
    _firstPlane2Pose = true;
  }
}


void ContactSurfaceController::updatePlane3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
  _plane3Position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  if(!_firstPlane3Pose)
  {
    _firstPlane3Pose = true;
  }
}


void ContactSurfaceController::dynamicReconfigureCallback(foot_interfaces::contactSurfaceController_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request bou. Updatig the parameters ...");

  _convergenceRate = config.convergenceRate;
  // _xOffset = config.xOffset;
  // _yOffset = config.yOffset;
  _filteredForceGain = config.filteredForceGain;
  _contactForceThreshold = config.contactForceThreshold;
  _targetForce = config.targetForce;
  _polishing = config.polishing;
  _linear = config.linear;
  _controlForce = config.controlForce;

  // _usePid = config.usePid;
  // _pidLimit = config.pidLimit;
  // _kp = config.kp;
  // _ki = config.ki;
  // _kd = config.kd;
  _A = config.A;
  _B = config.B;
  _C = config.C;
  _splitForceFromMotion = config.splitForceFromMotion;
  _forceStiffnessRateGain = config.forceStiffnessRateGain;
  _forceDampingRateGain = config.forceDampingRateGain;
  _userVelocityLimit = config.userVelocityLimit;
  _inertiaGain = config.inertiaGain;
  _dampingGain = config.dampingGain;

}


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

  Eigen::Vector4f ContactSurfaceController::slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t)
  {

    Eigen::Vector4f q;

    // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
    if(q1.dot(q2)<0.0f)
    {   
      q2 = -q2;
    }

    float dotProduct = q1.dot(q2);
    if(dotProduct > 1.0f)
    {
      dotProduct = 1.0f;
    }
    else if(dotProduct < -1.0f)
    {
      dotProduct = -1.0f;
    }

    float omega = acos(dotProduct);

    if(std::fabs(omega)<FLT_EPSILON)
    {
      q = q1.transpose()+t*(q2-q1).transpose();
    }
    else
    {
      q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
    }

    return q;
  }

  Eigen::Vector3f ContactSurfaceController::getDesiredVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor)
{
  Eigen::Vector3f velocity;

  position = position-attractor;

  velocity(2) = -position(2);

  float R = sqrt(position(0) * position(0) + position(1) * position(1));
  float T = atan2(position(1), position(0));

//  // double alpha = Convergence_Rate_ * Convergence_Rate_scale_;
//  // double omega = Cycle_speed_ + Cycle_speed_offset_;
 float r = 0.025f;
 float omega = M_PI;

  velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
  velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);

  return velocity;
}

  // if(_usePid)
  // {

  //   // _pidError = (_targetForce-force.norm());
  //   // _forceNorm = _filteredWrench.segment(0,3).norm();
  //   _forceNorm = std::fabs(_filteredWrench(2));
  //   _pidError = (_targetForce-_forceNorm);
  //   // _pidInteg += _dt*_pidError*std::tanh(delta/normalDistance);
  //   _pidInteg += _dt*_pidError*std::tanh(0.00001f/vcur.norm());
  //   _up = _kp*_pidError;
  //   _ui = _ki*_pidInteg;
  //   _ud = _kd*(_pidError-_pidLastError)/_dt;
  //   _pidLastError = _pidError;

  //   if(_ui < -_pidLimit)
  //   {
  //     _ui = -_pidLimit;
  //   }
  //   else if(_ui > _pidLimit)
  //   {
  //     _ui = _pidLimit;
  //   }

  //   // command = _up+_ui+_ud;
  //   command = _ui;
  //   if(command<-_pidLimit)
  //   {
  //     command = -_pidLimit;
  //   }
  //   else if(command>_pidLimit)
  //   {
  //     command = _pidLimit;
  //   }
  // }
  // else
  // {
  //   command = 0.0f;
  //   _ui = 0.0f;
  // }


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

    // float error = _targetForce-fabs(_filteredWrench(2));
    // float alpha = (1.0f-std::tanh(50*normalDistance));
    // float beta = (-std::tanh(50*normalDistance));
    // float alpha = -std::tanh(_B*(vn.norm()-_C));
    // _xf += _dt*(0.04*alpha*error+0.01*beta*fabs(_filteredWrench(2)));
    // _xf += _dt*(_A*alpha*error+_C*beta*_forceNorm;
    // _xf += _dt*(alpha*error-vn.norm())*0.01;
    // _xf += _dt*(alpha*error)*0.01;
    // _fc += _A*(alpha*error+beta*_targetForce);

  // L(1,1) = _convergenceRate*(1-std::tanh(50*normalDistance))*(1-std::tanh(0.1*fabs(_targetForce-_forceNorm)));
  // L(2,2) = _convergenceRate*(1-std::tanh(50*normalDistance))*(1-std::tanh(0.1*fabs(_targetForce-_forceNorm)));

    // _xf += _dt*(_D*alpha*error+_E*beta*fn.norm());
    // _xf += _dt*(alpha*error-vn.norm())*0.01;
    // _xf += _dt*(alpha*error)*0.01;
    // _fc += _A*(alpha*error+beta*_targetForce);

  // _vdes = (D*L*D.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-D.col(0)*D.col(0).transpose())*(xa-_pcur));
  // _vdes = (D*L*D.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-D.col(0)*D.col(0).transpose())*(xa-_pcur)-_xf*_planeNormal);
  // _vdes = (D*L*D.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-D.col(0)*D.col(0).transpose())*(xa-_pcur)+_xf*_wRb.col(2));

// float gamma = -std::tanh(10*(fabs(_filteredWrench(2))-2.0f));
    // // float gamma = 5.0f;
    // _fdis += gamma*(0.01-vcur.norm());

    // if(_fdis < 0)
    // {
    //   _fdis = 0;
    // }
    // else if(_fdis > 30.0f)
    // {
    //   _fdis = 30.0f;
    // }


    // float alpha = -std::tanh(_B*(vcur.norm()-_C));
    // // float alpha = -std::tanh(_B*(((_planeNormal*_planeNormal.transpose())*vcur).norm()-_C));
    // // float alpha = (1-std::tanh(10000*vcur.norm()));
    // float error = _targetForce-fabs(_filteredWrench(2));
    // float beta = -std::tanh(_B*vcur.norm());
    // // _fc += _A*(alpha*error+beta*fabs(_filteredWrench(2)));
    // _fc += _A*(alpha*error+beta*fabs(_filteredWrench(2)));
    // // _fc += _A*(alpha*error+beta*_targetForce);

    // float error = _targetForce-fabs(_filteredWrench(2));
    // float alpha = (1.0f-std::tanh(50*normalDistance));
    // float beta = (-std::tanh(50*normalDistance));
    // float alpha = -std::tanh(_B*(vn.norm()-_C));

  // _vdes = _twist.segment(0,3);

  // float ch= (1+std::tanh(40*((vcur).dot(_vdes)-0.01)))/2.0;
  // _vdes = _vdes*ch+(1-ch)*vcur;
  // std::cerr << "agreement: " << (vcur).dot(_vdes) << " gain: " << ch << std::endl;

  // KDL::Vector vec;

  // // Convert eigen matrix intp kdl
  // KDL::Rotation Rkdl(Re(0,0),Re(0,1),Re(0,2),
  //                    Re(1,0),Re(1,1),Re(1,2),
  //                    Re(2,0),Re(2,1),Re(2,2));  


  // float angle = Rkdl.GetRotAngle(vec);

  // // Compute angular velocity direction in desired frame
  // omega << vec.x(), vec.y(), vec.z();
  // omega *= angle;

  // Eigen::Matrix3f Rdes = Re*_wRb;
  // Eigen::Vector4f qf = rotationMatrixToQuaternion(Rdes);

  // _omegades.setConstant(0.0f);
  // _omegades = omegaTemp+0.3*_wRb*torque;
// 

  // _xp += -_planeNormal*0.01;

  // float ch = 0;
  // if(vcur.norm() < 1e-5f || _vdes.norm() < 1e-5f)
  // {
  //   ch = 1.0f;
  // }
  // else
  // {
  //   // ch = (1+std::tanh(10*(vcur).dot(_vdes)/(vcur.norm()*_vdes.norm())-0.1f))/2.0;
  //   ch = (1+std::tanh(5*(vcur).dot(_vdes)/(vcur.norm()*_vdes.norm())-0.5f))/2.0;
  // }


  // _contactForce.setConstant(0.0f);
  // float beta = (1.0f-cos(fabs(_filteredWrench(2))*M_PI/(_targetForce)))/2.0f;
  // _fc += std::tanh(1e-5f/vcur.norm())*0.1f*(beta*_targetForce-fabs(_filteredWrench(2)))
  // if(fabs(_filteredWrench(2)) > _targetForce)
  // {
  //   beta = 1.0f;
  // }

  // float alpha = -_A*std::tanh(_B*(vcur.norm()-_C));
  // _fc += alpha*(_targetForce-fabs(_filteredWrench(2)));

  // _vdes = (D*L*D.transpose())*((_xp-_pcur)+(Eigen::Matrix3f::Identity()-D.col(0)*D.col(0).transpose())*(xa-_pcur)+command*D.col(0));
  // if(normalDistance<1e-2f)
  // {
  //   if(!_firstClose)
  //   {
  //     std::cerr << "asasdasfdafasgfasgas" << std::endl;
  //     std::cerr << normalDistance << std::endl;
  //   }
  //   _taskAttractor << _pcur(0), _pcur(1), 0.22f;
  //   normalDistance = (_planeNormal*_planeNormal.transpose()*(_pcur-_taskAttractor)).norm();
  //   if(!_firstClose)
  //   {
  //     std::cerr << normalDistance << std::endl;
  //     _firstClose = true;
  //   }  
  // }
  // Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  // L(0,0) = 2.0f*normalDistance;
  // L(1,1) = 2.0f*normalDistance+0.1f*std::exp(-normalDistance);
  // L(2,2) = 2.0f*normalDistance+0.1f*std::exp(-normalDistance);


  // Eigen::Matrix3f D;
  // D.col(0) = -_planeNormal;
  // D.col(1) = (Eigen::Matrix3f::Identity()-_planeNormal*_planeNormal.transpose())*_desiredDir;

  // D.col(1) = D.col(1)/(D.col(1).norm());
  // D.col(2) = (D.col(0)).cross(D.col(1));
  
  // Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
  // L(0,0) = 2.0f*normalDistance;
  // L(1,1) = 2.0f*normalDistance+0.1f*std::exp(-normalDistance);
  // L(2,2) = 2.0f*normalDistance+0.1f*std::exp(-normalDistance);

  // _dir = -(_pcur-_taskAttractor);
  // _dir /= _dir.norm();

  // Eigen::Vector3f u = _dir.cross(D.col(1));
  // float c = _dir.dot(D.col(1));
  // float s = u.norm();
  // u/=s;

  // float theta = std::atan2(delta,normalDistance)*std::acos(c)/(M_PI/2.0f);

  // Eigen::Matrix3f K = getSkewSymmetricMatrix(u);
  // Eigen::Matrix3f R = Eigen::Matrix3f::Identity()+std::sin(theta)*K+(1.0f-cos(theta))*K*K;


  // _vdes = (D*L*D.transpose())*(R*_dir);


  // std::cerr << omega.transpose() << std::endl;


  // std::cerr << (angle*omega).transpose() << std::endl;
  // std::cerr << (acos(c)*k).transpose() << std::endl;

  // _omegades = acos(c)*k;

  // Eigen::Vector4f q;
  // q = _qdes;
  // Eigen:: Vector4f wq;
  // wq << 0.0f, _wRb.transpose()*omega;
  // Eigen::Vector4f dq = quaternionProduct(q,wq);
  // q += 0.5f*_dt*dq;
  // q /= q.norm();
  // _qdes = q;   
  // _omegades = omega;


  // std::cerr << D << std::endl;

  // std::cerr << L << std::endl;

  // std::cerr << "task attractor: " << std::endl;
  // std::cerr << _taskAttractor.transpose() << std::endl;

  // std::cerr << "exp: " << std::endl;

  // std::cerr << (_xp-_pcur).transpose() << std::endl;
  // std::cerr << "exa: " << std::endl;


  // std::cerr << _vdes.transpose() << std::endl;

  // std::cerr << _taskAttractor.transpose() << std::endl;

  // std::cerr << (R*_dir).transpose() << std::endl;

  // Eigen::Matrix3f Rdes = Re*_wRb;
  // _qdes = rotationMatrixToQuaternion(Rdes);

  // _omegades.setConstant(0.0f);

  // std::cerr << omegaTemp.transpose() << " : " << _twist.segment(3,3).transpose() << std::endl;
  // _qdes << w,x,y,z;
  // std::cerr << _taskAttractor.transpose() << std::endl;
  // std::cerr << "d: " << normalDistance << " vdes: " << _vdes.transpose() << " norm: " << _vdes.norm() << std::endl;
  // std::cerr << "d: " << normalDistance << " vcur: " << vcur.norm() << std::endl;
  // std::cerr << "gamma: " << gamma << " F contact: " << _fdis << " vcur: " << vcur.norm() << std::endl;