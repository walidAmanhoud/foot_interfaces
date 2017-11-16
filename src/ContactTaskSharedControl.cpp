#include "ContactTaskSharedControl.h"

// #include <tf/transform_datatypes.h>


ContactTaskSharedControl::ContactTaskSharedControl(ros::NodeHandle &n, double frequency): FootMouseController(n,frequency),
  _n(n),
  _loopRate(frequency),
  _dt(1 / frequency)
{
  _vuser.setConstant(0.0f);
  _vtask.setConstant(0.0f);
  _efWrench.setConstant(0.0f);
  _wrench.setConstant(0.0f);
  _wrenchBias.setConstant(0.0f);
  _filteredWrench.setConstant(0.0f);
  _twist.setConstant(0.0f);
  _gravity << 0.0f, 0.0f, -9.80665f;
  _loadOffset << 0.0f,0.0f,0.046f;

  _rinit << 0.0f, 0.0f, 0.115f;
  _r = _rinit;
  _Lr.setConstant(0.0f);
  _cr.setConstant(0.0f);
  _Br = 1.0f;
  _Gr = 0.2f;
  _nc.setConstant(0.0f);
  _vc.setConstant(0.0f);
  _Ln.setConstant(0.0f);
  _Bn = 0.2f;
  _Gn = 1.0f;


  // _kp = 0.01f;
  // _ki = 0.05f;
  // _targetForce = 10.0f;

  _taskId = 0;

  // _taskAttractor << -0.34f, 0.3f, 0.35f;
  _taskAttractor << -0.54f, 0.0f, 0.25f;

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

  _firstTask = false;
  _firstWrenchReceived = false;
  _wrenchBiasOK = false;

  _ft = 4.0f;

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

  _subEndEffectorWrench = _n.subscribe("lwr/ee_ft",1,&ContactTaskSharedControl::updateEndEffectorWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subForceTorqueSensor = _n.subscribe("/ft_sensor/netft_data", 1, &ContactTaskSharedControl::updateMeasuredWrench, this, ros::TransportHints().reliable().tcpNoDelay());
  _subRealTwist = _n.subscribe("/lwr/ee_vel", 1, &ContactTaskSharedControl::updateMeasuredTwist, this, ros::TransportHints().reliable().tcpNoDelay());
  _pubtaskAttractor = _n.advertise<geometry_msgs::PointStamped>("ContactTaskSharedControl/taskAttractor", 1);
  _pubArbitration = _n.advertise<std_msgs::Float32>("ContactTaskSharedControl/arbitration", 1);
  _pubForceNorm = _n.advertise<std_msgs::Float32>("ContactTaskSharedControl/forceNorm", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("ContactTaskSharedControl/plane", 1);
  _pubFilteredWrench = _n.advertise<geometry_msgs::WrenchStamped>("ContactTaskSharedControl/filteredWrench", 1);

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
    // case foot_interfaces::FootMouseMsg::FM_RIGHT_CLICK:
    // {
    //   processABButtonEvent(buttonState,newEvent,1.0f);
    //   // processRightClickEvent(buttonState,newEvent);
    //   break;
    // }
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

  // computeAutonomy();

  // std::cerr << "alpha:" << _alpha << std::endl;
  // std::cerr << "vuser: " << _vuser.transpose() << std::endl;

  //std::cerr << "vtask: "<<  _vtask.transpose() << std::endl;
  //std::cerr << "omegades : "<<  _omegades.transpose() << std::endl;

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

  
  Eigen::Vector4f temp = _qdes;
  if(_filteredWrench.segment(0,3).norm() > _contactForceThreshold && _wrenchBiasOK)
  {
    primaryTask(true);
    _qdes = temp;
    //std::cerr <<"a" << std::endl;
    // primaryTask();
    // contactPointEstimation();
    // surfaceNormalEstimation();
    // std::cerr << "Contact point estimation: " << (_wRb*_r).transpose() << std::endl;
    // std::cerr << "Normal surface estimation: " << (_nc).transpose() << std::endl;
    // std::cerr << "Force normal estimation: " << (_wRb*_filteredWrench.segment(0,3)/_filteredWrench.segment(0,3).norm()).transpose() << std::endl;
    forceTask();
  }
  else
  {
    _forceNorm = _filteredWrench.segment(0,3).norm();
    primaryTask(false);
    _firstContactEstimation = false;
    _firstSurfaceEstimation = false;
    _cr.setConstant(0.0f);
    _Lr.setConstant(0.0f);
    _Ln.setConstant(0.0f);
    // _ft = _contactForceThreshold;
    if(_pidInteg < 0.0f)
    {
      
      _pidInteg =0.0f;
    }
    // attractionTask();
    _firstContact = false;
    _targetReached = false;
  }
}

void ContactTaskSharedControl::forceTask()
{
  bool useWorldFrame = true;
  bool updateQuaternion = false;
  float stiffnessGain = 2.0f;
  float dampingGain = 0.0f;

  // Choose surface normal estimation
  // Eigen::Vector3f force = _wRb.transpose()*_efWrench.segment(0,3);
  // Eigen::Vector3f torque = _wRb.transpose()*_efWrench.segment(3,3);
  Eigen::Vector3f force = _filteredWrench.segment(0,3);
  Eigen::Vector3f torque = _filteredWrench.segment(3,3);
  Eigen::Vector3f normalDirection = force/force.norm();

  Eigen::Matrix3f orthogonalProjector = Eigen::Matrix3f::Identity()-_wRb.col(2)*_wRb.col(2).transpose();
  Eigen::Vector3f frictionComponent;
  Eigen::Vector3f vb = _vuser(1)*_wRb.col(1);
  Eigen::Vector3f tangentialForce;
  Eigen::Vector3f vmes = _twist.segment(0,3);

  Eigen::Vector3f v = orthogonalProjector*vmes;
  // if(_twist.segment(0,3).norm()>1e-3f)
  // {
  // std::cerr << vb.norm() << " " << torque.norm() << std::endl;
    
  // }
  // if(v.norm()>1e-3f)// || (force.norm() > _targetForce && _twist.segment(0,3).norm() > 1e-3f))
  if(vb.norm()>1e-3f)// || (force.norm() > _targetForce && _twist.segment(0,3).norm() > 1e-3f))
  {
    // frictionComponent = force.dot(vb/vb.norm())*vb/vb.norm(); 
    frictionComponent = orthogonalProjector*force;
  }
  else
  {
    frictionComponent.setConstant(0.0f);
  }

  force -= frictionComponent;
  normalDirection = force/force.norm();

  // _forceNorm = force.norm();


  // std::cerr << "Friction: " << frictionComponent.transpose() << std::endl;

  // Eigen::Vector3f normalDirection = _wRb.transpose()*_nc;
  // Eigen::Vector3f zw;
  // zw << 0.0f,0.0f,1.0f;
  // Eigen::Vector3f normalDirection = _wRb.transpose()*zw;

  Eigen::Vector3f endEffectorDirection;
  endEffectorDirection << 0.0f,0.0f,-1.0f;


  tangentialForce = endEffectorDirection*endEffectorDirection.transpose()*force;
  _forceNorm = tangentialForce.norm();
  // Compute rotation matrix to align end effector direction with surface normal direction
  // Use rodrigues' rotation formula
  Eigen::Vector3f k;
  k = endEffectorDirection.cross(normalDirection);
  float s = k.norm();
  k /= s;

  Eigen::Matrix3f K;
  K << 0.0f, -k(2), k(1),
       k(2), 0.0f, -k(0),
       -k(1), k(0), 0.0f;

  float c = endEffectorDirection.transpose()*normalDirection;

  Eigen::Matrix3f Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
  

  Eigen::Matrix3f R;

  // Express desired rotation error into world frame if wanted
  if(useWorldFrame)
  {
    R = _wRb*Re*_wRb.transpose();
  }
  else
  {
    R = Re;
  }

  // Convert eigen matrix intp kdl
  KDL::Rotation Rkdl(R(0,0),R(0,1),R(0,2),
                     R(1,0),R(1,1),R(1,2),
                     R(2,0),R(2,1),R(2,2));  

  // Get axis angle representation
  KDL::Vector vec;
  float angle = Rkdl.GetRotAngle(vec);

  Eigen::Vector3f omega, damping;
  // Compute angular velocity direction in desired frame
  omega << vec.x(), vec.y(), vec.z();
  omega *= angle;

  // Compute damping in world frame
  damping = -fabs(std::acos(c))*_twist.segment(3,3);
  // std::cerr << "angle: " << fabs(std::acos(c)) << " omega: "<< _twist.segment(3,3).norm() << std::endl;

  if(fabs(angle)< M_PI/3.0f)
  {
    if(updateQuaternion)
    {  
      Eigen::Vector4f q;
      q = _qcur;
      Eigen:: Vector4f wq;

      if(useWorldFrame)
      {
        wq << 0, _wRb.transpose()*(stiffnessGain*omega + dampingGain*damping)+_d2*torque;
      }
      else
      {
        wq << 0, stiffnessGain*omega+_d2*torque + _wRb.transpose()*dampingGain*damping;
      }

      Eigen::Vector4f dq = quaternionProduct(q,wq);
      q += 0.5f*_dt*dq;
      q /= q.norm();
      _qdes = q;   
    }
    else
    {
      _qdes = _qcur;
    }

    // Compute final omegades in world frame
    if(useWorldFrame)
    {
      _omegades = stiffnessGain*omega + dampingGain*damping+_d2*_wRb*torque;
    }
    else
    {
      _omegades = _wRb*(stiffnessGain*omega+_d2*torque) + dampingGain*damping;
    }

    // if(_usePid && fabs(std::acos(c))<0.3f)
    if(_usePid)
    {
      if(!_firstContact)
      {
        _ft = _forceNorm;
        _firstContact = true;
      }
      else
      {
        _ft += _dt*(_targetForce-_ft);
      }
      // _pidError = (_targetForce-force.norm());
      _pidError = (_targetForce-_forceNorm);
      _pidInteg += _dt*_pidError;
      _up = _kp*_pidError;
      _ui = _ki*_pidInteg;
      _ud = _kd*(_pidError-_pidLastError)/_dt;
      _pidLastError = _pidError;

      if(_ui < -_pidLimit)
      {
        _ui = -_pidLimit;
      }
      else if(_ui > _pidLimit)
      {
        _ui = _pidLimit;
      }

      float command = _up+_ui+_ud;
      if(command<-_pidLimit)
      {
        command = -_pidLimit;
      }
      else if(command>_pidLimit)
      {
        command = _pidLimit;
      }
      std::cerr << "up " << _up << " ui " << _ui << " ud " << _ud <<  " error " << _pidError << std::endl;

      // Eigen::Vector3f vf = -command*_wRb*normalDirection;
      Eigen::Vector3f vf = command*_wRb.col(2);
      Eigen::Vector3f vu = _vtask;

      float d, c, ct, ch;


      if(fabs(_pidError)<0.5)
      {
        _targetReached = true;
      }
      // d = fabs(_pidError);
      // ct = _attractorWeight*std::exp(-d);


      ch = _agreementWeight*(vf).dot(vu);
      d = _forceNorm;
      if(d <0.0f)
      {
        ct = 0.0f;
      }
      else if(d > _targetForce)
      {
        ct = 1.0f;
      }
      else
      {
        ct = (-cos(d*M_PI/(_targetForce))+1)/2.0f;
      }
      ct = _attractorWeight*ct;


      c = ch+ct;
      _alpha = c;

      if(_alpha>_arbitrationLimit)
      {
        _alpha = _arbitrationLimit;
      }
      else if(_alpha < 0.0f)
      {
        _alpha = 0.0f;
      }

      std::cerr << "alpha: " << _alpha << " c: " << c << " ch: " << ch <<" ct: " << ct << std::endl;
      
      // if(_twist.segment(3,3).norm()<_d1)
      // {
        // _vtask += command*_wRb.col(2);  
        _vtask = (1-_alpha)*vu+_alpha*vf+_vuser(1)*_wRb.col(1); 
      // }
      // if(angle<0.3f)
      // {

      if (_vtask.norm() > 0.3f) 
      {
        _vtask = _vtask / _vtask.norm()*0.3f;
      }

      std::cerr << "vtask: " << _vtask.norm() << " vf: " << vf.norm() << " vu: " << vu.norm() << std::endl;
      // }

    }
    else
    {
      _pidInteg = 0.0f;
      _firstContact = false;
    }
  }
    
}

void ContactTaskSharedControl::contactPointEstimation(void)
{

  if(!_firstContactEstimation)
  {
    _firstContactEstimation = true;
    _r = _rinit;
  }
  Eigen::Matrix3f Sf;
  Sf = getSkewSymmetricMatrix(_filteredWrench.segment(0,3));
  _Lr += (-_Br*_Lr-Sf*Sf)*_dt;
  _Lr = 0.5f*(_Lr+_Lr.transpose());

  std::cerr << _Lr << std::endl;

  _cr += (-_Br*_cr+Sf*_filteredWrench.segment(3,3))*_dt;

  std::cerr << _cr << std::endl;
  _r += -_Gr*(_Lr*_r-_cr)*_dt;

}

void ContactTaskSharedControl::surfaceNormalEstimation(void)
{

  if(!_firstSurfaceEstimation)
  {
    _firstSurfaceEstimation = true;
    _nc = _filteredWrench.segment(0,3)/_filteredWrench.segment(0,3).norm();
    _nc = _wRb*_nc;
  }

  Eigen::Vector3f angularVel = _twist.segment(3,3);
  _vc = _twist.segment(0,3)+angularVel.cross(_wRb*_r);
  _Ln += (-_Bn*_Ln+_vc*_vc.transpose())*_dt;

  Eigen::Matrix3f P;
  P = Eigen::Matrix3f::Identity()-_nc*_nc.transpose();

  _nc += -_Gn*P*_Ln*_nc*_dt;
  _nc.normalize();

}

void ContactTaskSharedControl::attractionTask()
{

  _vtask = -(_pcur-_taskAttractor);
  _omegades.setConstant(0.0f);

  _qdes = _qcur;

  if (_vtask.norm() > 0.15f) 
  {
    _vtask = _vtask / _vtask.norm()*0.15f;
  }
}

// void ContactTaskSharedControl::primaryTask(bool contact)
// {

//   Eigen::Matrix3f wRb;
//   wRb << _Rcur.UnitX().x(), _Rcur.UnitY().x(), _Rcur.UnitZ().x(),
//          _Rcur.UnitX().y(), _Rcur.UnitY().y(), _Rcur.UnitZ().y(), 
//          _Rcur.UnitX().z(), _Rcur.UnitY().z(), _Rcur.UnitZ().z();


//   _direction= wRb.transpose()*(_taskAttractor-_pcur);
//   _distance = _direction.norm();


//   _direction /= _direction.norm();

//   Eigen::Matrix3f orthogonalProjector = Eigen::Matrix3f::Identity()-_direction*_direction.transpose();
//   Eigen::Matrix<float,3,6> L;
//   L.block(0,0,3,3) = -orthogonalProjector/_distance;
//   Eigen::Matrix3f S;

//   S << 0.0f, -_direction(2), _direction(1),
//        _direction(2), 0.0f, -_direction(0),
//        -_direction(1), _direction(0), 0.0f;
//   L.block(0,3,3,3) = S;

//   Eigen::Matrix<float,6,3> W;
//   W.setConstant(0.0f);
//   Eigen::JacobiSVD<Eigen::MatrixXf> svd;
//   Eigen::MatrixXf singularValues;
//   svd = Eigen::JacobiSVD<Eigen::MatrixXf>(L, Eigen::ComputeFullU | Eigen::ComputeFullV);
//   singularValues = svd.singularValues();
//   // Compute pseudo inverse jacobian matrix
//   float tolerance = 1e-6f*std::max(L.rows(),L.cols())*singularValues.array().abs().maxCoeff();

//   for(int m = 0; m < std::min(W.rows(),W.cols()); m++)
//   {
//     if(singularValues(m,0)>tolerance)
//     {
//       W(m,m) = 1.0f/singularValues(m,0);
//     }
//     else
//     {
//       W(m,m) = 0.0f;
//     }
//   }
//   Eigen::Matrix<float,6,3> Linv;
//   Linv = svd.matrixV()*W*svd.matrixU().adjoint();

//   Eigen::Vector3f sd;
//   sd << 0.0f,0.0f,1.0f;

//   Eigen::Matrix<float,6,1> v;
//   Eigen::Matrix<float,6,1> gains;
//   gains << _linearSpeedGain*Eigen::Vector3f::Ones(), _angularSpeedGain*Eigen::Vector3f::Ones();

//   v = gains.cwiseProduct(Linv*sd);


//   Eigen::Vector3f ex, ey;
//   ex << 1.0f, 0.0f, 0.0f;
//   ey << 0.0f, 1.0f, 0.0f;

//   ex = -wRb.transpose()*ex;
//   ey = -wRb.transpose()*ey;

//   Eigen::Matrix<float, 6,1> n1,n2,n3,n4;
//   n1.setConstant(0.0f);
//   n2.setConstant(0.0f);
//   n3.setConstant(0.0f);
//   n4.setConstant(0.0f);

//   n1.segment(0,3) = _direction;
//   n2.segment(3,3) = _direction;
//   n3.segment(0,3) = -S*ey;
//   n3.segment(3,3) = -orthogonalProjector*ey/_distance;
//   n4.segment(0,3) = S*ex;
//   n4.segment(3,3) = orthogonalProjector*ex/_distance;

//   v.setConstant(0.0f);
//   v+= _vuser(0)*n1;
//   if(!contact)
//   {
//     v+=_vuser(1)*n4;//+_vuser(0)*n3;
//   }

//   if (v.segment(0,3).norm() > 0.15f) 
//   {
//     v.segment(0,3) = v.segment(0,3) / v.segment(0,3).norm()*0.15f;
//   }

//   if (v.segment(3,3).norm() > _angularVelocityLimit) 
//   {
//     v.segment(3,3) = v.segment(3,3) / v.segment(3,3).norm()*_angularVelocityLimit;
//   }
//   // std::cerr << v.segment(0,3).norm() << std::endl;
//   // std::cerr << v.segment(3,3).norm() << std::endl;
//   if(!contact)
//   {
//     Eigen::Vector4f q;
//     q = _qdes;
//     Eigen:: Vector4f wq;
//     wq << 0, v.segment(3,3);
//     Eigen::Vector4f dq = quaternionProduct(q,wq);
//     q += 0.5f*_dt*dq;
//     q /= q.norm();
//     _qdes = q;
    
//   }
  
//   // std::cerr << dq.transpose() << std::endl;
//   _vtask = wRb*(v.segment(0,3));
//   // _vtask.setConstant(0.0f);
//   _omegades = wRb*(v.segment(3,3));

//   //std::cerr << _direction.transpose() << std::endl;
//   // _qdes = _qcur;

//   _pdes = _pcur;

// }

void ContactTaskSharedControl::primaryTask(bool contact)
{
    
  if(fabs(_vuser(0)) < 1e-3f)
  {
    _vtask = -_convergenceRate*(_pcur-_pdes);
    _vtask.setConstant(0.0f);
  }
  else
  {
    _Rdes = KDL::Rotation::Quaternion(_qdes(1),_qdes(2),_qdes(3),_qdes(0));
    KDL::Vector temp = _Rdes.UnitZ();
    Eigen::Vector3f zAxis;
    zAxis << temp.x(),temp.y(),temp.z();
    _vtask = _vuser(0)*zAxis;
    _pdes = _pcur;

  }
  _omegades.setConstant(0.0f);
  _omegades(0) = _vuser(1)*_angularVelocityLimit/_linearVelocityLimit;

  Eigen::Vector4f q;
  q = _qdes;
  Eigen:: Vector4f wq;
  wq << 0, _wRb.transpose()*_omegades;
  Eigen::Vector4f dq = quaternionProduct(q,wq);
  q += 0.5f*_dt*dq;
  q /= q.norm();
  _qdes = q;
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

void ContactTaskSharedControl::dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request bou. Updatig the parameters ...");

  _arbitrationLimit = config.arbitrationLimit;
  _agreementWeight = config.agreementWeight;
  _attractorWeight = config.attractorWeight;
  _d1 = config.d1;
  _d2 = config.d2;
  _linearSpeedGain = config.linearSpeedGain;
  _angularSpeedGain = config.angularSpeedGain;
  _filteredGain = config.filteredGain;
  _contactForceThreshold = config.contactForceThreshold;
  _usePid = config.usePid;
  _targetForce = config.targetForce;
  _pidLimit = config.pidLimit;
  _kp = config.kp;
  _ki = config.ki;
  _kd = config.kd;

  _convergenceRate = config.convergenceRate;
  _zVelocity = config.zVelocity;
  _linearVelocityLimit = config.linearVelocityLimit;
  _angularVelocityLimit = config.angularVelocityLimit;
  _modeThreeTranslation = config.modeThreeTranslation;


}

void ContactTaskSharedControl::updateEndEffectorWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  _efWrench(0) = msg->wrench.force.x;
  _efWrench(1) = msg->wrench.force.y;
  _efWrench(2) = msg->wrench.force.z;
  _efWrench(3) = msg->wrench.torque.x;
  _efWrench(4) = msg->wrench.torque.y;
  _efWrench(5) = msg->wrench.torque.z;

}


void ContactTaskSharedControl::updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg)
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
  // if(!_firstWrenchReceived)
  // {
  //   _filteredWrench = _wrench;
  //   _firstWrenchReceived = true;
  // }
  // else
  // {
  //   _filteredWrench = _filteredGain*_filteredWrench+(1.0f-_filteredGain)*_wrench;
  // }
}

void ContactTaskSharedControl::updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  _twist(0) = msg->linear.x;
  _twist(1) = msg->linear.y;
  _twist(2) = msg->linear.z;
  _twist(3) = msg->angular.x;
  _twist(4) = msg->angular.y;
  _twist(5) = msg->angular.z;

  // if(!_firstWrenchReceived)
  // {
  //   _filteredWrench = _wrench;
  //   _firstWrenchReceived = true;
  // }
  // else
  // {
  //   _filteredWrench = _filteredGain*_filteredWrench+(1.0f-_filteredGain)*_wrench;
  // }
}


Eigen::Matrix3f ContactTaskSharedControl::getSkewSymmetricMatrix(Eigen::Vector3f input)
{
  Eigen::Matrix3f output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}