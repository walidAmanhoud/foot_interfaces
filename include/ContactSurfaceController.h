#ifndef __CONTACT_SURFACE_CONTROLLER_H__
#define __CONTACT_SURFACE_CONTROLLER_H__

#include <signal.h>
#include <mutex>
#include <pthread.h>
#include <vector>
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include <kdl/jntarray.hpp>
#include <kdl/framevel.hpp>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <dynamic_reconfigure/server.h>

#include "FootMouseInterface.h"
#include "foot_interfaces/FootMouseMsg.h"
#include "foot_interfaces/contactSurfaceController_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"

#define NB_SAMPLES 50
#define MAX_XY_REL 300
#define MAX_FRAME 200

class ContactSurfaceController 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		Eigen::Vector3f _vuser;
		Eigen::Vector3f _vtask;
		Eigen::Vector3f _taskAttractor;
		Eigen::Vector3f _direction;
		Eigen::Vector3f _contactForce;
		Eigen::Matrix<float,6,1> _efWrench;
		Eigen::Matrix<float,6,1> _wrench;
		Eigen::Matrix<float,6,1> _wrenchBias;
		Eigen::Matrix<float,6,1> _filteredWrench;
		Eigen::Matrix<float,6,1> _twist;
		float _forceNorm;

		float _alpha = 0.0f;
		float _distance = 0.0f;

		float _arbitrationLimit;
		float _agreementWeight;
		float _attractorWeight;
		float _d1;
		float _d2;


		// Subscribers and publishers declaration
		ros::Subscriber _subRealPose;						// Subscribe to robot current pose
		ros::Subscriber _subFootMouse;          // Subscribe to foot mouse data
		ros::Subscriber _subForceTorqueSensor;				// Subscribe to robot current pose
		ros::Subscriber _subRealTwist;						// Subscribe to robot current twist
		ros::Publisher _pubMarker;
		ros::Publisher _pubTaskAttractor;
		ros::Publisher _pubArbitration;
		ros::Publisher _pubForceNorm;
		ros::Publisher _pubFilteredWrench;
		ros::Publisher _pubDesiredPose;         // Publish desired pose
		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredWrench;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation

		ros::Subscriber _subOptitrackRobotBasisPose;
		ros::Subscriber _subOptitrackPlane1Pose;
		ros::Subscriber _subOptitrackPlane2Pose;
		ros::Subscriber _subOptitrackPlane3Pose;

		// Subsciber and publisher messages declaration
		foot_interfaces::FootMouseMsg _msgFootMouse;
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		geometry_msgs::Wrench _msgDesiredWrench;
		
		visualization_msgs::Marker _msgMarker;
		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		
		bool _firstTask;
		bool _firstWrenchReceived;
		bool _wrenchBiasOK;

		KDL::Rotation _Rplane;

		float _linearSpeedGain;
		float _angularSpeedGain;
		float _filteredForceGain;
		int _wrenchCount = 0;

		float _loadMass = 0.132f;
		Eigen::Vector3f _loadOffset;
		Eigen::Vector3f _gravity;
		float _toolOffset = 0.114f;
		// float _toolOffset = 0.0f;

		float _contactForceThreshold;

		// Pid force control
		float _kp;
		float _ki;
		float _kd;
		float _pidInteg = 0.0f;
		float _pidError = 0.0f;
		float _pidLastError = 0.0f;
		float _pidLimit;
		float _up;
		float _ui;
		float _ud;
		float _targetForce;
		float _ft;
		bool _usePid;
		bool _firstContact = false;
		bool _targetReached = false;


		// Foot mouse control variables
		KDL::Rotation _Rcur;					// Current end effector orientation
		KDL::Rotation _Rdes;					// Desired end effector orientation
		Eigen::Matrix3f _wRb;				// Current rotation matrix [m] (3x1)
		Eigen::Vector3f _pcur;				// Current position [m] (3x1)
		Eigen::Vector3f _pdes;				// Desired position [m] (3x1)
		Eigen::Vector3f _vdes;				// Desired velocity [m/s] (3x1)
		Eigen::Vector3f _omegades;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector4f _qcur;				// Current end effector quaternion (4x1)
		Eigen::Vector4f _qdes;				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _attractorPosition;				// Current position [m] (3x1)


        // Foot mouse controller configuration variables
    float _convergenceRate;       // Convergence rate of the DS
    float _xOffset;
    float _yOffset;
    float _zVelocity;             // Velocity along Z axis [m/s]
    float _linearVelocityLimit = 0.1f;   // Linear velocity limit [m/s]
    float _angularVelocityLimit;  // Angular velocity limit [rad/s]
    bool _modeThreeTranslation;   // Control mode (true: X,Y,Z translations / false: roll,pitch rotations + insertion,withdraw)

    		// State variables
		bool _firstRealPoseReceived;	// Monitor the first robot pose update
		bool _firstEventReceived;			// Monitor the first foot mouse event update
		uint8_t _lastEvent;						// Last foot mouse event
		bool _firstButton;						// Monitor first button A/B pressed
		bool _buttonPressed;					// Monitor button A/B pressed


		std::mutex _mutex;
  	int _count;
  	bool _stop = false;

    Eigen::Vector3f _planeNormal;
    Eigen::Vector3f _planeTangent;
    Eigen::Vector3f _p;
		Eigen::Vector3f _dir;
    float _angle;

    Eigen::Vector3f _desiredDir;
    bool _firstClose;

    Eigen::Vector3f _xp;
    float _fc;
    float _A;
    float _B;
    float _C;
    float _forceStiffnessRateGain;
    float _forceDampingRateGain;
    bool _polishing;

    float _fdis;

	Eigen::Vector3f _robotBasisPosition;
	Eigen::Vector3f _plane1Position;
	Eigen::Vector3f _plane2Position;
	Eigen::Vector3f _plane3Position;
	bool _firstRobotBasisPose;
	bool _firstPlane1Pose;
	bool _firstPlane2Pose;
	bool _firstPlane3Pose;

	Eigen::Vector3f _p1;
	Eigen::Vector3f _p2;
	Eigen::Vector3f _p3;

	float _xf;
	bool _controlForce;
	bool _splitForceFromMotion;

	bool _firstControl =  false;

	bool _linear;
	float _userVelocityLimit;

	static ContactSurfaceController* me;

			// Dynamic reconfigure (server+callback)
	dynamic_reconfigure::Server<foot_interfaces::contactSurfaceController_paramsConfig> _dynRecServer;
	dynamic_reconfigure::Server<foot_interfaces::contactSurfaceController_paramsConfig>::CallbackType _dynRecCallback;


	public:

		// Class constructor
		ContactSurfaceController(ros::NodeHandle &n, double frequency);

		bool init();

		void run();

	private:
		
		static void stopNode(int sig);
		
    void computeCommand();
    void autonomousControl();
    void autonomousControl2();

    void processCursorEvent(float relX, float relY, bool newEvent);

    void processABButtonEvent(int value, bool newEvent, int direction);

    void processRightClickEvent(int value, bool newEvent);

    void publishData();

    void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg);

    void updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg);

    void dynamicReconfigureCallback(foot_interfaces::contactSurfaceController_paramsConfig &config, uint32_t level);

    Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

    Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

    Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

  	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

	void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);

  	Eigen::Vector4f slerpQuaternion(Eigen::Vector4f q1, Eigen::Vector4f q2, float t);

	void updateRobotBasisPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void updatePlane1Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void updatePlane2Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void updatePlane3Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	Eigen::Vector3f getDesiredVelocity(Eigen::Vector3f position, Eigen::Vector3f attractor);




};


#endif
