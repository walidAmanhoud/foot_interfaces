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
#include "foot_interfaces/footMouseController_paramsConfig.h"

#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"

#define NB_SAMPLES 50
#define MAX_XY_REL 350
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
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation


		// Subsciber and publisher messages declaration
		foot_interfaces::FootMouseMsg _msgFootMouse;
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;
		
		visualization_msgs::Marker _msgMarker;
		geometry_msgs::PointStamped _msgTaskAttractor;
		geometry_msgs::WrenchStamped _msgFilteredWrench;
		
		bool _firstTask;
		bool _firstWrenchReceived;
		bool _wrenchBiasOK;

		KDL::Rotation _Rplane;

		float _linearSpeedGain;
		float _angularSpeedGain;
		float _filteredGain;
		int _wrenchCount = 0;

		float _loadMass = 0.132f;
		Eigen::Vector3f _loadOffset;
		Eigen::Vector3f _gravity;

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
    float _zVelocity;             // Velocity along Z axis [m/s]
    float _linearVelocityLimit;   // Linear velocity limit [m/s]
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


		static ContactSurfaceController* me;

	public:

		// Class constructor
		ContactSurfaceController(ros::NodeHandle &n, double frequency);

		bool init();

		void run();

	private:
		
		static void stopNode(int sig);
		
    void computeCommand();

    void processCursorEvent(float relX, float relY, bool newEvent);

    void processABButtonEvent(int value, bool newEvent, int direction);

    void processRightClickEvent(int value, bool newEvent);

    void publishData();

    void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

    void updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg);

    void updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    void updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg);

    // void dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level);

    Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);

    Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

    Eigen::Vector4f rotationMatrixToQuaternion(Eigen::Matrix3f R);

  	Eigen::Matrix3f quaternionToRotationMatrix(Eigen::Vector4f q);

		void quaternionToAxisAngle(Eigen::Vector4f q, Eigen::Vector3f &axis, float &angle);






};


#endif
