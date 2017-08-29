#ifndef __FOOT_MOUSE_CONTROLLER_H__
#define __FOOT_MOUSE_CONTROLLER_H__

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

// Max cursor value of the foot mouse
#define MAX_XY_REL 127
#define MAX_FRAME 200


class FootMouseController 
{

	protected:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Subscribers and publishers declaration
		ros::Subscriber _subRealPose;						// Subscribe to robot current pose
		ros::Subscriber _subFootMouse;          // Subscribe to foot mouse data
		ros::Publisher _pubDesiredPose;         // Publish desired pose
		ros::Publisher _pubDesiredTwist;				// Publish desired twist
		ros::Publisher _pubDesiredOrientation;  // Publish desired orientation


		// Subsciber and publisher messages declaration
		foot_interfaces::FootMouseMsg _msgFootMouse;
		geometry_msgs::Pose _msgRealPose;
		geometry_msgs::Pose _msgDesiredPose;
		geometry_msgs::Quaternion _msgDesiredOrientation;
		geometry_msgs::Twist _msgDesiredTwist;


		// State variables
		bool _firstRealPoseReceived;	// Monitor the first robot pose update
		bool _firstEventReceived;			// Monitor the first foot mouse event update
		uint8_t _lastEvent;						// Last foot mouse event
		bool _firstButton;						// Monitor first button A/B pressed
		bool _buttonPressed;					// Monitor button A/B pressed

		// Foot mouse control variables
		KDL::Rotation _Rcur;					// Current end effector orientation
		KDL::Rotation _Rdes;					// Desired end effector orientation
		Eigen::Vector3f _pcur;				// Current position [m] (3x1)
		Eigen::Vector3f _pdes;				// Desired position [m] (3x1)
		Eigen::Vector3f _vdes;				// Desired velocity [m/s] (3x1)
		Eigen::Vector3f _omegades;		// Desired angular velocity [rad/s] (3x1)
		Eigen::Vector4f _qcur;				// Current end effector quaternion (4x1)
		Eigen::Vector4f _qdes;				// Desired end effector quaternion (4x1)
		Eigen::Vector3f _attractorPosition;				// Current position [m] (3x1)


		// Foot mouse controller configuration variables
		float _convergenceRate;				// Convergence rate of the DS
		float _zVelocity;							// Velocity along Z axis [m/s]
		float _linearVelocityLimit;		// Linear velocity limit [m/s]
		float _angularVelocityLimit;  // Angular velocity limit [rad/s]
		bool _modeThreeTranslation;		// Control mode (true: X,Y,Z translations / false: roll,pitch rotations + insertion,withdraw)

		// Dynamic reconfigure (server+callback)
		dynamic_reconfigure::Server<foot_interfaces::footMouseController_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<foot_interfaces::footMouseController_paramsConfig>::CallbackType _dynRecCallback;

		// Class variables
		std::mutex _mutex;

		
	public:

		// Class constructor
		FootMouseController(ros::NodeHandle &n, double frequency);

		// Initialize node
		bool init();

		// Initialize ROS specific elements
		bool initROS();

		// Run node main loop
		void run();

	protected:

		// Compute quaternion product
		Eigen::Vector4f quaternionProduct(Eigen::Vector4f q1, Eigen::Vector4f q2);
	
	private:

		// Callback to update real robot pose
		void updateRealPose(const geometry_msgs::Pose::ConstPtr& msg);

		// Callback to update foot mouse data
		void updateFootMouseData(const foot_interfaces::FootMouseMsg::ConstPtr& msg);

		// Publish data to topics
		void publishData();

		// Process button A/B event of the foot mouse
		virtual void processABButtonEvent(int value, bool newEvent, int direction);

		// Process button wheel event of the foot mouse
		void processWheelEvent(int value, bool newEvent);

		// Process cursor event of the foot mouse
		virtual void processCursorEvent(float relX, float relY, bool newEvent);

		virtual void processRightClickEvent(int value, bool newEvent);

		// Compute command to be send to robot controller
		virtual void computeCommand();

		// Dynamic reconfigure callback
		void dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level);
    

    ros::Publisher _pubAttractorPosition;
    ros::Publisher _pubDesiredPath;
    bool _start;
    bool _rightClick;

    pthread_t _thread;
    
    static void* startPathPublishingLoop(void* ptr);
    void pathPublishingLoop();
    void publishFuturePath();
    nav_msgs::Path _msgDesiredPath;
    int _count;

};


#endif
