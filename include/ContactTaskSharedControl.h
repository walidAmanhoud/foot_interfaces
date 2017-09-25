#ifndef __CONTACT_TASK_SHARED_CONTROL_H__
#define __CONTACT_TASK_SHARED_CONTROL_H__

#include "FootMouseController.h"
#include "std_msgs/Float32.h"
#include "foot_interfaces/footMouseSharedControl_paramsConfig.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"

#define NB_SAMPLES 50

class ContactTaskSharedControl: public FootMouseController 
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

		int _taskId;
		ros::Subscriber _subEndEffectorWrench;					// Subscribe to robot current pose
		ros::Subscriber _subForceTorqueSensor;				// Subscribe to robot current pose
		ros::Subscriber _subRealTwist;						// Subscribe to robot current twist
		ros::Publisher _pubMarker;
		ros::Publisher _pubtaskAttractor;
		ros::Publisher _pubArbitration;
		ros::Publisher _pubForceNorm;
		ros::Publisher _pubFilteredWrench;
		
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

		// Contact point estimation
		Eigen::Vector3f _r;
		Eigen::Vector3f _rinit;
		Eigen::Matrix3f _Lr;
		Eigen::Vector3f _cr;
		float _Br;
		float _Gr;
		// Surface normal estimation
		Eigen::Matrix3f _Ln;
		Eigen::Vector3f _nc;
		Eigen::Vector3f _vc;
		float _Bn;
		float _Gn;
		bool _firstSurfaceEstimation = false;
		bool _firstContactEstimation = false;

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

		// static ContactTaskSharedControl* me;

	public:

		// Class constructor
		ContactTaskSharedControl(ros::NodeHandle &n, double frequency);

	private:
		
		void computeCommand();

		void computeAutonomy();

		void computeTaskVelocity();

		void primaryTask(bool contact);

		void forceTask();

		void attractionTask();

		void processRightClickEvent(int value, bool newEvent);

		void processCursorEvent(float relX, float relY, bool newEvent);

		void publishData();

		void processABButtonEvent(int value, bool newEvent, int direction);

		void dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level);

		void updateEndEffectorWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

		void updateMeasuredWrench(const geometry_msgs::WrenchStamped::ConstPtr& msg);

		void updateMeasuredTwist(const geometry_msgs::Twist::ConstPtr& msg);

		Eigen::Matrix3f getSkewSymmetricMatrix(Eigen::Vector3f input);

		void contactPointEstimation(void);

		void surfaceNormalEstimation(void);



};


#endif
