#ifndef __CONTACT_TASK_SHARED_CONTROL_H__
#define __CONTACT_TASK_SHARED_CONTROL_H__

#include "FootMouseController.h"
#include "std_msgs/Float32.h"
#include "foot_interfaces/footMouseSharedControl_paramsConfig.h"
#include "visualization_msgs/Marker.h"

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

		float _alpha = 0.0f;

		float _arbitrationLimit;
		float _agreementWeight;
		float _attractorWeight;
		float _d1;
		float _d2;

		int _taskId;
		ros::Publisher _pubMarker;
		ros::Publisher _pubtaskAttractor;
		ros::Publisher _pubArbitration;
		
		visualization_msgs::Marker _msgMarker;
		geometry_msgs::PointStamped _msgTaskAttractor;

		KDL::Rotation _Rplane;

	public:

		// Class constructor
		ContactTaskSharedControl(ros::NodeHandle &n, double frequency);

	private:
		
		void computeCommand();

		void computeAutonomy();

		void computeTaskVelocity();

		void primaryTask();

		void processRightClickEvent(int value, bool newEvent);

		void processCursorEvent(float relX, float relY, bool newEvent);

		void publishData();

		void processABButtonEvent(int value, bool newEvent, int direction);

		void dynamicReconfigureCallback(foot_interfaces::footMouseController_paramsConfig &config, uint32_t level);

};


#endif
