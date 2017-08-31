#ifndef __FOOT_MOUSE_SHARED_CONTROL_H__
#define __FOOT_MOUSE_SHARED_CONTROL_H__

#include "FootMouseController.h"

#define NB_TASKS 1

class FootMouseSharedControl: public FootMouseController 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		Eigen::Vector3f _vuser;
		Eigen::Vector3f _vtask[NB_TASKS];
		Eigen::Vector3f _taskAttractor[NB_TASKS];

		float _alpha = 0.0f;
		float _d1 = 0.0f;
		float _d2 = 0.2f;

		int _taskId;
		ros::Publisher _pubtaskAttractor[NB_TASKS];

		geometry_msgs::PointStamped _msgTaskAttractor[NB_TASKS];



	public:

		// Class constructor
		FootMouseSharedControl(ros::NodeHandle &n, double frequency);

	private:
		
		void computeCommand();

		void computeAutonomy();

		void computeTaskVelocity();

		void processRightClickEvent(int value, bool newEvent);

		void processCursorEvent(float relX, float relY, bool newEvent);

		void publishData();




};


#endif
