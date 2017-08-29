#ifndef __FOOT_MOUSE_SHARED_CONTROL_H__
#define __FOOT_MOUSE_SHARED_CONTROL_H__

#include "FootMouseController.h"

class FootMouseSharedControl: public FootMouseController 
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
		float _d1 = 0.0f;
		float _d2 = 0.2f;

	public:

		// Class constructor
		FootMouseSharedControl(ros::NodeHandle &n, double frequency);

	private:
		
		void computeCommand();

		void computeAutonomy();

		void computeTaskVelocity();

		void processRightClickEvent(int value, bool newEvent);

		void processCursorEvent(float relX, float relY, bool newEvent);




};


#endif
