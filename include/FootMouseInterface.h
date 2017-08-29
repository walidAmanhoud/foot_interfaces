#ifndef __FOOT_MOUSE_INTERFACE_H__
#define __FOOT_MOUSE_INTERFACE_H__

#include "ros/ros.h"
#include <linux/input.h>
#include <fcntl.h>
#include <foot_interfaces/FootMouseMsg.h>
#include <deque>
#include "foot_interfaces/footMouseInterface_paramsConfig.h"
#include <dynamic_reconfigure/server.h>


class FootMouseInterface 
{
	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Footmouse data publisher declaration
		ros::Publisher _pubFootMouseData;

		// Footmouse data
		foot_interfaces::FootMouseMsg _footMouseMessage;

		// Input device variables
		int _fd;									// File descriptor
		struct input_event _ie;		// Input event structure

		// State variables
		bool _synReceived;				// Monitor a SYN event received
		bool _relReceived;				// Monitor a REL event received

		float _filteredRelX;			// Filtered relative x motion
		float _filteredRelY;			// Filtered relative y motion

		// Moving average filter variables for cursor data
		std::deque<int> _winX; 		// Window for relative x motion
		std::deque<int> _winY; 		// Window for relative y motion

		// Foot mouse interface configuration variables
		bool _useMovingAverage; 	// Select filtering method for cursor data (moving average or 1D low pass filter) 
		float _alpha; 						// Gain for low pass filter
		int _windowSize;					// Window size for moving average

		// Dynamic reconfigure (server+callback)		
		dynamic_reconfigure::Server<foot_interfaces::footMouseInterface_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<foot_interfaces::footMouseInterface_paramsConfig>::CallbackType _dynRecCallback;

	public:

		// Class constructor
		FootMouseInterface(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init(std::string eventPath);

		// Run node main loop
		void run();

	private:

		// Read foot mouse data from input device
		void readFootMouse();

		// Publish data to topics
		void publishData();

		// Dynamic reconfigure callback
		void dynamicReconfigureCallback(foot_interfaces::footMouseInterface_paramsConfig &config, uint32_t level);
};


#endif
