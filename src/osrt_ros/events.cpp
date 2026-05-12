/**
 * @author      : osrruser1 <osrtusr1[at]docker>
 * @file        : events.cpp
 * @date     : Friday Mar 17, 2023 14:18:52 CET
 */
#include "osrt_ros/events.h"

void addEvent(std::string event_name, opensimrt_msgs::Event & e)
{
	ROS_DEBUG_STREAM("Added event" << event_name);
	e.header.stamp = ros::Time::now(); 
	e.name = event_name;
}

void addEvent(std::string event_name, opensimrt_msgs::CommonTimed & msg)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	msg.events.list.push_back(e);

}

void addEvent(std::string event_name, opensimrt_msgs::Events & ee)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	ee.list.push_back(e);

}
/*
opensimrt_msgs::Events addEvent(std::string event_name, const opensimrt_msgs::CommonTimedConstPtr & msg)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	const std::vector<opensimrt_msgs::Event> ee = msg->events.list;
	std::vector<opensimrt_msgs::Event> eee = ee;
	eee.push_back(e);
	opensimrt_msgs::Events allEvents;
	allEvents.list = eee;
	return allEvents;
}
opensimrt_msgs::Events addEvent(std::string event_name, const opensimrt_msgs::PosVelAccTimedConstPtr & msg)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	const std::vector<opensimrt_msgs::Event> ee = msg->events.list;
	std::vector<opensimrt_msgs::Event> eee = ee;
	eee.push_back(e);
	opensimrt_msgs::Events allEvents;
	allEvents.list = eee;
	return allEvents;
}
*/
//couldnt get the template to work, so here is another version...
opensimrt_msgs::Events combineEvents(const std::vector<opensimrt_msgs::Event> e1,const std::vector<opensimrt_msgs::Event> e2)
{
	std::vector<opensimrt_msgs::Event> ee1 = e1;
	std::vector<opensimrt_msgs::Event> ee2 = e2;
	ee1.insert(ee1.end(),ee2.begin(),ee2.end());
	opensimrt_msgs::Events combineEvents_;
	combineEvents_.list = ee1;
	return combineEvents_;

}


