/**
 * @author      : osrtuser <osrtusr1[at]docker>
 * @file        : events.h
 * @date     : Friday Mar 17, 2023 11:37:23 CET
 */

#ifndef EVENTS_H17032023

#define EVENTS_H17032023
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "opensimrt_msgs/Event.h"
#include "opensimrt_msgs/Events.h"
#include <ros/ros.h>
#include <vector>

void addEvent(std::string event_name, opensimrt_msgs::Event & e);

void addEvent(std::string event_name, opensimrt_msgs::CommonTimed & msg);

void addEvent(std::string event_name, opensimrt_msgs::Events & ee);
/********************************
 *
 * Combine events of two message pointers that have events. It's not working for dual and combined messages though.
 *
 *
 */ 
template <typename T>
opensimrt_msgs::Events addEvent(std::string event_name, T msg_t)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	const std::vector<opensimrt_msgs::Event> ee = msg_t->events.list;
	std::vector<opensimrt_msgs::Event> eee = ee;
	if (!msg_t->events.list.empty()) {
		e.header.seq = msg_t->events.list.back().header.seq +1;
	}
	eee.push_back(e);
	opensimrt_msgs::Events allEvents;
	allEvents.list = eee;
	return allEvents;
}

template <typename T, typename U>
opensimrt_msgs::Events combineEvents(T msg_t, U msg_u)
{
	const std::vector<opensimrt_msgs::Event> e1 = msg_t->events.list;
	std::vector<opensimrt_msgs::Event> ee1 = e1;
	const std::vector<opensimrt_msgs::Event> e2 = msg_u->events.list;
	std::vector<opensimrt_msgs::Event> ee2 = e2;
	ee1.insert(ee1.end(),ee2.begin(),ee2.end());
	opensimrt_msgs::Events combineEvents_;
	combineEvents_.list = ee1;
	return combineEvents_;

}

opensimrt_msgs::Events combineEvents(const std::vector<opensimrt_msgs::Event> e1,const std::vector<opensimrt_msgs::Event> e2);

//opensimrt_msgs::Events addEvent(std::string event_name, const opensimrt_msgs::CommonTimedConstPtr & msg);
//opensimrt_msgs::Events addEvent(std::string event_name, const opensimrt_msgs::PosVelAccTimedConstPtr & msg);
#endif /* end of include guard EVENTS_H */

