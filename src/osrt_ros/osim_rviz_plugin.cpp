#include <osrt_ros/osim_rviz_plugin.h>
#include <osrt_ros/osim_to_urdf.h>
#include <rviz/display_context.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/robot/robot_link.h>

namespace osrt_rviz {

	OsimModelDisplay::OsimModelDisplay()
	{
		visual_enabled_property_ =
			new Property("Visual Enabled", true, "Whether to display the visual representation of the robot.",
					this, &OsimModelDisplay::updateVisualVisible);

		collision_enabled_property_ =
			new Property("Collision Enabled", false,
					"Whether to display the collision representation of the robot.", this,
					&OsimModelDisplay::updateCollisionVisible);

		update_rate_property_ = new rviz::FloatProperty("Update Interval", 0,
				"Interval at which to update the links, in seconds. "
				"0 means to update every update cycle.",
				this);
		update_rate_property_->setMin(0);

		alpha_property_ = new rviz::FloatProperty("Alpha", 1, "Amount of transparency to apply to the links.", this,
				&OsimModelDisplay::updateAlpha);
		alpha_property_->setMin(0.0);
		alpha_property_->setMax(1.0);

		robot_description_property_ =
			new rviz::StringProperty("Robot Description", "robot_description",
					"Name of the parameter to search for to load the robot description.", this,
					&OsimModelDisplay::updateRobotDescription);

		tf_prefix_property_ = new rviz::StringProperty(
				"TF Prefix", "",
				"Robot Model normally assumes the link name is the same as the tf frame name. "
				" This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
				this, &OsimModelDisplay::updateTfPrefix);

	}
	OsimModelDisplay::~OsimModelDisplay()
	{


	}
	void OsimModelDisplay::updateAlpha()
	{
		robot_->setAlpha(alpha_property_->getFloat());
		context_->queueRender();
	}

	void OsimModelDisplay::updateRobotDescription()
	{
		if (isEnabled())
			load();
	}

	void OsimModelDisplay::updateVisualVisible()
	{
		robot_->setVisualVisible(visual_enabled_property_->getValue().toBool());
		context_->queueRender();
	}

	void OsimModelDisplay::updateCollisionVisible()
	{
		robot_->setCollisionVisible(collision_enabled_property_->getValue().toBool());
		context_->queueRender();
	}

	void OsimModelDisplay::updateTfPrefix()
	{
		clearStatuses();
		context_->queueRender();
	}

	void linkUpdaterStatusFunction(rviz::StatusProperty::Level level,
			const std::string& link_name,
			const std::string& text,
			OsimModelDisplay* display)
	{
		display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
	}


	void OsimModelDisplay::load()
	{
		clearStatuses();
		context_->queueRender();

		std::string content;
		try
		{
			if (!update_nh_.getParam(robot_description_property_->getStdString(), content))
			{
				std::string loc;
				if (update_nh_.searchParam(robot_description_property_->getStdString(), loc))
					update_nh_.getParam(loc, content);
				else
				{
					clear();
					setStatus(rviz::StatusProperty::Error, "URDF",
							QString("Parameter [%1] does not exist, and was not found by searchParam()")
							.arg(robot_description_property_->getString()));
					// try again in a second
					QTimer::singleShot(1000, this, &OsimModelDisplay::updateRobotDescription);
					return;
				}
			}
		}
		catch (const ros::InvalidNameException& e)
		{
			clear();
			setStatus(rviz::StatusProperty::Error, "URDF",
					QString("Invalid parameter name: %1.\n%2")
					.arg(robot_description_property_->getString(), e.what()));
			return;
		}

		if (content.empty())
		{
			clear();
			setStatus(rviz::StatusProperty::Error, "URDF", "URDF is empty");
			return;
		}

		if (content == robot_description_)
		{
			return;
		}

		robot_description_ = content;

		urdf::Model descr;
		if (!descr.initString(robot_description_))
		{
			clear();
			setStatus(rviz::StatusProperty::Error, "URDF", "Failed to parse URDF model");
			return;
		}

		setStatus(rviz::StatusProperty::Ok, "URDF", "URDF parsed OK");
		robot_->load(descr);
		std::stringstream ss;
		for (const auto& name_link_pair : robot_->getLinks())
		{
			const std::string& err = name_link_pair.second->getGeometryErrors();
			if (!err.empty())
				ss << "\n• for link '" << name_link_pair.first << "':\n" << err;
		}
		if (ss.tellp())
			setStatus(rviz::StatusProperty::Error, "URDF",
					QString("Errors loading geometries:").append(ss.str().c_str()));

		robot_->update(rviz::TFLinkUpdater(context_->getFrameManager(),
					boost::bind(linkUpdaterStatusFunction, boost::placeholders::_1,
						boost::placeholders::_2, boost::placeholders::_3, this),
					tf_prefix_property_->getStdString()));
	}





}
//plugin export macro
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(osrt_rviz::OsimModelDisplay, rviz::Display)

