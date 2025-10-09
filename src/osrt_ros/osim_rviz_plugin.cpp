#include <filesystem>
#include <osrt_ros/osim_rviz_plugin.h>
#include <osrt_ros/osim_to_urdf.h>
#include <rviz/display_context.h>
#include <rviz/robot/tf_link_updater.h>
#include <rviz/robot/robot_link.h>
#include <tinyxml2.h>


namespace osrt_rviz {

	OsimModelDisplay::OsimModelDisplay()
	{
		robot_description_property_->setDescription("Path for .osim model");
		robot_description_property_->setString("/srv/host_data/fk.osim");
		robot_description_property_->setName("Osim model path:");

		/*=
			new rviz::StringProperty("Robot Description", "robot_description",
					"Name of the parameter to search for to load the robot description.", this,
					&OsimModelDisplay::updateRobotDescription);
		*/
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

		std::string file_path, content; //
		try
		{
			if (!std::filesystem::exists(robot_description_property_->getStdString()))
			{
					clear();
					setStatus(rviz::StatusProperty::Error, "OSIM",
							QString("File [%1] does not exist.")
							.arg(robot_description_property_->getString()));
					// try again in a second
					QTimer::singleShot(1000, this, &OsimModelDisplay::updateRobotDescription);
					return;
			}
			else
				file_path = robot_description_property_->getStdString();
		}
		catch (const ros::InvalidNameException& e)
		{
			clear();
			setStatus(rviz::StatusProperty::Error, "OSIM",
					QString("Invalid parameter name: %1.\n%2")
					.arg(robot_description_property_->getString(), e.what()));
			return;
		}

		/// we need to generate content which will be the actual urdf conversion!
		///
		tinyxml2::XMLDocument* conv_urdf = OsimToUrdf::create_model(file_path);
		tinyxml2::XMLPrinter printer;
		conv_urdf->Print(&printer);
		content = printer.CStr();


		if (content == robot_description_)
		{
			return;
		}

		robot_description_ = content;

		urdf::Model descr;




		if (!descr.initString(robot_description_))
		{
			clear();
			setStatus(rviz::StatusProperty::Error, "URDF", "Failed to parse Converted URDF model!");
			return;
		}

		setStatus(rviz::StatusProperty::Ok, "URDF", "Converted URDF parsed OK");
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

