#include <osrt_ros/osim_to_urdf.h>
#include <rviz/robot/robot.h>
#include <rviz/display.h>

#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/default_plugin/robot_model_display.h>
//#include <tinyxml2.h>
//#include "osrt_rviz_plugin.moc"
#include <QObject>
#include <QTimer>
#include <rviz/display_context.h>

// i just copy pasted this but it isnt working 
namespace osrt_rviz
{
	//virtual void load() override
	//{
	//	tinyxml2::XMLDocument* modified_urdf = create_model("some_path"); //maybe i load this from a custom property right?

	//}
	class OsimModelDisplay : public rviz::RobotModelDisplay
	{
		public:
			OsimModelDisplay();
			~OsimModelDisplay() override;
  using Display::load;

private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateTfPrefix();
  void updateAlpha();
  void updateRobotDescription();

  protected:

  virtual void load() override;

	};
}
