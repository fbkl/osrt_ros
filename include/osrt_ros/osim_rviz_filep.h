
#include <rviz/properties/string_property.h>
#include <QObject>
#include <QTimer>
#include <QFileDialog>
#include <rviz/display_context.h>


namespace osrt_rviz
{
	class FileProperty : public rviz::StringProperty {
		Q_OBJECT
		public:
			FileProperty(const QString& name, const QString& default_value, const QString& description, Property* parent = nullptr, const char* changed_slot =  nullptr, QObject* receiver = nullptr): rviz::StringProperty(name, default_value, description, parent, changed_slot, receiver) {}

			QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option) override
			{
				QString path = QFileDialog::getOpenFileName(parent, "Select model", "/srv/host_data","*.osim");
				if (!path.isEmpty()) setValue(path);
				return nullptr;
			}

			virtual ~FileProperty();
	};
	class DirectoryProperty : public rviz::StringProperty {
		Q_OBJECT
		public:
			DirectoryProperty(const QString& name, const QString& default_value, const QString& description, Property* parent = nullptr, const char* changed_slot =  nullptr, QObject* receiver = nullptr): rviz::StringProperty(name, default_value, description, parent, changed_slot, receiver) {}

			QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option) override
			{
				QString path = QFileDialog::getExistingDirectory(parent, "Select geometry directory", "/srv/host_data", QFileDialog::ShowDirsOnly);
				if (!path.isEmpty()) setValue(path);
				return nullptr;
			}

			virtual ~DirectoryProperty();
	};
}
