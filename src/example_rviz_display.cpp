/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 *    Note: The header file should be in the same folder as cpp, otherwise,
 *          during runtime unknown symbol error occures.
 **/
#include "example_rviz_display.h"

// Important: the name should match, otherwise, runtime error occurs.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_rviz_display::ExampleRvizDisplay, rviz::Display)

namespace example_rviz_display {

ExampleRvizDisplay::ExampleRvizDisplay() {
  data1 = new rviz::StringProperty("Data1", "", "String Data1", this,
                                   SLOT(updateCurrentState()));
  booldata = new rviz::BoolProperty("Bool", false, "Bool Data", this,
                                    SLOT(updateCurrentState()));

  enumdata = new rviz::EditableEnumProperty("Enum", "", "EnumList", this,
                                            SLOT(updateCurrentState()));
  enumdata->addOptionStd("One");
  enumdata->addOptionStd("Two");
  enumdata->addOptionStd("Three");

  statusdata = new rviz::StatusProperty("Status", "Status Property",
                                        rviz::StatusLevel::Ok, this);

  intdata = new rviz::IntProperty("int", 1, "int data", this,
                                  SLOT(updateCurrentState()));
  intdata->setMin(-100);
  intdata->setMax(100);

  floatdata = new rviz::FloatProperty("float", 0, "float property", this,
                                      SLOT(updateCurrentState()));
  floatdata->setMin(-10);
  floatdata->setMax(10);

  statuslist = new rviz::StatusList("Status List", this);

  floatedit = new rviz::FloatEdit();
  combodata = new rviz::ComboBox();
  editcombo = new rviz::EditableComboBox();

  topic = new rviz::RosTopicProperty("Topic", "",
                                     "Select topic containing string data", "",
                                     this, SLOT(updateTopic()));

  QString message_type =
      QString::fromStdString(ros::message_traits::datatype<std_msgs::String>());
  topic->setMessageType(message_type);
  topic->setDescription(message_type + " topic to subscribe to.");
}

void ExampleRvizDisplay::updateTopic() {
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void ExampleRvizDisplay::subscribe() {
  data1->setValue(QString(""));
  booldata->setBool(false);
  sub =
      nh.subscribe<std_msgs::String>(topic->getTopic().toStdString(), 1,
                                     &ExampleRvizDisplay::processMessage, this);
}

void ExampleRvizDisplay::unsubscribe() {
  data1->setValue(QString(""));
  booldata->setBool(false);
  sub.shutdown();
}

void ExampleRvizDisplay::onInitialize() {
  data1->setValue(QString(""));
  booldata->setBool(false);
}

ExampleRvizDisplay::~ExampleRvizDisplay() {}

void ExampleRvizDisplay::reset() {
  data1->setValue(QString(""));
  booldata->setBool(false);
}

void ExampleRvizDisplay::updateCurrentState() {}

void ExampleRvizDisplay::processMessage(const std_msgs::String::ConstPtr& msg) {
  data1->setValue(QString(msg->data.c_str()));
  booldata->setBool(!booldata->getBool());
  intdata->setInt(intdata->getInt() + 1);
  std::cout << enumdata->getStdString() << std::endl;
}

}  // namespace example_rviz_display
