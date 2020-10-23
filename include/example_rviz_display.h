/**
 *    Author: Dr. Ing. Ahmad Kamal Nasir
 *    Email: dringakn@gmail.com
 **/
#ifndef EXAMPLE_DISPLAY_H
#define EXAMPLE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <ros/network.h>
#include <ros/ros.h>
#endif

#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/combo_box.h>
#include <rviz/properties/editable_combo_box.h>
#include <rviz/properties/float_edit.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/status_list.h>
#include <std_msgs/String.h>

namespace example_rviz_display {
class ExampleRvizDisplay : public rviz::Display {
  Q_OBJECT
 public:
  ExampleRvizDisplay();
  virtual ~ExampleRvizDisplay();
  virtual void subscribe();
  virtual void unsubscribe();

 protected:
  virtual void onInitialize();
  virtual void reset();

 private Q_SLOTS:
  void updateCurrentState();
  void updateTopic();

 private:
  void processMessage(const std_msgs::String::ConstPtr& msg);
  rviz::StringProperty* data1;
  rviz::BoolProperty* booldata;
  rviz::EditableEnumProperty* enumdata;
  rviz::StatusProperty* statusdata;
  rviz::IntProperty* intdata;
  rviz::FloatProperty* floatdata;
  rviz::FloatEdit* floatedit;
  rviz::ComboBox* combodata;
  rviz::StatusList* statuslist;
  rviz::EditableComboBox* editcombo;

  rviz::RosTopicProperty* topic;
  ros::NodeHandle nh;
  ros::Subscriber sub;
};

}  // namespace example_rviz_display

#endif  // EXAMPLE_DISPLAY_H