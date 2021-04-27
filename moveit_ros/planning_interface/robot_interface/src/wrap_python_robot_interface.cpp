/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <moveit/py_bindings_tools/gil_releaser.h>
#include <moveit_msgs/msg/robot_state.hpp>
// ####################################################################
// # old
// #include <visualization_msgs/MarkerArray.h>
// ####################################################################
// # new 425
#include "/opt/ros/foxy/share/visualization_msgs/msg/MarkerArray.msg"
// ####################################################################

#include <stdexcept>
#include <boost/python.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;
using moveit::py_bindings_tools::GILReleaser;

namespace moveit
{
class RobotInterfacePython : protected py_bindings_tools::ROScppInitializer
{
public:
  RobotInterfacePython(const std::string& robot_description, const std::string& ns = "")
    : py_bindings_tools::ROScppInitializer()
  {
    // #############################################################
    // old
    // robot_model_ = planning_interface::getSharedRobotModel(robot_description);
    // #############################################################
    // new 425 add node to match arguments size
    auto node = rclcpp::Node::make_shared("moveit_python_wrappers");
    robot_model_ = planning_interface::getSharedRobotModel(node, robot_description);
    // ##############################################################
    if (!robot_model_)
      throw std::runtime_error("RobotInterfacePython: invalid robot model");
    current_state_monitor_ =
        // ##############################################################################################
        // old
        // planning_interface::getSharedStateMonitor(robot_model_, planning_interface::getSharedTF(), ns);
        // ##############################################################################################
        // new
        planning_interface::getSharedStateMonitor(node, robot_model_, planning_interface::getSharedTF());
        // ##############################################################################################
  }

  const char* getRobotName() const
  {
    return robot_model_->getName().c_str();
  }

  boost::python::list getJointNames() const
  {
    return py_bindings_tools::listFromString(robot_model_->getJointModelNames());
  }

  boost::python::list getGroupJointNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return py_bindings_tools::listFromString(jmg->getJointModelNames());
    else
      return boost::python::list();
  }

  boost::python::list getGroupJointTips(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      std::vector<std::string> tips;
      jmg->getEndEffectorTips(tips);
      return py_bindings_tools::listFromString(tips);
    }
    else
      return boost::python::list();
  }

  boost::python::list getLinkNames() const
  {
    return py_bindings_tools::listFromString(robot_model_->getLinkModelNames());
  }

  boost::python::list getGroupLinkNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return py_bindings_tools::listFromString(jmg->getLinkModelNames());
    else
      return boost::python::list();
  }

  boost::python::list getGroupNames() const
  {
    return py_bindings_tools::listFromString(robot_model_->getJointModelGroupNames());
  }

  boost::python::list getJointLimits(const std::string& name) const
  {
    boost::python::list result;
    const moveit::core::JointModel* jm = robot_model_->getJointModel(name);
    if (jm)
    {
      const std::vector<moveit_msgs::msg::JointLimits>& lim = jm->getVariableBoundsMsg();
      for (const moveit_msgs::msg::JointLimits& joint_limit : lim)
      {
        boost::python::list l;
        l.append(joint_limit.min_position);
        l.append(joint_limit.max_position);
        result.append(l);
      }
    }
    return result;
  }

  const char* getPlanningFrame() const
  {
    return robot_model_->getModelFrame().c_str();
  }

  boost::python::list getLinkPose(const std::string& name)
  {
    boost::python::list l;
    if (!ensureCurrentState())
      return l;
    moveit::core::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const moveit::core::LinkModel* lm = state->getLinkModel(name);
    if (lm)
    {
      // getGlobalLinkTransform() returns a valid isometry by contract
      const Eigen::Isometry3d& t = state->getGlobalLinkTransform(lm);
      std::vector<double> v(7);
      v[0] = t.translation().x();
      v[1] = t.translation().y();
      v[2] = t.translation().z();
      Eigen::Quaterniond q(t.linear());
      v[3] = q.x();
      v[4] = q.y();
      v[5] = q.z();
      v[6] = q.w();
      l = py_bindings_tools::listFromDouble(v);
    }
    return l;
  }

  boost::python::list getDefaultStateNames(const std::string& group)
  {
    boost::python::list l;
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      for (auto& known_state : jmg->getDefaultStateNames())
      {
        l.append(known_state);
      }
    }
    return l;
  }

  boost::python::list getCurrentJointValues(const std::string& name)
  {
    boost::python::list l;
    if (!ensureCurrentState())
      return l;
    moveit::core::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const moveit::core::JointModel* jm = state->getJointModel(name);
    if (jm)
    {
      const double* pos = state->getJointPositions(jm);
      const unsigned int sz = jm->getVariableCount();
      for (unsigned int i = 0; i < sz; ++i)
        l.append(pos[i]);
    }

    return l;
  }

  boost::python::dict getJointValues(const std::string& group, const std::string& named_state)
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return boost::python::dict();
    std::map<std::string, double> values;
    jmg->getVariableDefaultPositions(named_state, values);
    return py_bindings_tools::dictFromType(values);
  }

  bool ensureCurrentState(double wait = 1.0)
  {
    if (!current_state_monitor_)
    {
      // ###############################################
      // old
      // ROS_ERROR("Unable to get current robot state");
      // ###############################################
      // new 425
      auto node = rclcpp::Node::make_shared("moveit_python_wrappers");
      RCLCPP_ERROR(node->get_logger(), "Unable to get current robot state");
      // ###############################################
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
    {
      GILReleaser gr;
      current_state_monitor_->startStateMonitor();
      if (!current_state_monitor_->waitForCompleteState(wait))
      {
        // #######################################################################################
        // old
        // ROS_WARN("Joint values for monitored state are requested but the full state is not known");
        // #######################################################################################
        // new 425
        auto node = rclcpp::Node::make_shared("moveit_python_wrappers");
        RCLCPP_WARN(node->get_logger(), "Joint values for monitored state are requested but the full state is not known");
      }
        // ########################################################################################
    }
    return true;
  }

  py_bindings_tools::ByteString getCurrentState()
  {
    if (!ensureCurrentState())
      return py_bindings_tools::ByteString("");
    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    moveit_msgs::msg::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(*s, msg);
    return py_bindings_tools::serializeMsg(msg);
  }

  boost::python::tuple getEndEffectorParentGroup(const std::string& group)
  {
    // name of the group that is parent to this end-effector group;
    // Second: the link this in the parent group that this group attaches to
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return boost::python::make_tuple("", "");
    std::pair<std::string, std::string> parent_group = jmg->getEndEffectorParentGroup();
    return boost::python::make_tuple(parent_group.first, parent_group.second);
  }

  py_bindings_tools::ByteString getRobotMarkersPythonDictList(boost::python::dict& values, boost::python::list& links)
  {
    moveit::core::RobotStatePtr state;
    if (ensureCurrentState())
    {
      state = current_state_monitor_->getCurrentState();
    }
    else
    {
      state.reset(new moveit::core::RobotState(robot_model_));
    }

    boost::python::list k = values.keys();
    int l = boost::python::len(k);
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(l);
    joint_state.position.resize(l);
    for (int i = 0; i < l; ++i)
    {
      joint_state.name[i] = boost::python::extract<std::string>(k[i]);
      joint_state.position[i] = boost::python::extract<double>(values[k[i]]);
    }
    state->setVariableValues(joint_state);
    visualization_msgs::MarkerArray msg;
    state->getRobotMarkers(msg, py_bindings_tools::stringFromList(links));

    return py_bindings_tools::serializeMsg(msg);
  }

  py_bindings_tools::ByteString getRobotMarkersPythonDict(boost::python::dict& values)
  {
    boost::python::list links = py_bindings_tools::listFromString(robot_model_->getLinkModelNames());
    return getRobotMarkersPythonDictList(values, links);
  }

  py_bindings_tools::ByteString getRobotMarkersFromMsg(const py_bindings_tools::ByteString& state_str)
  {
    moveit_msgs::msg::RobotState state_msg;
    moveit::core::RobotState state(robot_model_);
    py_bindings_tools::deserializeMsg(state_str, state_msg);
    moveit::core::robotStateMsgToRobotState(state_msg, state);

    visualization_msgs::MarkerArray msg;
    state.getRobotMarkers(msg, state.getRobotModel()->getLinkModelNames());

    return py_bindings_tools::serializeMsg(msg);
  }

  py_bindings_tools::ByteString getRobotMarkers()
  {
    if (!ensureCurrentState())
      return py_bindings_tools::ByteString();
    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    visualization_msgs::MarkerArray msg;
    s->getRobotMarkers(msg, s->getRobotModel()->getLinkModelNames());

    return py_bindings_tools::serializeMsg(msg);
  }

  py_bindings_tools::ByteString getRobotMarkersPythonList(const boost::python::list& links)
  {
    if (!ensureCurrentState())
      return py_bindings_tools::ByteString("");
    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    visualization_msgs::MarkerArray msg;
    s->getRobotMarkers(msg, py_bindings_tools::stringFromList(links));

    return py_bindings_tools::serializeMsg(msg);
  }

  py_bindings_tools::ByteString getRobotMarkersGroup(const std::string& group)
  {
    if (!ensureCurrentState())
      return py_bindings_tools::ByteString("");
    moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    visualization_msgs::MarkerArray msg;
    if (jmg)
    {
      s->getRobotMarkers(msg, jmg->getLinkModelNames());
    }

    return py_bindings_tools::serializeMsg(msg);
  }

  py_bindings_tools::ByteString getRobotMarkersGroupPythonDict(const std::string& group, boost::python::dict& values)
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return py_bindings_tools::ByteString("");
    boost::python::list links = py_bindings_tools::listFromString(jmg->getLinkModelNames());
    return getRobotMarkersPythonDictList(values, links);
  }

  boost::python::dict getCurrentVariableValues()
  {
    boost::python::dict d;

    if (!ensureCurrentState())
      return d;

    const std::map<std::string, double>& vars = current_state_monitor_->getCurrentStateValues();
    for (const std::pair<const std::string, double>& var : vars)
      d[var.first] = var.second;

    return d;
  }

  const char* getRobotRootLink() const
  {
    return robot_model_->getRootLinkName().c_str();
  }

  bool hasGroup(const std::string& group) const
  {
    return robot_model_->hasJointModelGroup(group);
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  //########################
  // # old not use in anywhere, commentout temporarily
  // https://answers.ros.org/question/295701/get-ros-2-nodehandle-in-case-of-composed-nodes/
  // https://qiita.com/wacatsuki/items/95e39fdc8b409e84498d
  // ros::NodeHandle nh_;
  // #######################
};
}  // namespace moveit

static void wrap_robot_interface()
{
  using namespace moveit;

  boost::python::class_<RobotInterfacePython> robot_class("RobotInterface", boost::python::init<std::string, boost::python::optional<std::string>>());

  robot_class.def("get_joint_names", &RobotInterfacePython::getJointNames);
  robot_class.def("get_group_joint_names", &RobotInterfacePython::getGroupJointNames);
  robot_class.def("get_group_default_states", &RobotInterfacePython::getDefaultStateNames);
  robot_class.def("get_group_joint_tips", &RobotInterfacePython::getGroupJointTips);
  robot_class.def("get_group_names", &RobotInterfacePython::getGroupNames);
  robot_class.def("get_link_names", &RobotInterfacePython::getLinkNames);
  robot_class.def("get_group_link_names", &RobotInterfacePython::getGroupLinkNames);
  robot_class.def("get_joint_limits", &RobotInterfacePython::getJointLimits);
  robot_class.def("get_link_pose", &RobotInterfacePython::getLinkPose);
  robot_class.def("get_planning_frame", &RobotInterfacePython::getPlanningFrame);
  robot_class.def("get_current_state", &RobotInterfacePython::getCurrentState);
  robot_class.def("get_current_variable_values", &RobotInterfacePython::getCurrentVariableValues);
  robot_class.def("get_current_joint_values", &RobotInterfacePython::getCurrentJointValues);
  robot_class.def("get_joint_values", &RobotInterfacePython::getJointValues);
  robot_class.def("get_robot_root_link", &RobotInterfacePython::getRobotRootLink);
  robot_class.def("has_group", &RobotInterfacePython::hasGroup);
  robot_class.def("get_robot_name", &RobotInterfacePython::getRobotName);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkers);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonList);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersFromMsg);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDictList);
  robot_class.def("get_robot_markers", &RobotInterfacePython::getRobotMarkersPythonDict);
  robot_class.def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroup);
  robot_class.def("get_group_markers", &RobotInterfacePython::getRobotMarkersGroupPythonDict);
  robot_class.def("get_parent_group", &RobotInterfacePython::getEndEffectorParentGroup);
}

BOOST_PYTHON_MODULE(_moveit_robot_interface)
{
  wrap_robot_interface();
}

/** @endcond */
