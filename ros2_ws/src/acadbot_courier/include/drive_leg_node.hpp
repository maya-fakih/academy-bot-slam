#pragma once
// drive_leg_node.hpp
// ---------------------------------------------------------------------------
// A BehaviorTree.CPP leaf node that drives to ONE named location via our
// existing Executor. This replaces the hand-written driveLeg()/onLegDone()
// C++ control flow with an equivalent BT leaf — the SAME Executor, SAME
// Nav2 action client underneath, just invoked from a tree instead of a
// callback chain.
//
// StatefulActionNode (not a plain SyncActionNode) because driving takes
// seconds/minutes — this node must return RUNNING immediately and let the
// tree keep ticking while Executor's async Nav2 goal is still in flight.
// ---------------------------------------------------------------------------
#include "behaviortree_cpp/bt_factory.h"
#include "executor.hpp"

class DriveLegNode : public BT::StatefulActionNode
{
public:
  DriveLegNode(const std::string & name, const BT::NodeConfig & config, Executor * executor)
  : BT::StatefulActionNode(name, config), executor_(executor) {}

  // Ports: what this leaf reads from the blackboard when placed in the XML.
  // "location_name/x/y/yaw" describe WHERE to drive; "leg" is just a label
  // ("pickup"/"dropoff") used for feedback, matching ExecuteDelivery's
  // current_leg field.
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("leg"),
      BT::InputPort<std::string>("location_name"),
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw"),
    };
  }

  // onStart(): called once when the tree ticks into this node. Kicks off
  // the async drive and returns RUNNING — mirrors driveLeg()'s job of
  // starting executor_->driveTo() and not blocking.
  BT::NodeStatus onStart() override
  {
    finished_ = false;
    success_ = false;

    getInput("location_name", location_name_);
    getInput("x", x_);
    getInput("y", y_);
    getInput("yaw", yaw_);

    executor_->driveTo(x_, y_, yaw_, "map",
      [](double) { /* feedback wiring happens at the dispatcher level, see note below */ },
      [this](bool success, const std::string & reason) {
        success_ = success;
        reason_ = reason;
        finished_ = true;
      });

    return BT::NodeStatus::RUNNING;
  }

  // onRunning(): called on every subsequent tick while still RUNNING.
  // Just checks the flag the async done-callback set — this is the BT
  // equivalent of onLegDone() checking success/failure.
  BT::NodeStatus onRunning() override
  {
    if (!finished_) return BT::NodeStatus::RUNNING;
    return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // onHalted(): called if a parent decorator/sequence aborts this node
  // early (e.g. a real requester cancel). Mirrors our cancel_requested path.
  void onHalted() override
  {
    executor_->cancel();
  }

private:
  Executor * executor_;
  std::string location_name_;
  double x_{0}, y_{0}, yaw_{0};
  bool finished_{false};
  bool success_{false};
  std::string reason_;
};