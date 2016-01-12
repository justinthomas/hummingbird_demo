#include <mav_manager/mav_manager_services.h>

#include <trajectory/trajectory.h>
#include <string>

float deadband(float val, float deadband) {
  if (val > deadband)
    return val - deadband;
  else if (val < -deadband)
    return val + deadband;
  else
    return 0.0;
}

class DemoServices : public MAVManagerServices
{
  public:

    Trajectory traj;

    // Subscribers
    ros::Subscriber odom_sub_;

    bool traj_active_;

    bool useRadioSimulator_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      const std::string tracker_str("std_trackers/RadioTrackerSimulator");
      res.success = mav->transition(tracker_str);
      res.message = "Transition to " + tracker_str;
      if (res.success)
      {
        res.message += " succeeded.";
        last_cb_ = "useRadioSimulator";
      }
      else
        res.message += " failed";

      return true;
    }
    bool useRadioForVelocity_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      const std::string tracker_str("std_trackers/RadioTrackerVelocity");
      res.success = mav->transition(tracker_str);
      res.message = "Transition to " + tracker_str;
      if (res.success)
      {
        res.message += " succeeded";
        last_cb_ = "useRadioForVelocity";
      }
      else
        res.message += " failed";

      return true;
    }
    bool prepTraj_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      // Read the file
      if (traj.LoadTrajectory())
      {
        traj.setOffsets(req.goal[0], req.goal[1], req.goal[2], mav->yaw());
        ROS_INFO("Trajectory Offsets set to %2.3f, %2.3f, %2.3f, %2.3f", req.goal[0], req.goal[1], req.goal[2], mav->yaw());
        traj.set_start_time();
        quadrotor_msgs::PositionCommand goal;
        traj.UpdateGoal(goal);

        res.success = mav->goTo(goal.position.x, goal.position.y, goal.position.z, goal.yaw);

        last_cb_ = "prepTraj";
      }
      else
      {
        res.success = false;
        res.message = "Cannot prep traj. Trajectory not loaded. Error Code: " + std::to_string(traj.get_error_code());
        ROS_WARN("%s", res.message.c_str());
      }
      return true;
    }
    bool executeTrajectory_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      // Make sure the trajectory is loaded
      if (traj.isLoaded())
      {
        res.success = false;
        res.message = "Trajectory not loaded. Cannot start trajectory.";
        ROS_WARN("%s", res.message.c_str());
        return true;
      }

      // Check the velocity error
      float vel = (mav->vel()).norm();
      if (vel > 0.1)
      {
        res.success = false;
        res.message = "Velocity too large to start trajectory";
        ROS_WARN("%s", res.message.c_str());
        return true;
      }

      // Check the position error
      traj.set_start_time();
      quadrotor_msgs::PositionCommand goal;
      traj.UpdateGoal(goal);
      MAVManager::Vec3 des_pos(goal.position.x, goal.position.y, goal.position.z);

      float e = (mav->pos() - des_pos).norm();
      if (e > 0.15)
      {
        ROS_WARN("Position error too large to start trajectory");
        res.success = false;
        res.message = "Position error too large to start trajectory";
        ROS_WARN("%s", res.message.c_str());
        return true;
      }

      if(mav->setPositionCommand(goal))
      {
        traj.set_start_time();
        traj_active_ = true;
        res.success = true;
        ROS_INFO("Executing trajectory...");
        last_cb_ = "executeTrajectory";
        return true;
      }
      else
      {
        res.success = false;
        res.message = "Could not transition to null tracker";
        ROS_WARN("Could not transition to null tracker");
        return true;
      }

      return false;
    }

    DemoServices(std::shared_ptr<MAVManager> m);

  private:

    ros::NodeHandle priv_nh_;
    bool useRadioForVelocity_;
    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);

    std::map<std::string, uint8_t> services_map;
};

void DemoServices::odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  auto radio = mav->radio();
  bool serial = radio[4] > 0;


  if (traj_active_)
  {
    if (last_cb_ == "executeTrajectory" && mav->active_tracker() == "std_trackers/NullTracker")
    {
      quadrotor_msgs::PositionCommand goal;
      traj.UpdateGoal(goal);
      mav->setPositionCommand(goal);

      if (traj.isCompleted())
        traj_active_ = !(mav->hover());
    }
    else
      traj_active_ = false;
  }
};

DemoServices::DemoServices(std::shared_ptr<MAVManager> m) :
  MAVManagerServices(m),
  traj_active_(false),
{
  // The trajectory filename
  std::string traj_filename;
  nh_.param("traj_filename", traj_filename, std::string("traj.csv"));
  ROS_INFO("Using traj: %s", traj_filename.c_str());
  traj.set_filename(traj_filename);

  // Subscribers
  odom_sub_ = priv_nh_.subscribe("odom", 1, &DemoServices::odometry_cb, this);

  srvs_.push_back(nh_.advertiseService("useRadioForVelocity", &DemoServices::useRadioForVelocity_cb, this));
  srvs_.push_back(nh_.advertiseService("prepTraj", &DemoServices::prepTraj_cb, this));
  srvs_.push_back(nh_.advertiseService("executeTrajectory", &DemoServices::executeTrajectory_cb, this));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  std::shared_ptr<MAVManager> mav = std::make_shared<MAVManager>();

  DemoServices mm_srvs(mav);

  ros::spin();

  return 0;
}
