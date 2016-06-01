#include <mav_manager/mav_manager_services.h>

#include <trajectory/trajectory.h>
#include <string>
#include <boost/thread.hpp>

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

    boost::thread *bt_;
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
    bool loadTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      // traj.LoadTrajectory(); // This is blocking, so we use the following instead
      traj.resetFlags();
      bt_ = new boost::thread(&Trajectory::LoadTrajectory, &traj);
      res.success = true;
      res.message = "Loading trajectory";
      last_cb_ = "loadTraj";
      return true;
    }
    bool prepTraj_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      if (traj.isLoaded())
      {
        bt_->detach();

        traj.setOffsets(req.goal[0], req.goal[1], req.goal[2], 0.0);
        ROS_INFO("Trajectory Offsets set to %2.3f, %2.3f, %2.3f, %2.3f", req.goal[0], req.goal[1], req.goal[2], 0.0);
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
    bool executeTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      // Make sure the trajectory is loaded
      if (!traj.isLoaded())
      {
        res.success = false;
        res.message = "Trajectory not loaded. Cannot start trajectory.";
        ROS_WARN("%s", res.message.c_str());
        return false;
      }

      // Check the velocity error
      float vel = (mav->vel()).norm();
      if (vel > 0.1)
      {
        res.success = false;
        res.message = "Velocity too large to start trajectory";
        ROS_WARN("%s", res.message.c_str());
        return false;
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
        return false;
      }

      if(mav->setPositionCommand(goal))
      {
        traj.set_start_time();
        traj_active_ = true;
        res.success = true;
        ROS_INFO("Executing trajectory...");
        last_cb_ = "executeTraj";
        return true;
      }
      else
      {
        res.success = false;
        res.message = "Could not transition to null tracker";
        ROS_WARN("Could not transition to null tracker");
        return false; 
      }

      return false;
    }

    DemoServices(std::shared_ptr<MAVManager> m);

  private:

    ros::NodeHandle priv_nh_;
    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg);

    std::map<std::string, uint8_t> services_map;
};

void DemoServices::odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  auto radio = mav->radio();
  bool serial = radio[4] > 0;


  if (traj_active_)
  {
    // There is a race condition here. active_tracker may not be correct?
    // auto active_tracker = mav->active_tracker();
    if (last_cb_ == "executeTraj") //&& active_tracker == "std_trackers/NullTracker")
    {
      quadrotor_msgs::PositionCommand goal;
      traj.UpdateGoal(goal);
      mav->setPositionCommand(goal);

      // TODO: Here, check the projection of b3 desired to disable attitude gains. We might
      // prefer to do this in matlab and just set the gains every time.
      if (false && goal.acceleration.z <= -9.81 + 1.0)
      {
        quadrotor_msgs::AttitudeGains msg;
        msg.kR[0]  = 0; msg.kR[1]  = 0; msg.kR[2]  = 0;
        msg.kOm[0] = 0; msg.kOm[1] = 0; msg.kOm[2] = 0;
        mav->setAttitudeGains(msg);
        // Maybe do this directly with a publisher to so3_control
        // mav->setAttitudeGains(
        // Need to make sure the gains go back to their original value (or something soft)
      }

      if (traj.isCompleted())
        traj_active_ = !(mav->hover());
    }
    else
    {
      ROS_INFO("Trajectory no longer active."); // active_tracker = %s", active_tracker.c_str());
      traj_active_ = false;
    }
  }
};

DemoServices::DemoServices(std::shared_ptr<MAVManager> m) :
  MAVManagerServices(m),
  traj_active_(false)
{
  // The trajectory filename
  std::string traj_filename;
  nh_.param("traj_filename", traj_filename, std::string("traj.csv"));
  ROS_INFO("Using traj: %s", traj_filename.c_str());
  traj.set_filename(traj_filename);

  // Subscribers
  odom_sub_ = priv_nh_.subscribe("odom", 1, &DemoServices::odometry_cb, this);

  srvs_.push_back(nh_.advertiseService("useRadioForVelocity", &DemoServices::useRadioForVelocity_cb, this));
  srvs_.push_back(nh_.advertiseService("loadTraj", &DemoServices::loadTraj_cb, this));
  srvs_.push_back(nh_.advertiseService("prepTraj", &DemoServices::prepTraj_cb, this));
  srvs_.push_back(nh_.advertiseService("executeTraj", &DemoServices::executeTraj_cb, this));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  std::shared_ptr<MAVManager> mav = std::make_shared<MAVManager>();

  DemoServices mm_srvs(mav);

  ros::spin();

  return 0;
}
