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

    bool useRadioForVelocity_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      if (useRadioForVelocity_ == false)
      {
        if (mav->have_recent_output_data())
        {
          res.success = setVelFromRadio();
          if (res.success)
          {
            useRadioForVelocity_ = true;
            res.message = "Using radio for velocity";
            last_cb_ = "useRadioForVelocity";
          }
          else
          {
            res.message = "Couldn't set velocity using the radio.";
            ROS_WARN("%s", res.message.c_str());
          }
        }
        else
        {
          res.success = false;
          res.message = "No recent radio data. Not using radio for velocity control.";
          ROS_WARN("%s", res.message.c_str());
        }
      }
      else
      {
        res.success = mav->hover();
        if (res.success)
        {
          useRadioForVelocity_ = false;
          res.message = "No longer using radio for velocity.";
        }
        else
        {
          res.message = "Could not hover. Still using radio for velocity.";
          ROS_WARN("%s", res.message.c_str());
        }
      }
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

    bool setVelFromRadio()
    {
      auto radio = mav->radio();

      // constants
      float scale = 255.0 / 2.0;
      float rc_max_v = 1.0;
      float rc_max_w = 90.0 * M_PI / 180.0;

      // scale radio
      float vel[4] = {0, 0, 0, 0};
      vel[0] = - ((float)radio[0] - scale) / scale * rc_max_v;
      vel[1] = - ((float)radio[1] - scale) / scale * rc_max_v;

      // Only consider z velocity if the FLT Mode switch is toggled
      if (radio[5] > 0)
        vel[2] = ((float)radio[2] - scale) / scale * rc_max_v;

      vel[3] = - ((float)radio[3] - scale) / scale * rc_max_w;

      // Deadbands on velocity
      float db[4] = {0.1, 0.1, 0.15, 0.03};
      for (unsigned int i = 0; i < 4; i++) vel[i] = deadband(vel[i], db[i]);

      return mav->setDesVelInWorldFrame(vel[0], vel[1], vel[2], vel[3]);
    }
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

  if (useRadioForVelocity_)
  {
    if (last_cb_ == "useRadioForVelocity" && mav->active_tracker() == "std_trackers/VelocityTracker")
      setVelFromRadio();
    else
      useRadioForVelocity_ = false;
  }
};

DemoServices::DemoServices(std::shared_ptr<MAVManager> m) :
  MAVManagerServices(m),
  traj_active_(false),
  useRadioForVelocity_(false)
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
