#include <mav_manager/mav_manager_services.h>

#include <trajectory/trajectory.h>
#include <string>

class DemoServices : public MAVManagerServices
{
  public:

    Trajectory traj;

    // Subscribers
    ros::Subscriber odom_sub_;

    bool traj_active_;

    // Services that typically use the null_tracker also need to set traj_active to false
    bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
    {
      mav->set_motors(req.b);
      res.success = true;
      res.message = "Motors activated";
      traj_active_ = false;
      return true;
    }
    bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      res.success = mav->eland();
      res.message = "Emergency Landing";
      traj_active_ = false;
      return res.success;
    }
    bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      mav->estop();
      res.success = true;
      res.message = "Emergency Stop";
      traj_active_ = false;
      return true;
    }

    bool useRadioForVelocity_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
    {
      // TODO: Use setDesVel instead
      //
      // res.success = mav_.useRadioForVelocity(req.b);
      res.success = false;
      if (res.success)
      {
        if (req.b)
          res.message = "Using radio for velocity";
        else
          res.message = "No longer using radio for velocity";
      }
      else
        res.message = "Failed to transition";

      return res.success;
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
        return res.success;
      }
      else
        ROS_WARN("Cannot prep traj. Trajectory not loaded. Error Code: %d", traj.get_error_code());

      return false;
    }
    bool executeTrajectory_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
    {
      if (traj.isLoaded())
      {
        // Make sure starting conditions are close enough
        double vel = (mav->vel()).norm();
        if (vel < 0.1)
        {
          traj.set_start_time();
          quadrotor_msgs::PositionCommand goal;
          traj.UpdateGoal(goal);
          Vec3 des_pos(goal.position.x, goal.position.y, goal.position.z);

          double e = (mav->pos() - des_pos).norm();
          if (e < 0.15)
          {
            if(mav->setPositionCommand(goal))
            {
              traj.set_start_time();
              traj_active_ = true;
              res.success = true;
              return true;
            }
            else
              ROS_WARN("Could not transition to null tracker");
          }
          else
            ROS_WARN("Position error too large to start trajectory");
        }
        else
          ROS_WARN("Velocity too large to start trajectory");
      }
      else
        ROS_WARN("Trajectory not loaded. Cannot start trajectory.");

      return false;
    }

    void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {
      if (traj_active_)
      {
        if ((mav->active_tracker()).compare("std_trackers/NullTracker") == 0)
        {
          quadrotor_msgs::PositionCommand goal;
          traj.UpdateGoal(goal);
          mav->setPositionCommand(goal);

          if (traj.isCompleted())
          {
            traj_active_ = !(mav->hover());
          }
        }
        else
        {
          traj_active_ = false;
        }
      }
    }

    DemoServices(std::shared_ptr<MAVManager> m) : MAVManagerServices(m), traj_active_(false)
    {
      // The trajectory filename
      std::string traj_filename;
      nh_priv_.param("traj_filename", traj_filename, std::string("traj.csv"));
      ROS_INFO("Using traj: %s", traj_filename.c_str());
      traj.set_filename(traj_filename);

      // Subscribers
      odom_sub_ = nh_.subscribe("odom", 1, &DemoServices::odometry_cb, this);

      srvs_.push_back(nh_.advertiseService("useRadioForVelocity", &DemoServices::useRadioForVelocity_cb, this));
      srvs_.push_back(nh_.advertiseService("prepTraj", &DemoServices::prepTraj_cb, this));
      srvs_.push_back(nh_.advertiseService("executeTrajectory", &DemoServices::executeTrajectory_cb, this));
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");
  ros::NodeHandle nh;

  std::shared_ptr<MAVManager> mav = std::make_shared<MAVManager>();

  DemoServices mm_srvs(mav);
  
  ros::spin();

  return 0;
}
