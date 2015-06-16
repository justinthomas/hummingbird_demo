#include <mav_manager/manager.h>
#include <mav_manager/Bool.h>
#include <mav_manager/Trigger.h>
#include <mav_manager/Vec4.h>

#include <trajectory/trajectory.h>
#include <string>

// Typedefs
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;

// quadrotor_msgs::PositionCommand traj_goal;
// static std::string traj_filename;

class MAV_Services
{
  public:

  // Let's make an MAV
  MAVManager mav_;
  Trajectory traj;

  // Subscribers
  ros::Subscriber odom_sub_;

  // Publishers
  // ros::Publisher pub_position_cmd_;

  // Services
  ros::ServiceServer
    srv_motors_,
    srv_takeoff_,
    srv_goHome_,
    srv_goTo_,
    srv_setDesVelWorld_,
    srv_setDesVelBody_,
    srv_useRadioForVelocity_,
    srv_prepTraj_,
    srv_executeTrajectory_,
    srv_hover_,
    srv_ehover_,
    srv_eland_,
    srv_estop_;

  bool traj_active_;


  bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
  {
    mav_.motors(req.b);
    res.success = true;
    res.message = "Motors activated";
    return true;
  }
  bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.takeoff();
    res.message = "Takeoff";
    return res.success;
  }
  bool goHome_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.goHome();
    res.message = "Going home";
    return res.success;
  }
  bool goTo_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.success = mav_.goTo(goal);
    res.message = "Go To";
    return res.success;
  }
  bool setDesVelWorld_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.success = mav_.setDesVelWorld(goal);
    res.message = "World Velocity";
    return res.success;
  }
  bool setDesVelBody_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
  {
    Vec4 goal(req.goal[0], req.goal[1], req.goal[2], req.goal[3]);
    res.success = mav_.setDesVelBody(goal);
    res.message = "Body Velocity";
    return res.success;
  }
  bool useRadioForVelocity_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res)
  {
    res.success = mav_.useRadioForVelocity(req.b);
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
  bool useNullTracker_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.useNullTracker();
    if (res.success)
      res.message = "Using NullTracker";
    else
      res.message = "Failed to transition to NullTracker";

    return res.success;
  }
  bool prepTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    if (traj.isLoaded())
    {
      traj.set_start_time();
      quadrotor_msgs::PositionCommand goal;
      traj.UpdateGoal(goal);

      res.success = mav_.goTo(goal.position.x, goal.position.y, goal.position.z, goal.yaw);
      return res.success;
    }
    else
      ROS_WARN("Cannot prep traj. Trajectory not loaded.");

    return false;
  }
  bool executeTrajectory_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    if (traj.isLoaded())
    {
      // Make sure starting conditions are close enough
      double vel = (mav_.vel()).norm();
      if (vel < 0.1)
      {
        traj.set_start_time();
        quadrotor_msgs::PositionCommand goal;
        traj.UpdateGoal(goal);
        Vec3 des_pos(goal.position.x, goal.position.y, goal.position.z);

        double e = (mav_.pos() - des_pos).norm();
        if (e < 0.15)
        {
          if(mav_.setPositionCommand(goal))
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
  bool hover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.hover();
    res.message = "Hover";
    traj_active_ = false;
    return res.success;
  }
  bool ehover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.ehover();
    res.message = "Emergency Hover";
    traj_active_ = false;
    return res.success;
  }
  bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    res.success = mav_.eland();
    res.message = "Emergency Landing";
    traj_active_ = false;
    return res.success;
  }
  bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res)
  {
    mav_.estop();
    res.success = true;
    res.message = "Emergency Stop";
    traj_active_ = false;
    return true;
  }

  void odometry_cb(const nav_msgs::Odometry::ConstPtr &msg) {

    if (traj_active_)
    {
      quadrotor_msgs::PositionCommand goal;
      traj.UpdateGoal(goal);
      mav_.setPositionCommand(goal);

      if (traj.isCompleted())
      {
        traj_active_ = !(mav_.hover());
      }
    }
  }

  // Constructor
  MAV_Services() : traj_active_(false), nh_(""), nh_priv_("~") {

    // The trajectory filename
    std::string traj_filename;
    nh_priv_.param("traj_filename", traj_filename, std::string("traj.csv"));
    traj.set_filename(traj_filename);

    if (traj.LoadTrajectory())
      ROS_INFO("Trajectory loaded");
    else
      ROS_WARN("Trajectory could not be loaded.");

    // Subscribers
    odom_sub_ = nh_.subscribe("odom", 1, &MAV_Services::odometry_cb, this);

    // Publishers
    // pub_position_cmd_ = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 1);

    // Services
    srv_motors_ = nh_.advertiseService("motors", &MAV_Services::motors_cb, this);
    srv_takeoff_ = nh_.advertiseService("takeoff", &MAV_Services::takeoff_cb, this);
    srv_goHome_ = nh_.advertiseService("goHome", &MAV_Services::goHome_cb, this);
    srv_goTo_ = nh_.advertiseService("goTo", &MAV_Services::goTo_cb, this);
    srv_setDesVelWorld_ = nh_.advertiseService("setDesVelWorld", &MAV_Services::setDesVelWorld_cb, this);
    srv_setDesVelBody_ = nh_.advertiseService("setDesVelBody", &MAV_Services::setDesVelBody_cb, this);
    srv_useRadioForVelocity_ = nh_.advertiseService("useRadioForVelocity", &MAV_Services::useRadioForVelocity_cb, this);
    srv_prepTraj_ = nh_.advertiseService("prepTraj", &MAV_Services::prepTraj_cb, this);
    srv_executeTrajectory_ = nh_.advertiseService("executeTrajectory", &MAV_Services::executeTrajectory_cb, this);
    srv_hover_ = nh_.advertiseService("hover", &MAV_Services::hover_cb, this);
    srv_ehover_ = nh_.advertiseService("ehover", &MAV_Services::ehover_cb, this);
    srv_eland_ = nh_.advertiseService("eland", &MAV_Services::eland_cb, this);
    srv_estop_ = nh_.advertiseService("estop", &MAV_Services::estop_cb, this);
  }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");
  ros::NodeHandle nh("");

  MAV_Services mav_srvs;

  // Let's spin some rotors
  ros::spin();

  return 0;
}
