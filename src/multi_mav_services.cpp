#include <ros/ros.h>
#include <mav_manager/mav_manager_services.h>

class MMControl
{
  public:
    MMControl();

  protected:


  private:
    ros::NodeHandle nh, priv_nh;
    std::vector<std::string> model_names_;
    std::vector<std::array<float, 4>> offsets_;
    int num_bots;

    std::vector<ros::ServiceClient>
      sc_motors,
      sc_takeoff,
      sc_goHome,
      sc_goTo,
      sc_setDesVelInWorldFrame,
      sc_hover,
      sc_ehover,
      sc_land,
      sc_eland,
      sc_estop,
      sc_loadTraj,
      sc_prepTraj,
      sc_executeTraj;

    std::vector<ros::ServiceServer> srvs;

    bool motors_cb(mav_manager::Bool::Request &req, mav_manager::Bool::Response &res) {
      return loop<mav_manager::Bool>(req, res, sc_motors, "motors");
    }
    bool takeoff_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_takeoff, "takeoff");
    }
    bool goHome_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_goHome, "goHome");
    }
    bool goTo_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res)
    {
      // Check equal length of offsets_ and goTo service call
      if (offsets_.size() != sc_goTo.size())
      {
        res.success = false;
        res.message = "offsets_.size() != sc_goTo.size()";
        return true;
      }

      mav_manager::Vec4 srv;

      res.success = true;
      for (unsigned i = 0; i < sc_goTo.size(); i++)
      {
        // TODO: Also need to check for collisions

        for (unsigned j = 0; j < 4; j++)
          srv.request.goal[j] = req.goal[j] + offsets_[i][j];

        if (!sc_goTo[i].call(srv))
        {
          res.success = false;
          res.message += "Robot " + std::to_string(i) + " failed during goTo_cb.  ";
        }
      }
      return res.success;
    }
    bool setDesVelInWorldFrame_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res) {
      return loop<mav_manager::Vec4>(req, res, sc_setDesVelInWorldFrame, "setDesVelInWorldFrame");
    }
    bool hover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_hover, "hover");
    }
    bool ehover_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_ehover, "ehover");
    }
    bool land_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_land, "land");
    }
    bool eland_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_eland, "eland");
    }
    bool estop_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_estop, "estop");
    }
    bool loadTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_loadTraj, "loadTraj");
    }
    bool prepTraj_cb(mav_manager::Vec4::Request &req, mav_manager::Vec4::Response &res) {
      return loop<mav_manager::Vec4>(req, res, sc_prepTraj, "prepTraj");
    }
    bool executeTraj_cb(mav_manager::Trigger::Request &req, mav_manager::Trigger::Response &res) {
      return loop<mav_manager::Trigger>(req, res, sc_executeTraj, "executeTraj");
    }

    template <typename T>
    bool loop(typename T::Request &req, typename T::Response &res, std::vector<ros::ServiceClient> service_clients, std::string str)
    {
      T srv;
      srv.request = req;

      res.success = true;
      for (unsigned i = 0; i < service_clients.size(); i++)
      {
        if (!service_clients[i].call(srv))
        {
          res.success = false;
          res.message = res.message + model_names_[i] + " failed to call " + str + ".\n";
        }
      }
      return true;
    }
};

MMControl::MMControl() : nh("multi_mav_services"), priv_nh("")
{
  ros::Duration(5.0).sleep();

  priv_nh.getParam("model_names", model_names_);
  num_bots = model_names_.size();

  std::cout << "Constructing multi_mav_control with " << num_bots << " bots" << std::endl;

  for (unsigned i = 0; i < num_bots; i++)
  {
    std::cout << "Starting service clients for " << model_names_[i] << std::endl;
    sc_motors.push_back(nh.serviceClient<mav_manager::Bool>(               "/" + model_names_[i] + "/mav_services/motors"));
    sc_takeoff.push_back(nh.serviceClient<mav_manager::Trigger>(           "/" + model_names_[i] + "/mav_services/takeoff"));
    sc_goHome.push_back(nh.serviceClient<mav_manager::Trigger>(            "/" + model_names_[i] + "/mav_services/goHome"));
    sc_goTo.push_back(nh.serviceClient<mav_manager::Vec4>(                 "/" + model_names_[i] + "/mav_services/goTo"));
    sc_setDesVelInWorldFrame.push_back(nh.serviceClient<mav_manager::Vec4>("/" + model_names_[i] + "/mav_services/setDesVelInWorldFrame"));
    sc_hover.push_back(nh.serviceClient<mav_manager::Trigger>(             "/" + model_names_[i] + "/mav_services/hover"));
    sc_ehover.push_back(nh.serviceClient<mav_manager::Trigger>(            "/" + model_names_[i] + "/mav_services/ehover"));
    sc_land.push_back(nh.serviceClient<mav_manager::Trigger>(              "/" + model_names_[i] + "/mav_services/land"));
    sc_eland.push_back(nh.serviceClient<mav_manager::Trigger>(             "/" + model_names_[i] + "/mav_services/eland"));
    sc_estop.push_back(nh.serviceClient<mav_manager::Trigger>(             "/" + model_names_[i] + "/mav_services/estop"));
    sc_loadTraj.push_back(nh.serviceClient<mav_manager::Trigger>(          "/" + model_names_[i] + "/mav_services/loadTraj"));
    sc_prepTraj.push_back(nh.serviceClient<mav_manager::Vec4>(             "/" + model_names_[i] + "/mav_services/prepTraj"));
    sc_executeTraj.push_back(nh.serviceClient<mav_manager::Trigger>(       "/" + model_names_[i] + "/mav_services/executeTraj"));

    std::vector<float> offsets;
    priv_nh.getParam("/" + model_names_[i] + "/formation_offsets", offsets);
    if (offsets.size() == 4)
    {
      std::array<float, 4> offsets_arr = {offsets[0], offsets[1], offsets[2], offsets[3]};
      offsets_.push_back(offsets_arr);
    }
    else
      ROS_ERROR("offsets.size() = %lu for %s, but should be offsets.size() = 4.", offsets.size(), model_names_[i].c_str());
  }

  srvs.push_back(nh.advertiseService("motors", &MMControl::motors_cb, this));
  srvs.push_back(nh.advertiseService("takeoff", &MMControl::takeoff_cb, this));
  srvs.push_back(nh.advertiseService("goHome", &MMControl::goHome_cb, this));
  srvs.push_back(nh.advertiseService("goTo", &MMControl::goTo_cb, this));
  srvs.push_back(nh.advertiseService("setDesVelInWorldFrame", &MMControl::setDesVelInWorldFrame_cb, this));
  srvs.push_back(nh.advertiseService("hover", &MMControl::hover_cb, this));
  srvs.push_back(nh.advertiseService("ehover", &MMControl::ehover_cb, this));
  srvs.push_back(nh.advertiseService("land", &MMControl::land_cb, this));
  srvs.push_back(nh.advertiseService("eland", &MMControl::eland_cb, this));
  srvs.push_back(nh.advertiseService("estop", &MMControl::estop_cb, this));
  srvs.push_back(nh.advertiseService("loadTraj", &MMControl::loadTraj_cb, this));
  srvs.push_back(nh.advertiseService("prepTraj", &MMControl::prepTraj_cb, this));
  srvs.push_back(nh.advertiseService("executeTraj", &MMControl::executeTraj_cb, this));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_mav_services");

  MMControl multi_mav_control;

  ros::spin();

  return 0;
}
