#ifndef UR10_EFFORT_CONTROLLER_EFFORT_CONTROLLER
#define UR10_EFFORT_CONTROLLER_EFFORT_CONTROLLER

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tf/transform_broadcaster.h>
#include <ur10_controller/UR10Model.h>
#include <ball_controller/ball_controller.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {
    enum ControllerType
    {
      JOINTPD,
      CARTESIANPD
    };
    
    class UR10EffortControl : public ControlEffort
    {
    private:
      bool is_first_iter_;

      Vector6d q_start_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;

      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;
      tf::TransformBroadcaster dh_br;

      // joint space controller
      Matrix6d Kp_j_;
      Matrix6d Kd_j_;
      Matrix6d Ki_j_;

      // cartesian space controller
      Matrix6d Kp_c_;
      Matrix6d Kd_c_;
      Matrix6d Ki_c_;

      Vector6d q_goal_;
      double spline_period_;

      Vector6d delta_q_;
      Vector6d delta_qp_;

      ur10_model::UR10Model model_;

      BallControl::BallController ball_controller;

    public:
      UR10EffortControl(double weight = 1.0, const QString &name = "UR10EffortCtrl");

      ~UR10EffortControl();

      void setQInit(const JointState &q_init);

      void setQHome(const JointState &q_home);

      void setQPark(const JointState &q_park);

    private:
      bool init();

      bool start();

      bool stop();

      Matrix3d eulerXYZToRotationMatrix(const Vector3d &euler);

      bool pubDH(const std::vector<Matrix4d> &H_stack, const std::vector<Matrix4d> &Hcm_stack);

      Vector6d update(const RobotTime &time, const JointState &state);

      Vector6d jointPDController(const RobotTime &time, const JointState &state, const VVector6d &vQd);

      Vector6d cartesianPDController(const RobotTime &time, const JointState &state, const VVector6d &EE_r);

    };

  } // namespace RobotControllers
} // namespace tuics_ur_robot_lli

#endif // UR10_EFFORT_CONTROLLER_EFFORT_CONTROLLER