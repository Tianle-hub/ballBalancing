#include <ur10_controller/ur10_effort_controller.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <chrono>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    SimpleEffortControl::SimpleEffortControl(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      is_first_iter_(true),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      q_goal_(Vector6d::Zero()),
      spline_period_(100.0),
      delta_q_(Vector6d::Zero()),
      delta_qp_(Vector6d::Zero())
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("simple_effort_controller_data", 1);
      model_.initModel();
    }

    SimpleEffortControl::~SimpleEffortControl()
    {
    }

    void SimpleEffortControl::setQInit(const JointState &q_init)
    {
      q_init_ = q_init;
    }
    void SimpleEffortControl::setQHome(const JointState &q_home)
    {
      q_home_ = q_home;model_.initModel();
    }
    void SimpleEffortControl::setQPark(const JointState &q_park)
    {
      q_park_ = q_park;
    }

    bool SimpleEffortControl::init()
    {
      ROS_WARN_STREAM("SimpleEffortControl::init");
      std::vector<double> vec;

      // check namespace
      std::string ns = "~simple_effort_ctrl";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("SimpleEffortControl init(): Control gains not defined in:" << ns);
        m_error = true;
        return false;
      }

      // D GAINS
      ros::param::get(ns + "/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (size_t i = 0; i < STD_DOF; i++)
      {
        Kd_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kd: \n" << Kd_);

      // P GAINS
      ros::param::get(ns + "/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_(i, i) = vec[i] / Kd_(i, i);
      }
      ROS_WARN_STREAM("Kp: \n" << Kp_);

      // GOAL
      ros::param::get(ns + "/goal", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        q_goal_(i) = vec[i];
      }
      
      // total time
      ros::param::get(ns + "/time", spline_period_);
      if (!(spline_period_ > 0))
      {
        ROS_ERROR_STREAM("spline_period_: is negative:" << spline_period_);
        spline_period_ = 100.0;
      }

      ROS_WARN_STREAM("Goal [DEG]: \n" << q_goal_.transpose());
      ROS_WARN_STREAM("Total Time [s]: " << spline_period_);
      q_goal_ = DEG2RAD(q_goal_);
      ROS_WARN_STREAM("Goal [RAD]: \n" << q_goal_.transpose());
      return true;
    }

    bool SimpleEffortControl::start()
    {
      ROS_WARN_STREAM("SimpleEffortControl::start");
      return true;
    }

    Vector6d SimpleEffortControl::update(const RobotTime &time, const JointState &state)
    {
      // auto start = std::chrono::high_resolution_clock::now();
      if (is_first_iter_)
      {
        q_start_ = state.q;
        ROS_WARN_STREAM("START [DEG]: \n" << q_start_.transpose());
        is_first_iter_ = false;
      }

      // control torque
      Vector6d tau;
      tau.setZero();

      // poly spline
      VVector6d vQd;
      vQd = getJointPVT5(q_start_, q_goal_, time.tD(), spline_period_);
      Vector6d q_goal_2;

      delta_q_ = state.q - vQd[0];
      delta_qp_ = state.qp - vQd[1];

      // reference
      JointState js_r;
      js_r = state;
      js_r.qp = vQd[1] - Kp_ * delta_q_;
      js_r.qpp = vQd[2] - Kp_ * delta_qp_;

      // torque
      Vector6d Sq = state.qp - js_r.qp;
      tau = -Kd_ * Sq;

      // Cartesian space
      if (time.tD() > 10.)
      {
        Matrix3d T0_W = AngleAxisd(M_PI, Vector3d::UnitZ()).matrix();
        Vector3d x_goal_t(1.0, 0.164, 0.750);
        // x_goal_t(2) = x_goal_t(2) - (time.tD()-10.)*0.05;
        x_goal_t = T0_W.inverse() * x_goal_t;

        Eigen::Quaterniond x_goal_r(0.71, 0, 0, 0.71);
        // Matrix3d x_goal_rm = AngleAxisd(-M_PI/2, Vector3d::UnitZ()).matrix() * T0_W.inverse() * x_goal_r.toRotationMatrix();
        Matrix3d x_goal_rm = T0_W.inverse() * x_goal_r.toRotationMatrix();
        Vector6d delta_x;

        Matrix4d Xef = model_.computeEEPos(state.q);
        Matrix3d Xef_rm = Xef.block<3,3>(0,0);
        Vector3d Xef_t = Xef.block<3,1>(0,3);

        Eigen::AngleAxisd delta_x_ra(x_goal_rm * Xef_rm.transpose());
        Vector3d delta_x_r = delta_x_ra.angle() * delta_x_ra.axis();

        delta_x << Xef_t(0) - x_goal_t(0), Xef_t(1) - x_goal_t(1), Xef_t(2) - x_goal_t(2), -delta_x_r(0), -delta_x_r(1), -delta_x_r(2);
        Vector6d xpr = - delta_x;

        Matrix6d J = model_.computeEEJacobian(state.q);

        Vector6d Xefp = J * state.qp;
        Vector6d Sx = Xefp - xpr;
        Sq = J.inverse() * Sx;
        tau = - 0.5 * Kd_ * Sq;
      }


      // publish the ControlData (only for debugging)
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = state.q(i);
        msg.qp[i] = state.qp(i);
        msg.qpp[i] = state.qpp(i);

        msg.qd[i] = vQd[0](i);
        msg.qpd[i] = vQd[1](i);

        msg.Dq[i] = delta_q_(i);
        msg.Dqp[i] = delta_qp_(i);

        // msg.Xef_0[i] = Xef(i);
        // msg.Xefp_0[i] = Xefp(i);

        // msg.Xd_0[i] = vXd[0](i);
        // msg.Xdp_0[i] = vXd[1](i);

        msg.torques[i] = state.tau(i);

        msg.DX[i] = Sq[i];
      }
      control_data_pub_.publish(msg);

      Vector6d G = model_.computeGeneralizedGravity(state.q);
      std::vector<Matrix4d> H_stack = model_.computeForwardKinematics(state.q);
      std::vector<Matrix4d> Hcm_stack = model_.computeFKCoMs(state.q);

      pubDH(H_stack, Hcm_stack);

      Vector6d qpr = Vector6d::Zero();
      Vector6d qppr = Vector6d::Zero();
      model_.updateJointState(state);
      auto Yr = model_.computeRefRegressor(state.q, state.qp, js_r.qp, js_r.qpp);
      auto Theta = model_.getTheta();
      model_.updateTheta(state.q, state.qp, js_r.qp, js_r.qpp, Sq);

      tau = tau + Yr*Theta;

      // auto stop = std::chrono::high_resolution_clock::now();
      // auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
      // std::cout << duration.count() << std::endl;

      return tau;
    }

    bool SimpleEffortControl::stop()
    {
      return true;
    }

    bool SimpleEffortControl::pubDH(const std::vector<Matrix4d> &H_stack, const std::vector<Matrix4d> &Hcm_stack)
    {
      tf::Transform transform;
      tf::Quaternion q;
      Eigen::Quaterniond q_eigen;
      Matrix4d H;

      // base_link
      transform.setOrigin(tf::Vector3(0, 0, 0));
      q.setRPY(0,0,0);
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

      // dh_base_link
      transform.setOrigin(tf::Vector3(0, 0, 0));
      q.setRPY(0,0,3.1415926);
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dh_joint_0"));

      for (int i = 0; i < STD_DOF; ++i)
      {
        H = H_stack[i];
        q_eigen = H.block<3,3>(0,0);
        q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());

        transform.setOrigin(tf::Vector3(H(0,3), H(1,3), H(2,3)));
        transform.setRotation(q);
        
        dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_joint_0", "dh_joint_" + std::to_string(i+1)));
      } 

      for (int i = 0; i < STD_DOF; ++i)
      {
        H = Hcm_stack[i];
        q_eigen = H.block<3,3>(0,0);
        q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());

        transform.setOrigin(tf::Vector3(H(0,3), H(1,3), H(2,3)));
        transform.setRotation(q);
        
        dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_joint_0", "dh_com_" + std::to_string(i)));
      } 

      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
