#include <ur10_controller/ur10_effort_controller.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>
#include <chrono>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    UR10EffortControl::UR10EffortControl(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      is_first_iter_(true),
      Kp_c_(Matrix6d::Zero()),
      Kd_c_(Matrix6d::Zero()),
      Ki_c_(Matrix6d::Zero()),
      Kp_j_(Matrix6d::Zero()),
      Kd_j_(Matrix6d::Zero()),
      Ki_j_(Matrix6d::Zero()),
      q_goal_(Vector6d::Zero()),
      spline_period_(100.0),
      delta_q_(Vector6d::Zero()),
      delta_qp_(Vector6d::Zero())
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("simple_effort_controller_data", 1);
      model_.initModel();
      ball_controller.initModel(Vector4d(0., 0., 0.05, 0.), Vector2d(0, 0));
    }

    UR10EffortControl::~UR10EffortControl()
    {
    }

    void UR10EffortControl::setQInit(const JointState &q_init)
    {
      q_init_ = q_init;
    }
    void UR10EffortControl::setQHome(const JointState &q_home)
    {
      q_home_ = q_home;
    }
    void UR10EffortControl::setQPark(const JointState &q_park)
    {
      q_park_ = q_park;
    }

    bool UR10EffortControl::init()
    {
      ROS_WARN_STREAM("UR10EffortControl::init");
      std::vector<double> vec;

      // check namespace
      std::string ns = "~ur10_effort_ctrl";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("UR10EffortControl init(): Control gains not defined in:" << ns);
        m_error = true;
        return false;
      }

      // D GAINS
      ros::param::get(ns + "/joint/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d : wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (size_t i = 0; i < STD_DOF; i++)
      {
        Kd_j_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kd of joint space controller: \n" << Kd_j_);

      // P GAINS
      ros::param::get(ns + "/joint/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_j_(i, i) = vec[i] / Kd_j_(i, i);
      }
      ROS_WARN_STREAM("Kp of joint space controller: \n" << Kp_j_);

      // GOAL
      ros::param::get(ns + "/joint/goal", vec);
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
      ros::param::get(ns + "/joint/time", spline_period_);
      if (!(spline_period_ > 0))
      {
        ROS_ERROR_STREAM("spline_period_: is negative:" << spline_period_);
        spline_period_ = 100.0;
      }

      ROS_WARN_STREAM("Goal [DEG]: \n" << q_goal_.transpose());
      ROS_WARN_STREAM("Total Time [s]: " << spline_period_);
      q_goal_ = DEG2RAD(q_goal_);
      ROS_WARN_STREAM("Goal [RAD]: \n" << q_goal_.transpose());

      // D GAINS
      ros::param::get(ns + "/cartesian/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d : wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (size_t i = 0; i < STD_DOF; i++)
      {
        Kd_c_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kd of joint space controller: \n" << Kd_c_);

      // P GAINS
      ros::param::get(ns + "/cartesian/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_c_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kp of joint space controller: \n" << Kp_c_);
      return true;
    }

    bool UR10EffortControl::start()
    {
      ROS_WARN_STREAM("UR10EffortControl::start");
      return true;
    }

    Vector6d UR10EffortControl::update(const RobotTime &time, const JointState &state)
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

      if (time.tD() <= 50.)
      {
      // poly spline
      VVector6d vQd;
      vQd = getJointPVT5(q_start_, q_goal_, time.tD(), spline_period_);

      tau = jointPDController(time, state, vQd);
      }

      // Cartesian space
      if (time.tD() > 50.)
      {
        Vector3d x_goal_t(1.0, 0.164, 0.750);
        // x_goal_t(2) = x_goal_t(2) - (time.tD()-10.)*0.05;

        Eigen::Quaterniond x_goal_r(0.71, 0, 0, 0.71);

        Vector3d EE_pos_r = model_.computeEEPos(state.q).block<3,3>(0,0).eulerAngles(0, 1, 2);

        Vector4d ball_state = ball_controller.updateModel(time.tD() - 50., EE_pos_r.block<2,1>(0,0));

        Vector2d u_ball_d = ball_controller.update(ball_state);

        Vector6d x_goal;
        // x_goal << x_goal_t, (Eigen::AngleAxisd(M_PI/4, Vector3d::UnitX()) * x_goal_r).toRotationMatrix().eulerAngles(0, 1, 2);
        x_goal << x_goal_t, x_goal_r.toRotationMatrix().eulerAngles(0, 1, 2);

        // x_goal(3) = u_ball_d(0);
        // x_goal(4) = -u_ball_d(1);

        // ROS_INFO_STREAM("u_ball_d : \n" << u_ball_d);

        VVector6d EE_d;
        EE_d.resize(3);

        EE_d[0] = x_goal;
        EE_d[1] = Vector6d::Zero();
        EE_d[2] = Vector6d::Zero();

        tau = cartesianPDController(time, state, EE_d);
      }

      // Vector6d G = model_.computeGeneralizedGravity(state.q);
      std::vector<Matrix4d> H_stack = model_.computeForwardKinematics(state.q);
      std::vector<Matrix4d> Hcm_stack = model_.computeFKCoMs(state.q);

      pubDH(H_stack, Hcm_stack);

      // auto stop = std::chrono::high_resolution_clock::now();
      // auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
      // std::cout << duration.count() << std::endl;

      return tau;
    }

    bool UR10EffortControl::stop()
    {
      return true;
    }

    Vector6d  UR10EffortControl::jointPDController(const RobotTime &time, const JointState &state, const VVector6d &vQd)
    {
      Vector6d tau;

      delta_q_ = state.q - vQd[0];
      delta_qp_ = state.qp - vQd[1];

      // reference
      JointState js_r;
      js_r = state;
      js_r.qp = vQd[1] - Kp_j_ * delta_q_;
      js_r.qpp = vQd[2] - Kp_j_ * delta_qp_;

      // torque
      Vector6d Sq = state.qp - js_r.qp;

      auto Yr = model_.computeRefRegressor(state.q, state.qp, js_r.qp, js_r.qpp);
      auto Theta = model_.getTheta();
      model_.updateTheta(state.q, state.qp, js_r.qp, js_r.qpp, Sq);

      tau = -Kd_j_ * Sq + Yr * Theta;

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
        msg.qppd[i] = vQd[2](i);

        msg.Dq[i] = delta_q_(i);
        msg.Dqp[i] = delta_qp_(i);

        msg.torques[i] = state.tau(i);
      }
      control_data_pub_.publish(msg);

      return tau;
    }

    Vector6d  UR10EffortControl::cartesianPDController(const RobotTime &time, const JointState &state, const VVector6d &EE_d)
    {
      // current end effector state w.r.t base
      Matrix4d EE_pos = model_.computeEEPos(state.q);
      Vector3d EE_t = EE_pos.block<3,1>(0,3);
      Matrix3d EE_r = EE_pos.block<3,3>(0,0);

      Vector6d Xef_0;
      Xef_0 << EE_t, EE_r.eulerAngles(0, 1, 2);

      Matrix6d Jee = model_.computeEEJacobian(state.q);
      Matrix6d Jee_inv = Jee.inverse();
      Vector6d EE_vel = Jee * state.qp;
      
      // end effector reference w.r.t world
      Vector3d EE_t_d = EE_d[0].block<3,1>(0,0);
      Matrix3d EE_r_d = eulerXYZToRotationMatrix(EE_d[0].block<3,1>(3,0));

      // end effector reference w.r.t base
      Matrix3d RW_0 = model_.getTransformBaseToWorld().inverse().block<3,3>(0,0);
      EE_r_d = RW_0 * EE_r_d;
      EE_t_d = RW_0 * EE_t_d;

      // rotation error
      Eigen::AngleAxisd delta_x_r_aa(EE_r * EE_r_d.transpose());
      Vector3d delta_x_r = delta_x_r_aa.angle() * delta_x_r_aa.axis();

      // translation error 
      Vector3d delta_x_t = EE_t - EE_t_d;

      // error in cartesian space
      Vector6d delta_x;
      delta_x << delta_x_t, delta_x_r;

      Vector6d delta_xp = EE_vel - EE_d[1];

      // velocity reference in cartesian space
      Vector6d EE_vel_ref = EE_d[1] - Kp_c_ * delta_x;

      // acceleration reference in cartesian space
      Matrix6d Jee_dot = model_.computeEEJacobainDerivative(state.q, state.qp);
      Vector6d EE_acc_ref = EE_d[2] - Kp_c_ * delta_xp;

      Vector6d Sx = EE_vel - EE_vel_ref;
      Vector6d Sq = Jee_inv * Sx;

      Sx(3) = -Sx(3);

      Vector6d Sq2 = Jee_inv * Sx;
      // model_.updateJointState(state);

      // joint reference for regressor
      Vector6d qpr = Jee_inv * EE_vel_ref;
      Vector6d qppr = Jee_inv * (EE_acc_ref - Jee_dot * state.qp);

      auto Yr = model_.computeRefRegressor(state.q, state.qp, qpr, qppr);
      auto Theta = model_.getTheta();
      model_.updateTheta(state.q, state.qp, qpr, qppr, Sq2);

      // Sq(3) = -Sq(3);

      Vector6d tau = - Kd_c_ * Sq + Yr * Theta;

      // publish the ControlData (only for debugging)
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = state.q(i);
        msg.qp[i] = state.qp(i);
        msg.qpp[i] = state.qpp(i);

        msg.Xef_0[i] = Xef_0(i);
        msg.Xefp_0[i] = EE_vel(i);

        msg.Xd_0[i] = EE_d[0](i);
        msg.Xdp_0[i] = EE_d[1](i);

        msg.DX[i] = delta_x(i);
        msg.DXp[i] = delta_xp(i);

        msg.torques[i] = state.tau(i);
      }
      control_data_pub_.publish(msg);

      return tau;
    }

    Matrix3d UR10EffortControl::eulerXYZToRotationMatrix(const Vector3d &euler)
    {
      Eigen::Quaterniond R = AngleAxisd(euler(0), Vector3d::UnitX())
       * AngleAxisd(euler(1), Vector3d::UnitY()) * AngleAxisd(euler(2), Vector3d::UnitZ());

      return R.toRotationMatrix();
    }

    bool UR10EffortControl::pubDH(const std::vector<Matrix4d> &H_stack, const std::vector<Matrix4d> &Hcm_stack)
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
