#include <tum_ics_ur10_controller_tutorial/simple_effort_controller.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

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
      q_home_ = q_home;
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

      // erros
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

        msg.torques[i] = state.tau(i);
      }
      control_data_pub_.publish(msg);

      pubDH(state);
      // ROS_WARN_STREAM("tau=" << tau.transpose());
      return tau;
    }

    bool SimpleEffortControl::stop()
    {
      return true;
    }

    bool SimpleEffortControl::pubDH(const JointState &state)
    {
      tf::Transform transform;
      tf::Quaternion q;
      Eigen::Quaterniond q_eigen;
      Eigen::Matrix3d mat;
      double q1 = state.q(0);
      double q2 = state.q(1);
      double q3 = state.q(2);
      double q4 = state.q(3);
      double q5 = state.q(4);
      double q6 = state.q(5);

      double d1 = 0.1273;
      double a2 = -0.612;
      double a3 = -0.5723;
      double d4 = 0.163941;
      double d5 = 0.1157;
      double d6 = 0.0922;

      // base_link
      transform.setOrigin(tf::Vector3(0, 0, 0));
      q.setRPY(0,0,0);
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

      // dh_base_link
      transform.setOrigin(tf::Vector3(0, 0, 0));
      q.setRPY(0,0,3.1415926);
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dh_base_link"));

      // dh_joint_1
      transform.setOrigin(tf::Vector3(0, 0, d1));
      mat(0, 0) = cos(q1);
      mat(0, 1) = 0;
      mat(0, 2) = sin(q1);
      mat(1, 0) = sin(q1);
      mat(1, 1) = 0;
      mat(1, 2) = -cos(q1);
      mat(2, 0) = 0;
      mat(2, 1) = 1;
      mat(2, 2) = 0;
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_joint_1"));

      // dh_joint_2
      transform.setOrigin(tf::Vector3(a2*cos(q1)*cos(q2), a2*cos(q2)*sin(q1), d1 + a2*sin(q2)));
      mat(0, 0) = cos(q1)*cos(q2);
      mat(0, 1) = -cos(q1)*sin(q2);
      mat(0, 2) = sin(q1);
      mat(1, 0) = cos(q2)*sin(q1);
      mat(1, 1) = -sin(q1)*sin(q2);
      mat(1, 2) = -cos(q1);
      mat(2, 0) = sin(q2);
      mat(2, 1) = cos(q2);
      mat(2, 2) = 0;
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_joint_2"));

      // dh_joint_3
      transform.setOrigin(tf::Vector3(a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + a2*sin(q2) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2)));
      mat(0, 0) = cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3);
      mat(0, 1) = - cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2);
      mat(0, 2) = sin(q1);
      mat(1, 0) = cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3);
      mat(1, 1) = - cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2);
      mat(1, 2) = -cos(q1);
      mat(2, 0) = cos(q2)*sin(q3) + cos(q3)*sin(q2);
      mat(2, 1) = cos(q2)*cos(q3) - sin(q2)*sin(q3);
      mat(2, 2) = 0;
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_joint_3"));

      // dh_joint_4
      transform.setOrigin(tf::Vector3(d4*sin(q1) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), a2*cos(q2)*sin(q1) - d4*cos(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + a2*sin(q2) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2)));
      mat(0, 0) = cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2));
      mat(0, 1) = sin(q1);
      mat(0, 2) = cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
      mat(1, 0) = - cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
      mat(1, 1) = -cos(q1);
      mat(1, 2) = cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
      mat(2, 0) = cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3));
      mat(2, 1) = 0;
      mat(2, 2) = sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3));
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_joint_4"));

      // dh_joint_5
      transform.setOrigin(tf::Vector3(d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d4*sin(q1) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - d4*cos(q1) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2)));
      mat(0, 0) = sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
      mat(0, 1) = - cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
      mat(0, 2) = cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
      mat(1, 0) = - cos(q1)*sin(q5) - cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));
      mat(1, 1) = sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
      mat(1, 2) = sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5);
      mat(2, 0) = cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      mat(2, 1) = cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2));
      mat(2, 2) = -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_joint_5"));

      // // dh_joint_
      // transform.setOrigin(tf::Vector3());
      // mat(0, 0) = ;
      // mat(0, 1) = ;
      // mat(0, 2) = ;
      // mat(1, 0) = ;
      // mat(1, 1) = ;
      // mat(1, 2) = ;
      // mat(2, 0) = ;
      // mat(2, 1) = ;
      // mat(2, 2) = ;
      // q_eigen = mat;
      // q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      // transform.setRotation(q);
      // dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_joint_"));


      // dh_ee
      transform.setOrigin(tf::Vector3(d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d4*sin(q1) + d6*(cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - d4*cos(q1) - d6*(cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2) - d6*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))) );
      mat(0, 0) = cos(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
      mat(0, 1) = - sin(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
      mat(0, 2) = cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
      mat(1, 0) = - cos(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)));
      mat(1, 1) = sin(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)));
      mat(1, 2) = sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5);
      mat(2, 0) = sin(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + cos(q5)*cos(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      mat(2, 1) = cos(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - cos(q5)*sin(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      mat(2, 2) = -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_ee"));

      double tcm11 = 0.021;
      double tcm12 = 0.000;
      double tcm13 = 0.027;
      double tcm21 = 0.38;
      double tcm22 = 0.000;
      double tcm23 = 0.158;
      double tcm31 = 0.24;
      double tcm32 = 0.000;
      double tcm33 = 0.068;
      double tcm41 = 0.000;
      double tcm42 = 0.007;
      double tcm43 = 0.018;
      double tcm51 = 0.000;
      double tcm52 = 0.007;
      double tcm53 = 0.018;
      double tcm61 = 0;
      double tcm62 = 0;
      double tcm63 = -0.026;

      // dh_cm_1
      transform.setOrigin(tf::Vector3(tcm11*cos(q1) + tcm13*sin(q1), tcm11*sin(q1) - tcm13*cos(q1), d1 + tcm12));
      mat(0, 0) = cos(q1);
      mat(0, 1) = 0;
      mat(0, 2) = sin(q1);
      mat(1, 0) = sin(q1);
      mat(1, 1) = 0;
      mat(1, 2) = -cos(q1);
      mat(2, 0) = 0;
      mat(2, 1) = 1;
      mat(2, 2) = 0;
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_cm_1"));

      // dh_cm_2
      transform.setOrigin(tf::Vector3(tcm23*sin(q1) + a2*cos(q1)*cos(q2) + tcm21*cos(q1)*cos(q2) - tcm22*cos(q1)*sin(q2), a2*cos(q2)*sin(q1) - tcm23*cos(q1) + tcm21*cos(q2)*sin(q1) - tcm22*sin(q1)*sin(q2), d1 + a2*sin(q2) + tcm22*cos(q2) + tcm21*sin(q2)));
      mat(0, 0) = cos(q1)*cos(q2);
      mat(0, 1) = -cos(q1)*sin(q2);
      mat(0, 2) = sin(q1);
      mat(1, 0) = cos(q2)*sin(q1);
      mat(1, 1) = -sin(q1)*sin(q2);
      mat(1, 2) = -cos(q1);
      mat(2, 0) = sin(q2);
      mat(2, 1) = cos(q2);
      mat(2, 2) = 0;
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_cm_2"));

      // dh_cm_3
      transform.setOrigin(tf::Vector3(tcm31*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - tcm32*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + tcm33*sin(q1) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), a2*cos(q2)*sin(q1) - tcm32*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - tcm33*cos(q1) - tcm31*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + tcm31*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + tcm32*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + a2*sin(q2) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2)));
      mat(0, 0) = cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3);
      mat(0, 1) = - cos(q1)*cos(q2)*sin(q3) - cos(q1)*cos(q3)*sin(q2);
      mat(0, 2) = sin(q1);
      mat(1, 0) = cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3);
      mat(1, 1) = - cos(q2)*sin(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2);
      mat(1, 2) = -cos(q1);
      mat(2, 0) = cos(q2)*sin(q3) + cos(q3)*sin(q2);
      mat(2, 1) = cos(q2)*cos(q3) - sin(q2)*sin(q3);
      mat(2, 2) = 0;
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_cm_3"));

      // dh_cm_4
      transform.setOrigin(tf::Vector3(tcm41*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + tcm43*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d4*sin(q1) + tcm42*sin(q1) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), tcm43*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - tcm41*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - d4*cos(q1) - tcm42*cos(q1) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + a2*sin(q2) + tcm41*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - tcm43*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2)));
      mat(0, 0) = cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2));
      mat(0, 1) = sin(q1);
      mat(0, 2) = cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
      mat(1, 0) = - cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
      mat(1, 1) = -cos(q1);
      mat(1, 2) = cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
      mat(2, 0) = cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3));
      mat(2, 1) = 0;
      mat(2, 2) = sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3));
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_cm_4"));

      // dh_cm_5
      transform.setOrigin(tf::Vector3(d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - tcm52*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d4*sin(q1) + tcm51*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + tcm53*(cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - tcm53*(cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - tcm51*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - tcm52*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - d4*cos(q1) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + tcm52*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2) + tcm51*cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - tcm53*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))));
      mat(0, 0) = sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
      mat(0, 1) = - cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
      mat(0, 2) = cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
      mat(1, 0) = - cos(q1)*sin(q5) - cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));
      mat(1, 1) = sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
      mat(1, 2) = sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5);
      mat(2, 0) = cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      mat(2, 1) = cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2));
      mat(2, 2) = -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_cm_5"));

      // dh_cm_ee
      transform.setOrigin(tf::Vector3(d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d4*sin(q1) + d6*(cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + tcm63*(cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + tcm61*(cos(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))) - tcm62*(sin(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) + cos(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3), d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - tcm63*(cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - tcm61*(cos(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) + sin(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))) + tcm62*(sin(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))) - d4*cos(q1) - d6*(cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3), d1 + tcm61*(sin(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + cos(q5)*cos(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))) + tcm62*(cos(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - cos(q5)*sin(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))) + a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + a3*cos(q2)*sin(q3) + a3*cos(q3)*sin(q2) - d6*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - tcm63*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))));
      mat(0, 0) = cos(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
      mat(0, 1) = - sin(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
      mat(0, 2) = cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
      mat(1, 0) = - cos(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)));
      mat(1, 1) = sin(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)));
      mat(1, 2) = sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5);
      mat(2, 0) = sin(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + cos(q5)*cos(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      mat(2, 1) = cos(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - cos(q5)*sin(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      mat(2, 2) = -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));
      q_eigen = mat;
      q.setValue(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
      transform.setRotation(q);
      dh_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dh_base_link", "dh_cm_ee"));
      return true;

    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
