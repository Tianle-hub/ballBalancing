#include <tum_ics_ur10_controller_tutorial/UR10Model.h>
#include <chrono>

using namespace std::chrono;

namespace tum_ics_ur_robot_lli
{
  namespace ur10_model
  {
    UR10Model::UR10Model()
    {
    }

    UR10Model::~UR10Model()
    {
    }

    void UR10Model::initModel(const JointState &state_init, const int n)
    {
      DOF = n;
      H_0_.resize(n);
      Hr_.resize(n);
      Hcm_0_.resize(n);
      J_0_.resize(n);
      ps_.l << 0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922;
      ps_.m << 7.1, 12.7, 4.27, 2, 2, 0.365;
      // ps_.m << 7.778, 12.93, 3.87, 1.96, 1.96, 0.202;
      ps_.I << 0.03408, 0.00002, -0.00425, 0.03529, 0.00008, 0.02156,
               0.02814, 0.00005, -0.01561, 0.77068, 0.00002, 0.76943,
               0.01014, 0.00008, 0.00916, 0.30928, 0.00000, 0.30646,
               0.00296, -0.00001, -0.00000, 0.00222, -0.00024, 0.00258,
               0.00296, -0.00001, -0.00000, 0.00222, -0.00024, 0.00258,
               0.00040, 0.00000,  -0.00000, 0.00041, 0.00000, 0.00034;
      ps_.tcm << 0.021, 0.000, 0.027,
                 0.38, 0.000, 0.158,
                 0.24, 0.000, 0.068,
                 0.000, 0.007, 0.018,
                 0.000, 0.007, 0.018,
                 0, 0, -0.026;
      g = -9.82;
      
      d << ps_.l(0), 0., 0., ps_.l(3), ps_.l(4), ps_.l(5);

      a << 0., ps_.l(1), ps_.l(2), 0., 0., 0.;

      alpha << M_PI/2, 0., 0., M_PI/2, -M_PI/2, 0;
    }

    void UR10Model::updateJointState(const JointState &state_new)
    {
      state_ = state_new;
    }

    std::vector<Matrix4d> UR10Model::computeForwardKinematics()
    {
      auto start = high_resolution_clock::now();
      // UR10Model::computeFKJoint0();
      // UR10Model::computeFKJoint1();
      // UR10Model::computeFKJoint2();
      // UR10Model::computeFKJoint3();
      // UR10Model::computeFKJoint4();
      // UR10Model::computeFKJoint5();


      H_0_[0] = relativeTrans(state_.q(0), d(0), a(0), alpha(0));

      for(int i = 1; i < DOF; ++i)
      {
        H_0_[i] = H_0_[i-1] * relativeTrans(state_.q(i), d(i), a(i), alpha(i));
      }
      // computeJacobian();
      // Hr_[0] << cos(state_.q(0)), 0,  sin(state_.q(0)),  0,
      //           sin(state_.q(0)), 0, -cos(state_.q(0)),  0,
      //                 0, 1,        0, ps_.l(0),
      //                 0, 0,        0,  1;

      // Hr_[1] << cos(state_.q(1)), -sin(state_.q(1)), 0, ps_.l(1)*cos(state_.q(1)),
      //           sin(state_.q(1)),  cos(state_.q(1)), 0, ps_.l(1)*sin(state_.q(1)),
      //                 0,        0, 1,          0,
      //                 0,        0, 0,          1;

      // Hr_[2] << cos(state_.q(2)), -sin(state_.q(2)), 0, ps_.l(2)*cos(state_.q(2)),
      //           sin(state_.q(2)),  cos(state_.q(2)), 0, ps_.l(2)*sin(state_.q(2)),
      //                 0,        0, 1,          0,
      //                 0,        0, 0,          1;

      // Hr_[3] << cos(state_.q(3)), 0,  sin(state_.q(3)),  0,
      //           sin(state_.q(3)), 0, -cos(state_.q(3)),  0,
      //                 0, 1,        0, ps_.l(3),
      //                 0, 0,        0,  1;

      // Hr_[4] << cos(state_.q(4)),  0, -sin(state_.q(4)),  0,
      //           sin(state_.q(4)),  0,  cos(state_.q(4)),  0,
      //                 0, -1,        0, ps_.l(4),
      //                 0,  0,        0,  1;

      // Hr_[5] << cos(state_.q(5)), -sin(state_.q(5)), 0,  0,
      //           sin(state_.q(5)),  cos(state_.q(5)), 0,  0,
      //                 0,        0, 1, ps_.l(5),
      //                 0,        0, 0,  1;
      // H_0_[0] = Hr_[0];
      // for (int i = 1; i < DOF; ++i)
      // {
      //   H_0_[i] = H_0_[i-1] * Hr_[i];
      // }
      auto Jef = computeEEJacobian();
      auto JefInv = Jef.inverse();
      // ROS_INFO_STREAM("JefInv:\n" << Jef);
      auto stop = high_resolution_clock::now();
      auto duration = duration_cast<nanoseconds>(stop - start);
      std::cout << duration.count() << std::endl;
      return H_0_;
    }

    std::vector<Matrix4d> UR10Model::computeFKCoMs()
    {
      UR10Model::computeFKCoM0();
      UR10Model::computeFKCoM1();
      UR10Model::computeFKCoM2();
      UR10Model::computeFKCoM3();
      UR10Model::computeFKCoM4();
      UR10Model::computeFKCoM5();

      return Hcm_0_;
    }

    void UR10Model::computeJacobian()
    {
      Vector3d z(0, 0, 1);
      Eigen::Matrix<double,3,6> z_0;
      z_0.block<3,1>(0,0) << 0, 0, 1;

      for (int i = 1; i < DOF; ++i)
      {
        z_0.block<3,1>(0,i) = H_0_[i-1].block<3,3>(0,0) * z;
      }

      Eigen::Matrix<double,3,7> t_0;
      t_0.block<3,1>(0,0) << 0, 0, 0;

      for (int i = 0; i < DOF; ++i)
      {
        z_0.block<3,1>(0,i+1) = H_0_[i].block<3,1>(0,3);
      }

      Matrix6d J = Matrix6d::Zero();
      J.block<3,1>(0,0) = z_0.block<3,1>(0,0).cross(t_0.block<3,1>(0,1)-t_0.block<3,1>(0,0));
      J.block<3,1>(3,0) = z_0.block<3,1>(0,0);

      ROS_INFO_STREAM("z_0:\n" << z_0);
      ROS_INFO_STREAM("Jacobian1:\n" << J);
    }

    void UR10Model::computeDynamic()
    {

    }

    void UR10Model::computeInertialMatrix()
    {
      Matrix6d M = Matrix6d::Zero();
      
      M_ = M;
    }

    void UR10Model::computeCoriolisMatrix()
    {

    }

    Vector6d UR10Model::computeGeneralizedGravity()
    {
      Vector6d G = Vector6d::Zero();
      G(0) = 0;
      G(1) = g*(ps_.l(1)*ps_.m(1)*cos(state_.q(1)) + ps_.l(1)*ps_.m(2)*cos(state_.q(1)) + ps_.l(1)*ps_.m(3)*cos(state_.q(1)) + ps_.l(1)*ps_.m(4)*cos(state_.q(1)) + ps_.l(1)*ps_.m(5)*cos(state_.q(1)) + ps_.m(1)*ps_.tcm(1,0)*cos(state_.q(1)) - ps_.m(1)*ps_.tcm(1,1)*sin(state_.q(1)) + ps_.l(4)*ps_.m(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(4)*ps_.m(5)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm(3,0)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm(3,2)*sin(state_.q(1) + state_.q(2) + state_.q(3)) - ps_.m(4)*ps_.tcm(4,1)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(2)*ps_.m(2)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(3)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(4)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(5)*cos(state_.q(1) + state_.q(2)) + ps_.m(2)*ps_.tcm(2,0)*cos(state_.q(1) + state_.q(2)) - ps_.m(2)*ps_.tcm(2,1)*sin(state_.q(1) + state_.q(2)) - ps_.l(5)*ps_.m(5)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) + ps_.m(4)*ps_.tcm(4,0)*cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) - ps_.m(4)*ps_.tcm(4,2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm(5,2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) - ps_.m(5)*ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) + ps_.m(5)*ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) + ps_.m(5)*ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)));
      G(2) = g*(ps_.l(4)*ps_.m(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(4)*ps_.m(5)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm(3,0)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm(3,2)*sin(state_.q(1) + state_.q(2) + state_.q(3)) - ps_.m(4)*ps_.tcm(4,1)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(2)*ps_.m(2)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(3)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(4)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(5)*cos(state_.q(1) + state_.q(2)) + ps_.m(2)*ps_.tcm(2,0)*cos(state_.q(1) + state_.q(2)) - ps_.m(2)*ps_.tcm(2,1)*sin(state_.q(1) + state_.q(2)) - ps_.l(5)*ps_.m(5)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) + ps_.m(4)*ps_.tcm(4,0)*cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) - ps_.m(4)*ps_.tcm(4,2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm(5,2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) - ps_.m(5)*ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) + ps_.m(5)*ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) + ps_.m(5)*ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)));
      G(3) = g*ps_.m(4)*(ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) - ps_.tcm(4,1)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.tcm(4,0)*cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) - ps_.tcm(4,2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4))) - g*ps_.m(5)*(ps_.l(5)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.tcm(5,2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) + ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) + ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) + ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) + ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) + ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)) + ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) - ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)) - ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5))) + g*ps_.m(3)*(ps_.tcm(3,0)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.tcm(3,2)*sin(state_.q(1) + state_.q(2) + state_.q(3)));
      G(4) = -g*ps_.m(5)*(ps_.l(5)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) + ps_.tcm(5,2)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) + ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3))*sin(state_.q(4)) + ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5))*sin(state_.q(4)) - ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(4))*sin(state_.q(5)) - ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(4))*sin(state_.q(5))) - g*ps_.m(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*(ps_.tcm(4,2)*cos(state_.q(4)) + ps_.tcm(4,0)*sin(state_.q(4)));
      G(5) = -g*ps_.m(5)*(ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) - ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) + ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) - ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) + ps_.tcm(5,1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) + ps_.tcm(5,1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5)) + ps_.tcm(5,0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)) + ps_.tcm(5,0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)));
      G_ = G;
      return G_;
    }

    void UR10Model::computeAllTerms()
    {
    }

    void UR10Model::computeFKJoint0()
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(state_.q(0));
      H(0, 1) = 0;
      H(0, 2) = sin(state_.q(0));
      H(0, 3) = 0;
      H(1, 0) = sin(state_.q(0));
      H(1, 1) = 0;
      H(1, 2) = -cos(state_.q(0));
      H(1, 3) = 0;
      H(2, 0) = 0;
      H(2, 1) = 1;
      H(2, 2) = 0;
      H(2, 3) = ps_.l(0);
      H_0_[0] = H;
    }

    void UR10Model::computeFKJoint1()
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(state_.q(0))*cos(state_.q(1));
      H(0, 1) = -cos(state_.q(0))*sin(state_.q(1));
      H(0, 2) = sin(state_.q(0));
      H(0, 3) = ps_.l(1)*cos(state_.q(0))*cos(state_.q(1));
      H(1, 0) = cos(state_.q(1))*sin(state_.q(0));
      H(1, 1) = -sin(state_.q(0))*sin(state_.q(1));
      H(1, 2) = -cos(state_.q(0));
      H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0));
      H(2, 0) = sin(state_.q(1));
      H(2, 1) = cos(state_.q(1));
      H(2, 2) = 0;
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1));
      H_0_[1] = H;
    }

    void UR10Model::computeFKJoint2()
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(0, 1) = - cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) - cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1));
      H(0, 2) = sin(state_.q(0));
      H(0, 3) = ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 0) = cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 1) = - cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) - cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1));
      H(1, 2) = -cos(state_.q(0));
      H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 0) = cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1));
      H(2, 1) = cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2));
      H(2, 2) = 0;
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      H_0_[2] = H;
    }

    void UR10Model::computeFKJoint3()
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1)));
      H(0, 1) = sin(state_.q(0));
      H(0, 2) = cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)));
      H(0, 3) = ps_.l(3)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 0) = - cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1)));
      H(1, 1) = -cos(state_.q(0));
      H(1, 2) = cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)));
      H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.l(3)*cos(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 0) = cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)));
      H(2, 1) = 0;
      H(2, 2) = sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) - cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)));
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      H_0_[3] = H;
    }

    void UR10Model::computeFKJoint4()
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))));
      H(0, 1) = - cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)));
      H(0, 2) = cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))));
      H(0, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 0) = - cos(state_.q(0))*sin(state_.q(4)) - cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))));
      H(1, 1) = sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) - cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1)));
      H(1, 2) = sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1)))) - cos(state_.q(0))*cos(state_.q(4));
      H(1, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.l(3)*cos(state_.q(0)) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 0) = cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      H(2, 1) = cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)));
      H(2, 2) = -sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      H_0_[4] = H;
    }

    void UR10Model::computeFKJoint5()
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(state_.q(5))*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) - sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))));
      H(0, 1) = - sin(state_.q(5))*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) - cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))));
      H(0, 2) = cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))));
      H(0, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 0) = - cos(state_.q(5))*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))));
      H(1, 1) = sin(state_.q(5))*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))));
      H(1, 2) = sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1)))) - cos(state_.q(0))*cos(state_.q(4));
      H(1, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.l(3)*cos(state_.q(0)) - ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 0) = sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + cos(state_.q(4))*cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      H(2, 1) = cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) - cos(state_.q(4))*sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      H(2, 2) = -sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1)) - ps_.l(5)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      H_0_[5] = H;
    }

    void UR10Model::computeFKCoM0()
    {
      // Matrix4d H = H_0_[0];
      // H(0, 3) = ps_.tcm(0,0)*cos(state_.q(0)) + ps_.tcm(0,2)*sin(state_.q(0));
      // H(1, 3) = ps_.tcm(0,0)*sin(state_.q(0)) - ps_.tcm(0,2)*cos(state_.q(0));
      // H(2, 3) = ps_.l(0) + ps_.tcm(0,1);
      // Hcm_0_[0] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = ps_.tcm(0, 0);
      H(1, 3) = ps_.tcm(0, 1);
      H(2, 3) = ps_.tcm(0, 2);
      Hcm_0_[0] = H_0_[0] * H;
    }

    void UR10Model::computeFKCoM1()
    {
      // Matrix4d H = H_0_[1];
      // H(0, 3) = ps_.tcm(1,2)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.tcm(1,0)*cos(state_.q(0))*cos(state_.q(1)) - ps_.tcm(1,1)*cos(state_.q(0))*sin(state_.q(1));
      // H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.tcm(1,2)*cos(state_.q(0)) + ps_.tcm(1,0)*cos(state_.q(1))*sin(state_.q(0)) - ps_.tcm(1,1)*sin(state_.q(0))*sin(state_.q(1));
      // H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) + ps_.tcm(1,1)*cos(state_.q(1)) + ps_.tcm(1,0)*sin(state_.q(1));
      // Hcm_0_[1] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = ps_.tcm(1, 0);
      H(1, 3) = ps_.tcm(1, 1);
      H(2, 3) = ps_.tcm(1, 2);
      Hcm_0_[1] = H_0_[1] * H;
    }

    void UR10Model::computeFKCoM2()
    {
      // Matrix4d H = H_0_[2];
      // H(0, 3) = ps_.tcm(2,0)*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - ps_.tcm(2,1)*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + ps_.tcm(2,2)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.tcm(2,1)*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - ps_.tcm(2,2)*cos(state_.q(0)) - ps_.tcm(2,0)*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(2, 3) = ps_.l(0) + ps_.tcm(2,0)*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + ps_.tcm(2,1)*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) + ps_.l(1)*sin(state_.q(1)) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      // Hcm_0_[2] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = ps_.tcm(2, 0);
      H(1, 3) = ps_.tcm(2, 1);
      H(2, 3) = ps_.tcm(2, 2);
      Hcm_0_[2] = H_0_[2] * H;
    }

    void UR10Model::computeFKCoM3()
    {
      // Matrix4d H = H_0_[3];
      // H(0, 3) = ps_.tcm(3,0)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1)))) + ps_.tcm(3,2)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.tcm(3,1)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(1, 3) = ps_.tcm(3,2)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.tcm(3,0)*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1)))) - ps_.l(3)*cos(state_.q(0)) - ps_.tcm(3,1)*cos(state_.q(0)) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) + ps_.tcm(3,0)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm(3,2)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      // Hcm_0_[3] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = ps_.tcm(3, 0);
      H(1, 3) = ps_.tcm(3, 1);
      H(2, 3) = ps_.tcm(3, 2);
      Hcm_0_[3] = H_0_[3] * H;
    }

    void UR10Model::computeFKCoM4()
    {
      // Matrix4d H = H_0_[4];
      // H(0, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm(4,1)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.tcm(4,0)*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.tcm(4,2)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(1, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.tcm(4,2)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - ps_.tcm(4,0)*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - ps_.tcm(4,1)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.l(3)*cos(state_.q(0)) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.tcm(4,1)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1)) + ps_.tcm(4,0)*cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm(4,2)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      // Hcm_0_[4] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = ps_.tcm(4, 0);
      H(1, 3) = ps_.tcm(4, 1);
      H(2, 3) = ps_.tcm(4, 2);
      Hcm_0_[4] = H_0_[4] * H;
    }

    void UR10Model::computeFKCoM5()
    {
      // Matrix4d H = H_0_[5];
      // H(0, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.tcm(5,2)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.tcm(5,0)*(cos(state_.q(5))*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) - sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))))) - ps_.tcm(5,1)*(sin(state_.q(5))*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))))) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(1, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.tcm(5,2)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - ps_.tcm(5,0)*(cos(state_.q(5))*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) + sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))))) + ps_.tcm(5,1)*(sin(state_.q(5))*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))))) - ps_.l(3)*cos(state_.q(0)) - ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      // H(2, 3) = ps_.l(0) + ps_.tcm(5,0)*(sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + cos(state_.q(4))*cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))))) + ps_.tcm(5,1)*(cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) - cos(state_.q(4))*sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))))) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1)) - ps_.l(5)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm(5,2)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      // Hcm_0_[5] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = ps_.tcm(5, 0);
      H(1, 3) = ps_.tcm(5, 1);
      H(2, 3) = ps_.tcm(5, 2);
      Hcm_0_[5] = H_0_[5] * H;
    }

    Matrix6d UR10Model::computeEEJacobian()
    {
      Jef_0_(0,0) = ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*cos(state_.q(0)) - ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0)) - ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) + ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      Jef_0_(0,1) = -cos(state_.q(0))*(ps_.l(2)*sin(state_.q(1) + state_.q(2)) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) - ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(0,2) = cos(state_.q(0))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) - ps_.l(2)*sin(state_.q(1) + state_.q(2)) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(0,3) = cos(state_.q(0))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(0,4) = - cos(state_.q(1) + state_.q(2) + state_.q(3))*(ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))) - sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(0,5) = (cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4)))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) - ps_.l(4)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3)))) - sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4))*(ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) + ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0)));
      Jef_0_(1,0) = ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      Jef_0_(1,1) = -sin(state_.q(0))*(ps_.l(2)*sin(state_.q(1) + state_.q(2)) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) - ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(1,2) = sin(state_.q(0))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) - ps_.l(2)*sin(state_.q(1) + state_.q(2)) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(1,3) = sin(state_.q(0))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3))));
      Jef_0_(1,4) = sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3)))) - cos(state_.q(1) + state_.q(2) + state_.q(3))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)));
      Jef_0_(1,5) = (cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4)))*(ps_.l(4)*(cos(state_.q(1) + state_.q(2))*cos(state_.q(3)) - sin(state_.q(1) + state_.q(2))*sin(state_.q(3))) - ps_.l(4)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(5)*sin(state_.q(4))*(cos(state_.q(1) + state_.q(2))*sin(state_.q(3)) + sin(state_.q(1) + state_.q(2))*cos(state_.q(3)))) - sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)));
      Jef_0_(2,0) = 0;
      Jef_0_(2,1) = cos(state_.q(0))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(0))*(ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*cos(state_.q(0)) - ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0)) - ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) + ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)));
      Jef_0_(2,2) = cos(state_.q(0))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(0))*(ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*cos(state_.q(0)) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0)) - ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) + ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)));
      Jef_0_(2,3) = cos(state_.q(0))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) - cos(state_.q(0))*(ps_.l(2)*cos(state_.q(1) + state_.q(2)) + ps_.l(1)*cos(state_.q(1))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(0))*(ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) + ps_.l(3)*cos(state_.q(0)) + sin(state_.q(0))*(ps_.l(2)*cos(state_.q(1) + state_.q(2)) + ps_.l(1)*cos(state_.q(1))) - ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0)) - ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) + ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)));
      Jef_0_(2,4) = - sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*(ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))) - sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0)));
      Jef_0_(2,5) = (cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4)))*(ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4))) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))) - (cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4)))*(ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) + ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) + cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4))) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0)));
      Jef_0_(3,0) = 0;
      Jef_0_(3,1) = sin(state_.q(0));
      Jef_0_(3,2) = sin(state_.q(0));
      Jef_0_(3,3) = sin(state_.q(0));
      Jef_0_(3,4) = sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0));
      Jef_0_(3,5) = cos(state_.q(4))*sin(state_.q(0)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(0))*sin(state_.q(4));
      Jef_0_(4,0) = 0;
      Jef_0_(4,1) = -cos(state_.q(0));
      Jef_0_(4,2) = -cos(state_.q(0));
      Jef_0_(4,3) = -cos(state_.q(0));
      Jef_0_(4,4) = sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0));
      Jef_0_(4,5) = - cos(state_.q(0))*cos(state_.q(4)) - cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(0))*sin(state_.q(4));
      Jef_0_(5,0) = 1;
      Jef_0_(5,1) = 0;
      Jef_0_(5,2) = 0;
      Jef_0_(5,3) = 0;
      Jef_0_(5,4) = -cos(state_.q(1) + state_.q(2) + state_.q(3));
      Jef_0_(5,5) = -sin(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4));

      return Jef_0_;
    }

    Matrix4d UR10Model::relativeTrans(const double theta, const double d, const double a, const double alpha)
    {
      Matrix4d Hz = Matrix4d::Identity();
      Hz.block<3,3>(0,0) = rotZ(theta);
      Hz(2,3) = d;

      Matrix4d Hx = Matrix4d::Identity();
      Hx.block<3,3>(0,0) = rotX(alpha);
      Hx(0,3) = a;

      return Hz * Hx;
    }

    Matrix3d UR10Model::rotX(const double alphaX)
    {
      Matrix3d Rx;
      Rx << 1., 0., 0., 0., cos(alphaX), -sin(alphaX), 0., sin(alphaX), cos(alphaX);
      return Rx;
    }

    Matrix3d UR10Model::rotY(const double betaY)
    {
      Matrix3d Ry;
      Ry << cos(betaY), 0., sin(betaY), 0., 1., 0., -sin(betaY), 0., cos(betaY);
      return Ry;
    }

    Matrix3d UR10Model::rotZ(const double gammaZ)
    {
      Matrix3d Rz;
      Rz << cos(gammaZ), -sin(gammaZ), 0., sin(gammaZ), cos(gammaZ), 0., 0., 0., 1.;
      return Rz;
    }
  }
}