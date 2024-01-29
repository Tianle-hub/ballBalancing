#include <tum_ics_ur10_controller_tutorial/UR10Model.h>

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
      H_0_.resize(n);
      Hcm_0_.resize(n);
      J_0_.resize(n);
      ps_.l << 0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922;
      ps_.m << 7.1, 12.7, 4.27, 2, 2, 0.365;
      // ps_.m << 7.778, 12.93, 3.87, 1.96, 1.96, 0.202;
      ps_.I0 << 0.03408, 0.00002, -0.00425, 0.03529, 0.00008, 0.02156;
      ps_.I1 << 0.02814, 0.00005, -0.01561, 0.77068, 0.00002, 0.76943;
      ps_.I2 << 0.01014, 0.00008, 0.00916, 0.30928, 0.00000, 0.30646;
      ps_.I3 << 0.00296, -0.00001, -0.00000, 0.00222, -0.00024, 0.00258;
      ps_.I4 << 0.00296, -0.00001, -0.00000, 0.00222, -0.00024, 0.00258;
      ps_.I5 << 0.00040, 0.00000,  -0.00000, 0.00041, 0.00000, 0.00034;
      ps_.tcm0 << 0.021, 0.000, 0.027;
      ps_.tcm1 << 0.38, 0.000, 0.158;
      ps_.tcm2 << 0.24, 0.000, 0.068;
      ps_.tcm3 << 0.000, 0.007, 0.018;
      ps_.tcm4 << 0.000, 0.007, 0.018;
      ps_.tcm5 << 0, 0, -0.026;
      g = -9.82;
    }

    void UR10Model::updateJointState(const JointState &state_new)
    {
      state_ = state_new;
    }

    std::vector<Matrix4d> UR10Model::computeForwardKinematics()
    {
      UR10Model::computeFKJoint0();
      UR10Model::computeFKJoint1();
      UR10Model::computeFKJoint2();
      UR10Model::computeFKJoint3();
      UR10Model::computeFKJoint4();
      UR10Model::computeFKJoint5();

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
      G(1) = g*(ps_.l(1)*ps_.m(1)*cos(state_.q(1)) + ps_.l(1)*ps_.m(2)*cos(state_.q(1)) + ps_.l(1)*ps_.m(3)*cos(state_.q(1)) + ps_.l(1)*ps_.m(4)*cos(state_.q(1)) + ps_.l(1)*ps_.m(5)*cos(state_.q(1)) + ps_.m(1)*ps_.tcm1(0)*cos(state_.q(1)) - ps_.m(1)*ps_.tcm1(1)*sin(state_.q(1)) + ps_.l(4)*ps_.m(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(4)*ps_.m(5)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm3(0)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm3(2)*sin(state_.q(1) + state_.q(2) + state_.q(3)) - ps_.m(4)*ps_.tcm4(1)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(2)*ps_.m(2)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(3)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(4)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(5)*cos(state_.q(1) + state_.q(2)) + ps_.m(2)*ps_.tcm2(0)*cos(state_.q(1) + state_.q(2)) - ps_.m(2)*ps_.tcm2(1)*sin(state_.q(1) + state_.q(2)) - ps_.l(5)*ps_.m(5)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) + ps_.m(4)*ps_.tcm4(0)*cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) - ps_.m(4)*ps_.tcm4(2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm5(2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) - ps_.m(5)*ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) + ps_.m(5)*ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) + ps_.m(5)*ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)));
      G(2) = g*(ps_.l(4)*ps_.m(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(4)*ps_.m(5)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm3(0)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.m(3)*ps_.tcm3(2)*sin(state_.q(1) + state_.q(2) + state_.q(3)) - ps_.m(4)*ps_.tcm4(1)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.l(2)*ps_.m(2)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(3)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(4)*cos(state_.q(1) + state_.q(2)) + ps_.l(2)*ps_.m(5)*cos(state_.q(1) + state_.q(2)) + ps_.m(2)*ps_.tcm2(0)*cos(state_.q(1) + state_.q(2)) - ps_.m(2)*ps_.tcm2(1)*sin(state_.q(1) + state_.q(2)) - ps_.l(5)*ps_.m(5)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) + ps_.m(4)*ps_.tcm4(0)*cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) - ps_.m(4)*ps_.tcm4(2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm5(2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.m(5)*ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) - ps_.m(5)*ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) + ps_.m(5)*ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5)) - ps_.m(5)*ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)) - ps_.m(5)*ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) + ps_.m(5)*ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)));
      G(3) = g*ps_.m(4)*(ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) - ps_.tcm4(1)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.tcm4(0)*cos(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) - ps_.tcm4(2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4))) - g*ps_.m(5)*(ps_.l(5)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) - ps_.l(4)*sin(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.tcm5(2)*cos(state_.q(1) + state_.q(2) + state_.q(3))*sin(state_.q(4)) + ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) + ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) + ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) + ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) + ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)) + ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) - ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)) - ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5))) + g*ps_.m(3)*(ps_.tcm3(0)*cos(state_.q(1) + state_.q(2) + state_.q(3)) + ps_.tcm3(2)*sin(state_.q(1) + state_.q(2) + state_.q(3)));
      G(4) = -g*ps_.m(5)*(ps_.l(5)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) + ps_.tcm5(2)*sin(state_.q(1) + state_.q(2) + state_.q(3))*cos(state_.q(4)) + ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3))*sin(state_.q(4)) + ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5))*sin(state_.q(4)) - ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(4))*sin(state_.q(5)) - ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(4))*sin(state_.q(5))) - g*ps_.m(4)*sin(state_.q(1) + state_.q(2) + state_.q(3))*(ps_.tcm4(2)*cos(state_.q(4)) + ps_.tcm4(0)*sin(state_.q(4)));
      G(5) = -g*ps_.m(5)*(ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*sin(state_.q(5)) - ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(5)) + ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(5))*sin(state_.q(3)) - ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*sin(state_.q(3))*sin(state_.q(5)) + ps_.tcm5(1)*cos(state_.q(1) + state_.q(2))*cos(state_.q(4))*cos(state_.q(5))*sin(state_.q(3)) + ps_.tcm5(1)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*cos(state_.q(5)) + ps_.tcm5(0)*cos(state_.q(1) + state_.q(2))*cos(state_.q(4))*sin(state_.q(3))*sin(state_.q(5)) + ps_.tcm5(0)*sin(state_.q(1) + state_.q(2))*cos(state_.q(3))*cos(state_.q(4))*sin(state_.q(5)));
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
      Matrix4d H = H_0_[0];
      H(0, 3) = ps_.tcm0(0)*cos(state_.q(0)) + ps_.tcm0(2)*sin(state_.q(0));
      H(1, 3) = ps_.tcm0(0)*sin(state_.q(0)) - ps_.tcm0(2)*cos(state_.q(0));
      H(2, 3) = ps_.l(0) + ps_.tcm0(1);
      Hcm_0_[0] = H;
    }

    void UR10Model::computeFKCoM1()
    {
      Matrix4d H = H_0_[1];
      H(0, 3) = ps_.tcm1(2)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.tcm1(0)*cos(state_.q(0))*cos(state_.q(1)) - ps_.tcm1(1)*cos(state_.q(0))*sin(state_.q(1));
      H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.tcm1(2)*cos(state_.q(0)) + ps_.tcm1(0)*cos(state_.q(1))*sin(state_.q(0)) - ps_.tcm1(1)*sin(state_.q(0))*sin(state_.q(1));
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) + ps_.tcm1(1)*cos(state_.q(1)) + ps_.tcm1(0)*sin(state_.q(1));
      Hcm_0_[1] = H;
    }

    void UR10Model::computeFKCoM2()
    {
      Matrix4d H = H_0_[2];
      H(0, 3) = ps_.tcm2(0)*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - ps_.tcm2(1)*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + ps_.tcm2(2)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 3) = ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) - ps_.tcm2(1)*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - ps_.tcm2(2)*cos(state_.q(0)) - ps_.tcm2(0)*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 3) = ps_.l(0) + ps_.tcm2(0)*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + ps_.tcm2(1)*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) + ps_.l(1)*sin(state_.q(1)) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      Hcm_0_[2] = H;
    }

    void UR10Model::computeFKCoM3()
    {
      Matrix4d H = H_0_[3];
      H(0, 3) = ps_.tcm3(0)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1)))) + ps_.tcm3(2)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.tcm3(1)*sin(state_.q(0)) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 3) = ps_.tcm3(2)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.tcm3(0)*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1)))) - ps_.l(3)*cos(state_.q(0)) - ps_.tcm3(1)*cos(state_.q(0)) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) + ps_.tcm3(0)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm3(2)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1));
      Hcm_0_[3] = H;
    }

    void UR10Model::computeFKCoM4()
    {
      Matrix4d H = H_0_[4];
      H(0, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm4(1)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.tcm4(0)*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.tcm4(2)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.tcm4(2)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - ps_.tcm4(0)*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - ps_.tcm4(1)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.l(3)*cos(state_.q(0)) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 3) = ps_.l(0) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.tcm4(1)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1)) + ps_.tcm4(0)*cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm4(2)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      Hcm_0_[4] = H;
    }

    void UR10Model::computeFKCoM5()
    {
      Matrix4d H = H_0_[5];
      H(0, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)))) + ps_.l(3)*sin(state_.q(0)) + ps_.l(5)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.tcm5(2)*(cos(state_.q(4))*sin(state_.q(0)) - sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + ps_.tcm5(0)*(cos(state_.q(5))*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) - sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))))) - ps_.tcm5(1)*(sin(state_.q(5))*(sin(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))))) + cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(0))*cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2))))) + ps_.l(1)*cos(state_.q(0))*cos(state_.q(1)) + ps_.l(2)*cos(state_.q(0))*cos(state_.q(1))*cos(state_.q(2)) - ps_.l(2)*cos(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(1, 3) = ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)))) - ps_.tcm5(2)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - ps_.tcm5(0)*(cos(state_.q(5))*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) + sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))))) + ps_.tcm5(1)*(sin(state_.q(5))*(cos(state_.q(0))*sin(state_.q(4)) + cos(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) - cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))) - sin(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))))) - ps_.l(3)*cos(state_.q(0)) - ps_.l(5)*(cos(state_.q(0))*cos(state_.q(4)) - sin(state_.q(4))*(cos(state_.q(3))*(sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2)) - cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0))) + sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(0))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(0))*sin(state_.q(1))))) + ps_.l(1)*cos(state_.q(1))*sin(state_.q(0)) + ps_.l(2)*cos(state_.q(1))*cos(state_.q(2))*sin(state_.q(0)) - ps_.l(2)*sin(state_.q(0))*sin(state_.q(1))*sin(state_.q(2));
      H(2, 3) = ps_.l(0) + ps_.tcm5(0)*(sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + cos(state_.q(4))*cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))))) + ps_.tcm5(1)*(cos(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) - cos(state_.q(4))*sin(state_.q(5))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))))) + ps_.l(1)*sin(state_.q(1)) - ps_.l(4)*(cos(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))) - sin(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1)))) + ps_.l(2)*cos(state_.q(1))*sin(state_.q(2)) + ps_.l(2)*cos(state_.q(2))*sin(state_.q(1)) - ps_.l(5)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2)))) - ps_.tcm5(2)*sin(state_.q(4))*(cos(state_.q(3))*(cos(state_.q(1))*sin(state_.q(2)) + cos(state_.q(2))*sin(state_.q(1))) + sin(state_.q(3))*(cos(state_.q(1))*cos(state_.q(2)) - sin(state_.q(1))*sin(state_.q(2))));
      Hcm_0_[5] = H;
    }
  }
}