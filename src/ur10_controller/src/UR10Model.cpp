#include <ur10_controller/UR10Model.h>

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

    void UR10Model::initModel()
    {
      // TODO: put parameters in yaml 
      H_0_.resize(STD_DOF);
      // Hr_.resize(STD_DOF);
      Hcm_0_.resize(STD_DOF);
      J_0_.resize(STD_DOF);
      l << 0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922 + 0.083; // end effector
      m << 7.1, 12.7, 4.27, 2, 2, 0.365;
      // m << 7.778, 12.93, 3.87, 1.96, 1.96, 0.202;
      I << 0.03408, 0.00002, -0.00425, 0.03529, 0.00008, 0.02156,
            0.02814, 0.00005, -0.01561, 0.77068, 0.00002, 0.76943,
            0.01014, 0.00008, 0.00916, 0.30928, 0.00000, 0.30646,
            0.00296, -0.00001, -0.00000, 0.00222, -0.00024, 0.00258,
            0.00296, -0.00001, -0.00000, 0.00222, -0.00024, 0.00258,
            0.00040, 0.00000,  -0.00000, 0.00041, 0.00000, 0.00034;
      tcm << 0.021, 0.000, 0.027,
              0.38, 0.000, 0.158,
              0.24, 0.000, 0.068,
              0.000, 0.007, 0.018,
              0.000, 0.007, 0.018,
              0, 0, -0.026;
      g = -9.81;
      d << l(0), 0., 0., l(3), l(4), l(5);
      a << 0., l(1), l(2), 0., 0., 0.;
      alpha << M_PI/2, 0., 0., M_PI/2, -M_PI/2, 0;
      initTheta();
      Gamma_inv_  = 5 * Theta_.asDiagonal();
      T0_W_ = Matrix4d::Identity();
      T0_W_.block<3,3>(0,0) = AngleAxisd(M_PI, Vector3d::UnitZ()).matrix();
    }

    void UR10Model::updateJointState(const JointState &state_new)
    {
      state_ = state_new;
    }

    std::vector<Matrix4d> UR10Model::computeForwardKinematics(const Vector6d &q)
    {
      H_0_[0] = relativeTrans(q(0), d(0), a(0), alpha(0));

      for(int i = 1; i < STD_DOF; ++i)
      {
        H_0_[i] = H_0_[i-1] * relativeTrans(q(i), d(i), a(i), alpha(i));
      }

      return H_0_;
    }

    void UR10Model::updateTheta(const Vector6d &q, const Vector6d &qp, const Vector6d &qpr, const Vector6d &qppr, const Vector6d &Sq)
    {
      auto Yr = computeRefRegressor(q, qp, qpr, qppr);
      Matrix6d I = Matrix6d::Identity();
      I(3,3) = -1;
      Theta_ = Theta_  - Gamma_inv_ * Yr.transpose() * I * Sq * 0.002;
    }

    std::vector<Matrix4d> UR10Model::computeFKCoMs(const Vector6d &q)
    {
      computeForwardKinematics(q);
      
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
      // TODO: ckeck results
      Vector3d z(0, 0, 1);
      Eigen::Matrix<double,3,6> z_0;
      z_0.block<3,1>(0,0) << 0, 0, 1;

      for (int i = 1; i < STD_DOF; ++i)
      {
        z_0.block<3,1>(0,i) = H_0_[i-1].block<3,3>(0,0) * z;
      }

      Eigen::Matrix<double,3,7> t_0;
      t_0.block<3,1>(0,0) << 0, 0, 0;

      for (int i = 0; i < STD_DOF; ++i)
      {
        z_0.block<3,1>(0,i+1) = H_0_[i].block<3,1>(0,3);
      }

      Matrix6d J = Matrix6d::Zero();
      J.block<3,1>(0,0) = z_0.block<3,1>(0,0).cross(t_0.block<3,1>(0,1)-t_0.block<3,1>(0,0));
      J.block<3,1>(3,0) = z_0.block<3,1>(0,0);
    }

    Vector6d UR10Model::computeGeneralizedGravity(const Vector6d &q)
    {
      G_(0) = 0;
      G_(1) = g*(l(1)*m(1)*cos(q(1)) + l(1)*m(2)*cos(q(1)) + l(1)*m(3)*cos(q(1)) + l(1)*m(4)*cos(q(1)) + l(1)*m(5)*cos(q(1)) + m(1)*tcm(1,0)*cos(q(1)) - m(1)*tcm(1,1)*sin(q(1)) + l(4)*m(4)*sin(q(1) + q(2) + q(3)) + l(4)*m(5)*sin(q(1) + q(2) + q(3)) + m(3)*tcm(3,0)*cos(q(1) + q(2) + q(3)) + m(3)*tcm(3,2)*sin(q(1) + q(2) + q(3)) - m(4)*tcm(4,1)*sin(q(1) + q(2) + q(3)) + l(2)*m(2)*cos(q(1) + q(2)) + l(2)*m(3)*cos(q(1) + q(2)) + l(2)*m(4)*cos(q(1) + q(2)) + l(2)*m(5)*cos(q(1) + q(2)) + m(2)*tcm(2,0)*cos(q(1) + q(2)) - m(2)*tcm(2,1)*sin(q(1) + q(2)) - l(5)*m(5)*cos(q(1) + q(2) + q(3))*sin(q(4)) + m(4)*tcm(4,0)*cos(q(1) + q(2) + q(3))*cos(q(4)) - m(4)*tcm(4,2)*cos(q(1) + q(2) + q(3))*sin(q(4)) - m(5)*tcm(5,2)*cos(q(1) + q(2) + q(3))*sin(q(4)) - m(5)*tcm(5,1)*cos(q(1) + q(2))*cos(q(5))*sin(q(3)) - m(5)*tcm(5,1)*sin(q(1) + q(2))*cos(q(3))*cos(q(5)) - m(5)*tcm(5,0)*cos(q(1) + q(2))*sin(q(3))*sin(q(5)) - m(5)*tcm(5,0)*sin(q(1) + q(2))*cos(q(3))*sin(q(5)) + m(5)*tcm(5,0)*cos(q(1) + q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - m(5)*tcm(5,1)*cos(q(1) + q(2))*cos(q(3))*cos(q(4))*sin(q(5)) - m(5)*tcm(5,0)*sin(q(1) + q(2))*cos(q(4))*cos(q(5))*sin(q(3)) + m(5)*tcm(5,1)*sin(q(1) + q(2))*cos(q(4))*sin(q(3))*sin(q(5)));
      G_(2) = g*(l(4)*m(4)*sin(q(1) + q(2) + q(3)) + l(4)*m(5)*sin(q(1) + q(2) + q(3)) + m(3)*tcm(3,0)*cos(q(1) + q(2) + q(3)) + m(3)*tcm(3,2)*sin(q(1) + q(2) + q(3)) - m(4)*tcm(4,1)*sin(q(1) + q(2) + q(3)) + l(2)*m(2)*cos(q(1) + q(2)) + l(2)*m(3)*cos(q(1) + q(2)) + l(2)*m(4)*cos(q(1) + q(2)) + l(2)*m(5)*cos(q(1) + q(2)) + m(2)*tcm(2,0)*cos(q(1) + q(2)) - m(2)*tcm(2,1)*sin(q(1) + q(2)) - l(5)*m(5)*cos(q(1) + q(2) + q(3))*sin(q(4)) + m(4)*tcm(4,0)*cos(q(1) + q(2) + q(3))*cos(q(4)) - m(4)*tcm(4,2)*cos(q(1) + q(2) + q(3))*sin(q(4)) - m(5)*tcm(5,2)*cos(q(1) + q(2) + q(3))*sin(q(4)) - m(5)*tcm(5,1)*cos(q(1) + q(2))*cos(q(5))*sin(q(3)) - m(5)*tcm(5,1)*sin(q(1) + q(2))*cos(q(3))*cos(q(5)) - m(5)*tcm(5,0)*cos(q(1) + q(2))*sin(q(3))*sin(q(5)) - m(5)*tcm(5,0)*sin(q(1) + q(2))*cos(q(3))*sin(q(5)) + m(5)*tcm(5,0)*cos(q(1) + q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - m(5)*tcm(5,1)*cos(q(1) + q(2))*cos(q(3))*cos(q(4))*sin(q(5)) - m(5)*tcm(5,0)*sin(q(1) + q(2))*cos(q(4))*cos(q(5))*sin(q(3)) + m(5)*tcm(5,1)*sin(q(1) + q(2))*cos(q(4))*sin(q(3))*sin(q(5)));
      G_(3) = g*m(4)*(l(4)*sin(q(1) + q(2) + q(3)) - tcm(4,1)*sin(q(1) + q(2) + q(3)) + tcm(4,0)*cos(q(1) + q(2) + q(3))*cos(q(4)) - tcm(4,2)*cos(q(1) + q(2) + q(3))*sin(q(4))) - g*m(5)*(l(5)*cos(q(1) + q(2) + q(3))*sin(q(4)) - l(4)*sin(q(1) + q(2) + q(3)) + tcm(5,2)*cos(q(1) + q(2) + q(3))*sin(q(4)) + tcm(5,1)*cos(q(1) + q(2))*cos(q(5))*sin(q(3)) + tcm(5,1)*sin(q(1) + q(2))*cos(q(3))*cos(q(5)) + tcm(5,0)*cos(q(1) + q(2))*sin(q(3))*sin(q(5)) + tcm(5,0)*sin(q(1) + q(2))*cos(q(3))*sin(q(5)) + tcm(5,1)*cos(q(1) + q(2))*cos(q(3))*cos(q(4))*sin(q(5)) + tcm(5,0)*sin(q(1) + q(2))*cos(q(4))*cos(q(5))*sin(q(3)) - tcm(5,1)*sin(q(1) + q(2))*cos(q(4))*sin(q(3))*sin(q(5)) - tcm(5,0)*cos(q(1) + q(2))*cos(q(3))*cos(q(4))*cos(q(5))) + g*m(3)*(tcm(3,0)*cos(q(1) + q(2) + q(3)) + tcm(3,2)*sin(q(1) + q(2) + q(3)));
      G_(4) = -g*m(5)*(l(5)*sin(q(1) + q(2) + q(3))*cos(q(4)) + tcm(5,2)*sin(q(1) + q(2) + q(3))*cos(q(4)) + tcm(5,0)*cos(q(1) + q(2))*cos(q(5))*sin(q(3))*sin(q(4)) + tcm(5,0)*sin(q(1) + q(2))*cos(q(3))*cos(q(5))*sin(q(4)) - tcm(5,1)*cos(q(1) + q(2))*sin(q(3))*sin(q(4))*sin(q(5)) - tcm(5,1)*sin(q(1) + q(2))*cos(q(3))*sin(q(4))*sin(q(5))) - g*m(4)*sin(q(1) + q(2) + q(3))*(tcm(4,2)*cos(q(4)) + tcm(4,0)*sin(q(4)));
      G_(5) = -g*m(5)*(tcm(5,1)*cos(q(1) + q(2))*cos(q(3))*sin(q(5)) - tcm(5,0)*cos(q(1) + q(2))*cos(q(3))*cos(q(5)) + tcm(5,0)*sin(q(1) + q(2))*cos(q(5))*sin(q(3)) - tcm(5,1)*sin(q(1) + q(2))*sin(q(3))*sin(q(5)) + tcm(5,1)*cos(q(1) + q(2))*cos(q(4))*cos(q(5))*sin(q(3)) + tcm(5,1)*sin(q(1) + q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + tcm(5,0)*cos(q(1) + q(2))*cos(q(4))*sin(q(3))*sin(q(5)) + tcm(5,0)*sin(q(1) + q(2))*cos(q(3))*cos(q(4))*sin(q(5)));
      return G_;
    }

    void UR10Model::computeAllTerms()
    {
    }

    Matrix4d UR10Model::relativeTrans(const double theta, const double d, const double a, const double alpha)
    {
      Matrix4d Hz = Matrix4d::Identity();
      Hz.block<3,3>(0,0) = Tum::Tools::MathTools::Rz(theta);
      Hz(2,3) = d;

      Matrix4d Hx = Matrix4d::Identity();
      Hx.block<3,3>(0,0) = Tum::Tools::MathTools::Rx(alpha);
      Hx(0,3) = a;

      return Hz * Hx;
    }

    // Matrix3d UR10Model::rotX(const double alphaX)
    // {
    //   Matrix3d Rx;
    //   Rx << 1., 0., 0., 0., cos(alphaX), -sin(alphaX), 0., sin(alphaX), cos(alphaX);
    //   return Rx;
    // }

    // Matrix3d UR10Model::rotY(const double betaY)
    // {
    //   Matrix3d Ry;
    //   Ry << cos(betaY), 0., sin(betaY), 0., 1., 0., -sin(betaY), 0., cos(betaY);
    //   return Ry;
    // }

    // Matrix3d UR10Model::rotZ(const double gammaZ)
    // {
    //   Matrix3d Rz;
    //   Rz << cos(gammaZ), -sin(gammaZ), 0., sin(gammaZ), cos(gammaZ), 0., 0., 0., 1.;
    //   return Rz;
    // }

    Matrix4d UR10Model::computeFKJoint0(const Vector6d &q)
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(q(0));
      H(0, 1) = 0;
      H(0, 2) = sin(q(0));
      H(0, 3) = 0;
      H(1, 0) = sin(q(0));
      H(1, 1) = 0;
      H(1, 2) = -cos(q(0));
      H(1, 3) = 0;
      H(2, 0) = 0;
      H(2, 1) = 1;
      H(2, 2) = 0;
      H(2, 3) = l(0);
      return H;
    }

    Matrix4d UR10Model::computeFKJoint1(const Vector6d &q)
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(q(0))*cos(q(1));
      H(0, 1) = -cos(q(0))*sin(q(1));
      H(0, 2) = sin(q(0));
      H(0, 3) = l(1)*cos(q(0))*cos(q(1));
      H(1, 0) = cos(q(1))*sin(q(0));
      H(1, 1) = -sin(q(0))*sin(q(1));
      H(1, 2) = -cos(q(0));
      H(1, 3) = l(1)*cos(q(1))*sin(q(0));
      H(2, 0) = sin(q(1));
      H(2, 1) = cos(q(1));
      H(2, 2) = 0;
      H(2, 3) = l(0) + l(1)*sin(q(1));
      return H;
    }

    Matrix4d UR10Model::computeFKJoint2(const Vector6d &q)
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2));
      H(0, 1) = - cos(q(0))*cos(q(1))*sin(q(2)) - cos(q(0))*cos(q(2))*sin(q(1));
      H(0, 2) = sin(q(0));
      H(0, 3) = l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      H(1, 0) = cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2));
      H(1, 1) = - cos(q(1))*sin(q(0))*sin(q(2)) - cos(q(2))*sin(q(0))*sin(q(1));
      H(1, 2) = -cos(q(0));
      H(1, 3) = l(1)*cos(q(1))*sin(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      H(2, 0) = cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1));
      H(2, 1) = cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2));
      H(2, 2) = 0;
      H(2, 3) = l(0) + l(1)*sin(q(1)) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1));
      return H;
    }

    Matrix4d UR10Model::computeFKJoint3(const Vector6d &q)
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)));
      H(0, 1) = sin(q(0));
      H(0, 2) = cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)));
      H(0, 3) = l(3)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      H(1, 0) = - cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) - sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)));
      H(1, 1) = -cos(q(0));
      H(1, 2) = cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)));
      H(1, 3) = l(1)*cos(q(1))*sin(q(0)) - l(3)*cos(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      H(2, 0) = cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)));
      H(2, 1) = 0;
      H(2, 2) = sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) - cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)));
      H(2, 3) = l(0) + l(1)*sin(q(1)) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1));
      return H;
    }

    Matrix4d UR10Model::computeFKJoint4(const Vector6d &q)
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
      H(0, 1) = - cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) - sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)));
      H(0, 2) = cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
      H(0, 3) = l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(3)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      H(1, 0) = - cos(q(0))*sin(q(4)) - cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))));
      H(1, 1) = sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) - cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)));
      H(1, 2) = sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - cos(q(0))*cos(q(4));
      H(1, 3) = l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - l(3)*cos(q(0)) + l(1)*cos(q(1))*sin(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      H(2, 0) = cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      H(2, 1) = cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)));
      H(2, 2) = -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      H(2, 3) = l(0) + l(1)*sin(q(1)) - l(4)*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1));
      return H;
    }

    Matrix4d UR10Model::computeFKJoint5(const Vector6d &q)
    {
      Matrix4d H = Matrix4d::Identity();
      H(0, 0) = cos(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))));
      H(0, 1) = - sin(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))));
      H(0, 2) = cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))));
      H(0, 3) = l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(3)*sin(q(0)) + l(5)*(cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      H(1, 0) = - cos(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))));
      H(1, 1) = sin(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))));
      H(1, 2) = sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - cos(q(0))*cos(q(4));
      H(1, 3) = l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - l(3)*cos(q(0)) - l(5)*(cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) + l(1)*cos(q(1))*sin(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      H(2, 0) = sin(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + cos(q(4))*cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      H(2, 1) = cos(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(4))*sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      H(2, 2) = -sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      H(2, 3) = l(0) + l(1)*sin(q(1)) - l(4)*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1)) - l(5)*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      return H;
    }


    Matrix4d UR10Model::computeEEPos(const Vector6d &q)
    {
      Matrix4d H = computeFKJoint5(q);
      // Vector6d X;
      // X << H(0,3), H(1,3), H(2,3);
      // X.block<3,1>(3,0) = H.block<3,3>(0,0).eulerAngles(0,1,2);

      return H;
    }


    void UR10Model::computeFKCoM0()
    {
      // Matrix4d H = H_0_[0];
      // H(0, 3) = tcm(0,0)*cos(q(0)) + tcm(0,2)*sin(q(0));
      // H(1, 3) = tcm(0,0)*sin(q(0)) - tcm(0,2)*cos(q(0));
      // H(2, 3) = l(0) + tcm(0,1);
      // Hcm_0_[0] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = tcm(0, 0);
      H(1, 3) = tcm(0, 1);
      H(2, 3) = tcm(0, 2);
      Hcm_0_[0] = H_0_[0] * H;
    }

    void UR10Model::computeFKCoM1()
    {
      // Matrix4d H = H_0_[1];
      // H(0, 3) = tcm(1,2)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + tcm(1,0)*cos(q(0))*cos(q(1)) - tcm(1,1)*cos(q(0))*sin(q(1));
      // H(1, 3) = l(1)*cos(q(1))*sin(q(0)) - tcm(1,2)*cos(q(0)) + tcm(1,0)*cos(q(1))*sin(q(0)) - tcm(1,1)*sin(q(0))*sin(q(1));
      // H(2, 3) = l(0) + l(1)*sin(q(1)) + tcm(1,1)*cos(q(1)) + tcm(1,0)*sin(q(1));
      // Hcm_0_[1] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = tcm(1, 0);
      H(1, 3) = tcm(1, 1);
      H(2, 3) = tcm(1, 2);
      Hcm_0_[1] = H_0_[1] * H;
    }

    void UR10Model::computeFKCoM2()
    {
      // Matrix4d H = H_0_[2];
      // H(0, 3) = tcm(2,0)*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - tcm(2,1)*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + tcm(2,2)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      // H(1, 3) = l(1)*cos(q(1))*sin(q(0)) - tcm(2,1)*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - tcm(2,2)*cos(q(0)) - tcm(2,0)*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      // H(2, 3) = l(0) + tcm(2,0)*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + tcm(2,1)*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) + l(1)*sin(q(1)) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1));
      // Hcm_0_[2] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = tcm(2, 0);
      H(1, 3) = tcm(2, 1);
      H(2, 3) = tcm(2, 2);
      Hcm_0_[2] = H_0_[2] * H;
    }

    void UR10Model::computeFKCoM3()
    {
      // Matrix4d H = H_0_[3];
      // H(0, 3) = tcm(3,0)*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) + tcm(3,2)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(3)*sin(q(0)) + tcm(3,1)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      // H(1, 3) = tcm(3,2)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - tcm(3,0)*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) - l(3)*cos(q(0)) - tcm(3,1)*cos(q(0)) + l(1)*cos(q(1))*sin(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      // H(2, 3) = l(0) + l(1)*sin(q(1)) + tcm(3,0)*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - tcm(3,2)*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1));
      // Hcm_0_[3] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = tcm(3, 0);
      H(1, 3) = tcm(3, 1);
      H(2, 3) = tcm(3, 2);
      Hcm_0_[3] = H_0_[3] * H;
    }

    void UR10Model::computeFKCoM4()
    {
      // Matrix4d H = H_0_[4];
      // H(0, 3) = l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) - tcm(4,1)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(3)*sin(q(0)) + tcm(4,0)*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + tcm(4,2)*(cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      // H(1, 3) = l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - tcm(4,2)*(cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - tcm(4,0)*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - tcm(4,1)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - l(3)*cos(q(0)) + l(1)*cos(q(1))*sin(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      // H(2, 3) = l(0) + l(1)*sin(q(1)) - l(4)*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + tcm(4,1)*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1)) + tcm(4,0)*cos(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - tcm(4,2)*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      // Hcm_0_[4] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = tcm(4, 0);
      H(1, 3) = tcm(4, 1);
      H(2, 3) = tcm(4, 2);
      Hcm_0_[4] = H_0_[4] * H;
    }

    void UR10Model::computeFKCoM5()
    {
      // Matrix4d H = H_0_[5];
      // H(0, 3) = l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(3)*sin(q(0)) + l(5)*(cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + tcm(5,2)*(cos(q(4))*sin(q(0)) - sin(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + tcm(5,0)*(cos(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) - sin(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))))) - tcm(5,1)*(sin(q(5))*(sin(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))))) + cos(q(5))*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))))) + l(1)*cos(q(0))*cos(q(1)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      // H(1, 3) = l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) - tcm(5,2)*(cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - tcm(5,0)*(cos(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) + sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))))) + tcm(5,1)*(sin(q(5))*(cos(q(0))*sin(q(4)) + cos(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) - cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))))) - l(3)*cos(q(0)) - l(5)*(cos(q(0))*cos(q(4)) - sin(q(4))*(cos(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0))) + sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))))) + l(1)*cos(q(1))*sin(q(0)) + l(2)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      // H(2, 3) = l(0) + tcm(5,0)*(sin(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + cos(q(4))*cos(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))))) + tcm(5,1)*(cos(q(5))*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) - cos(q(4))*sin(q(5))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))))) + l(1)*sin(q(1)) - l(4)*(cos(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))) - sin(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1)))) + l(2)*cos(q(1))*sin(q(2)) + l(2)*cos(q(2))*sin(q(1)) - l(5)*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2)))) - tcm(5,2)*sin(q(4))*(cos(q(3))*(cos(q(1))*sin(q(2)) + cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2)) - sin(q(1))*sin(q(2))));
      // Hcm_0_[5] = H;
      Matrix4d H = Matrix4d::Identity();
      H(0, 3) = tcm(5, 0);
      H(1, 3) = tcm(5, 1);
      H(2, 3) = tcm(5, 2);
      Hcm_0_[5] = H_0_[5] * H;
    }

    Matrix6d UR10Model::computeEEJacobian(const Vector6d &q)
    {

      Matrix6d Jef;
      Jef(0,0) = l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + l(3)*cos(q(0)) - l(1)*cos(q(1))*sin(q(0)) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2));
      Jef(0,1) = -cos(q(0))*(l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(0,2) = cos(q(0))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(0,3) = cos(q(0))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(0,4) = - cos(q(1) + q(2) + q(3))*(l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0))) - sin(q(1) + q(2) + q(3))*sin(q(0))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(0,5) = (cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))) - sin(q(1) + q(2) + q(3))*sin(q(4))*(l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) + l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)));
      Jef(1,0) = l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + l(3)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2));
      Jef(1,1) = -sin(q(0))*(l(2)*sin(q(1) + q(2)) + l(1)*sin(q(1)) - l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(1,2) = sin(q(0))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(1,3) = sin(q(0))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3))));
      Jef(1,4) = sin(q(1) + q(2) + q(3))*cos(q(0))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))) - cos(q(1) + q(2) + q(3))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)));
      Jef(1,5) = (cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)))*(l(4)*(cos(q(1) + q(2))*cos(q(3)) - sin(q(1) + q(2))*sin(q(3))) - l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))) - sin(q(1) + q(2) + q(3))*sin(q(4))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) - l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)));
      Jef(2,0) = 0;
      Jef(2,1) = cos(q(0))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + l(3)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(0))*(l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + l(3)*cos(q(0)) - l(1)*cos(q(1))*sin(q(0)) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2)));
      Jef(2,2) = cos(q(0))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + l(3)*sin(q(0)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(0))*(l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + l(3)*cos(q(0)) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2)));
      Jef(2,3) = cos(q(0))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) - cos(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) + l(3)*sin(q(0)) + l(1)*cos(q(0))*cos(q(1)) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2))) - sin(q(0))*(l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + l(3)*cos(q(0)) + sin(q(0))*(l(2)*cos(q(1) + q(2)) + l(1)*cos(q(1))) - l(1)*cos(q(1))*sin(q(0)) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2)));
      Jef(2,4) = - sin(q(1) + q(2) + q(3))*cos(q(0))*(l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0))) - sin(q(1) + q(2) + q(3))*sin(q(0))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0)));
      Jef(2,5) = (cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) - l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(4)*sin(q(1) + q(2) + q(3))*cos(q(0))) - (cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)))*(l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) - sin(q(3))*(sin(q(0))*sin(q(1))*sin(q(2)) - cos(q(1))*cos(q(2))*sin(q(0)))) + l(5)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) - l(4)*sin(q(1) + q(2) + q(3))*sin(q(0)));
      Jef(3,0) = 0;
      Jef(3,1) = sin(q(0));
      Jef(3,2) = sin(q(0));
      Jef(3,3) = sin(q(0));
      Jef(3,4) = sin(q(1) + q(2) + q(3))*cos(q(0));
      Jef(3,5) = cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4));
      Jef(4,0) = 0;
      Jef(4,1) = -cos(q(0));
      Jef(4,2) = -cos(q(0));
      Jef(4,3) = -cos(q(0));
      Jef(4,4) = sin(q(1) + q(2) + q(3))*sin(q(0));
      Jef(4,5) = - cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4));
      Jef(5,0) = 1;
      Jef(5,1) = 0;
      Jef(5,2) = 0;
      Jef(5,3) = 0;
      Jef(5,4) = -cos(q(1) + q(2) + q(3));
      Jef(5,5) = -sin(q(1) + q(2) + q(3))*sin(q(4));

      return Jef;
    }

    Matrix6d UR10Model::computeEEJacobainDerivative(const Vector6d &q, const Vector6d &qp)
    {
      Matrix6d Jdot;
      Jdot(0,0) = l(1)*qp(1)*sin(q(0))*sin(q(1)) - l(3)*qp(0)*sin(q(0)) - l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - l(1)*qp(0)*cos(q(0))*cos(q(1)) - l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) - l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3)) - l(2)*qp(0)*cos(q(0))*cos(q(1))*cos(q(2)) + l(2)*qp(0)*cos(q(0))*sin(q(1))*sin(q(2)) + l(2)*qp(1)*cos(q(1))*sin(q(0))*sin(q(2)) + l(2)*qp(1)*cos(q(2))*sin(q(0))*sin(q(1)) + l(2)*qp(2)*cos(q(1))*sin(q(0))*sin(q(2)) + l(2)*qp(2)*cos(q(2))*sin(q(0))*sin(q(1));
      Jdot(1,0) = l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(3)*qp(0)*cos(q(0)) - l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) + l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - l(1)*qp(0)*cos(q(1))*sin(q(0)) - l(1)*qp(1)*cos(q(0))*sin(q(1)) - l(2)*qp(0)*cos(q(1))*cos(q(2))*sin(q(0)) - l(2)*qp(1)*cos(q(0))*cos(q(1))*sin(q(2)) - l(2)*qp(1)*cos(q(0))*cos(q(2))*sin(q(1)) - l(2)*qp(2)*cos(q(0))*cos(q(1))*sin(q(2)) - l(2)*qp(2)*cos(q(0))*cos(q(2))*sin(q(1)) + l(2)*qp(0)*sin(q(0))*sin(q(1))*sin(q(2));
      Jdot(2,0) = 0;
      Jdot(3,0) = 0;
      Jdot(4,0) = 0;
      Jdot(5,0) = 0;
      Jdot(0,1) = qp(0)*sin(q(0))*(l(1)*sin(q(1)) + l(2)*sin(q(1) + q(2)) - l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) - l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - cos(q(0))*(l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(1)*qp(1)*cos(q(1)) + l(2)*cos(q(1) + q(2))*(qp(1) + qp(2)) - l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))));
      Jdot(1,1) = - sin(q(0))*(l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(1)*qp(1)*cos(q(1)) + l(2)*cos(q(1) + q(2))*(qp(1) + qp(2)) - l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - qp(0)*cos(q(0))*(l(1)*sin(q(1)) + l(2)*sin(q(1) + q(2)) - l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) - l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))));
      Jdot(2,1) = sin(q(0))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + l(3)*qp(0)*sin(q(0)) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(1)*qp(0)*cos(q(0))*cos(q(1)) - l(1)*qp(1)*sin(q(0))*sin(q(1)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3)) + l(2)*qp(0)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*qp(0)*cos(q(0))*sin(q(1))*sin(q(2)) - l(2)*qp(1)*cos(q(1))*sin(q(0))*sin(q(2)) - l(2)*qp(1)*cos(q(2))*sin(q(0))*sin(q(1)) - l(2)*qp(2)*cos(q(1))*sin(q(0))*sin(q(2)) - l(2)*qp(2)*cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(0))*(l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) - l(3)*qp(0)*cos(q(0)) - l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(1)*qp(0)*cos(q(1))*sin(q(0)) + l(1)*qp(1)*cos(q(0))*sin(q(1)) + l(2)*qp(0)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*qp(1)*cos(q(0))*cos(q(1))*sin(q(2)) + l(2)*qp(1)*cos(q(0))*cos(q(2))*sin(q(1)) + l(2)*qp(2)*cos(q(0))*cos(q(1))*sin(q(2)) + l(2)*qp(2)*cos(q(0))*cos(q(2))*sin(q(1)) - l(2)*qp(0)*sin(q(0))*sin(q(1))*sin(q(2))) - qp(0)*sin(q(0))*(l(3)*sin(q(0)) + l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(1)*cos(q(0))*cos(q(1)) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2))) - qp(0)*cos(q(0))*(l(3)*cos(q(0)) + l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(1)*cos(q(1))*sin(q(0)) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2)));
      Jdot(3,1) = qp(0)*cos(q(0));
      Jdot(4,1) = qp(0)*sin(q(0));
      Jdot(5,1) = 0;
      Jdot(0,2) = - cos(q(0))*(l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(2)*cos(q(1) + q(2))*(qp(1) + qp(2)) - l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - qp(0)*sin(q(0))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))));
      Jdot(1,2) = qp(0)*cos(q(0))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) - l(2)*sin(q(1) + q(2)) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - sin(q(0))*(l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(2)*cos(q(1) + q(2))*(qp(1) + qp(2)) - l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))));
      Jdot(2,2) = sin(q(0))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + l(3)*qp(0)*sin(q(0)) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3)) + l(2)*qp(0)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*qp(0)*cos(q(0))*sin(q(1))*sin(q(2)) - l(2)*qp(1)*cos(q(1))*sin(q(0))*sin(q(2)) - l(2)*qp(1)*cos(q(2))*sin(q(0))*sin(q(1)) - l(2)*qp(2)*cos(q(1))*sin(q(0))*sin(q(2)) - l(2)*qp(2)*cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(0))*(l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) - l(3)*qp(0)*cos(q(0)) - l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(2)*qp(0)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*qp(1)*cos(q(0))*cos(q(1))*sin(q(2)) + l(2)*qp(1)*cos(q(0))*cos(q(2))*sin(q(1)) + l(2)*qp(2)*cos(q(0))*cos(q(1))*sin(q(2)) + l(2)*qp(2)*cos(q(0))*cos(q(2))*sin(q(1)) - l(2)*qp(0)*sin(q(0))*sin(q(1))*sin(q(2))) - qp(0)*sin(q(0))*(l(3)*sin(q(0)) + l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2))) - qp(0)*cos(q(0))*(l(3)*cos(q(0)) + l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2)));
      Jdot(3,2) = qp(0)*cos(q(0));
      Jdot(4,2) = qp(0)*sin(q(0));
      Jdot(5,2) = 0;
      Jdot(0,3) = cos(q(0))*(l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - qp(0)*sin(q(0))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))));
      Jdot(1,3) = sin(q(0))*(l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) + qp(0)*cos(q(0))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))));
      Jdot(2,3) = sin(q(0))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + sin(q(0))*(l(1)*qp(1)*sin(q(1)) + l(2)*sin(q(1) + q(2))*(qp(1) + qp(2))) + l(3)*qp(0)*sin(q(0)) - qp(0)*cos(q(0))*(l(1)*cos(q(1)) + l(2)*cos(q(1) + q(2))) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(1)*qp(0)*cos(q(0))*cos(q(1)) - l(1)*qp(1)*sin(q(0))*sin(q(1)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3)) + l(2)*qp(0)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*qp(0)*cos(q(0))*sin(q(1))*sin(q(2)) - l(2)*qp(1)*cos(q(1))*sin(q(0))*sin(q(2)) - l(2)*qp(1)*cos(q(2))*sin(q(0))*sin(q(1)) - l(2)*qp(2)*cos(q(1))*sin(q(0))*sin(q(2)) - l(2)*qp(2)*cos(q(2))*sin(q(0))*sin(q(1))) - cos(q(0))*(l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) - cos(q(0))*(l(1)*qp(1)*sin(q(1)) + l(2)*sin(q(1) + q(2))*(qp(1) + qp(2))) - l(3)*qp(0)*cos(q(0)) - qp(0)*sin(q(0))*(l(1)*cos(q(1)) + l(2)*cos(q(1) + q(2))) - l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(1)*qp(0)*cos(q(1))*sin(q(0)) + l(1)*qp(1)*cos(q(0))*sin(q(1)) + l(2)*qp(0)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*qp(1)*cos(q(0))*cos(q(1))*sin(q(2)) + l(2)*qp(1)*cos(q(0))*cos(q(2))*sin(q(1)) + l(2)*qp(2)*cos(q(0))*cos(q(1))*sin(q(2)) + l(2)*qp(2)*cos(q(0))*cos(q(2))*sin(q(1)) - l(2)*qp(0)*sin(q(0))*sin(q(1))*sin(q(2))) - qp(0)*sin(q(0))*(l(3)*sin(q(0)) - cos(q(0))*(l(1)*cos(q(1)) + l(2)*cos(q(1) + q(2))) + l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(1)*cos(q(0))*cos(q(1)) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)) + l(2)*cos(q(0))*cos(q(1))*cos(q(2)) - l(2)*cos(q(0))*sin(q(1))*sin(q(2))) - qp(0)*cos(q(0))*(l(3)*cos(q(0)) + sin(q(0))*(l(1)*cos(q(1)) + l(2)*cos(q(1) + q(2))) + l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(1)*cos(q(1))*sin(q(0)) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)) - l(2)*cos(q(1))*cos(q(2))*sin(q(0)) + l(2)*sin(q(0))*sin(q(1))*sin(q(2)));
      Jdot(3,3) = qp(0)*cos(q(0));
      Jdot(4,3) = qp(0)*sin(q(0));
      Jdot(5,3) = 0;
      Jdot(0,4) = cos(q(1) + q(2) + q(3))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3))) + sin(q(1) + q(2) + q(3))*(l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)))*(qp(1) + qp(2) + qp(3)) - sin(q(0))*sin(q(1) + q(2) + q(3))*(l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - sin(q(0))*cos(q(1) + q(2) + q(3))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))))*(qp(1) + qp(2) + qp(3));
      Jdot(1,4) = sin(q(1) + q(2) + q(3))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)))*(qp(1) + qp(2) + qp(3)) - cos(q(1) + q(2) + q(3))*(l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) + l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3))) + cos(q(0))*sin(q(1) + q(2) + q(3))*(l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) + cos(q(0))*cos(q(1) + q(2) + q(3))*(l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))))*(qp(1) + qp(2) + qp(3));
      Jdot(2,4) = cos(q(0))*sin(q(1) + q(2) + q(3))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3))) - sin(q(0))*sin(q(1) + q(2) + q(3))*(l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) + l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3))) - qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3))) + qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3))*(l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3))) - cos(q(0))*cos(q(1) + q(2) + q(3))*(l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)))*(qp(1) + qp(2) + qp(3)) - sin(q(0))*cos(q(1) + q(2) + q(3))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)))*(qp(1) + qp(2) + qp(3));
      Jdot(3,4) = cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3));
      Jdot(4,4) = qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3)) + sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3));
      Jdot(5,4) = sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3));
      Jdot(0,5) = (cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)))*(l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(4)*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) - (l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) - l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))))*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + sin(q(4))*sin(q(1) + q(2) + q(3))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) - l(4)*(cos(q(3))*(qp(0)*cos(q(0))*cos(q(1))*sin(q(2)) + qp(0)*cos(q(0))*cos(q(2))*sin(q(1)) + qp(1)*cos(q(1))*cos(q(2))*sin(q(0)) + qp(2)*cos(q(1))*cos(q(2))*sin(q(0)) - qp(1)*sin(q(0))*sin(q(1))*sin(q(2)) - qp(2)*sin(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(qp(0)*cos(q(0))*sin(q(1))*sin(q(2)) - qp(0)*cos(q(0))*cos(q(1))*cos(q(2)) + qp(1)*cos(q(1))*sin(q(0))*sin(q(2)) + qp(1)*cos(q(2))*sin(q(0))*sin(q(1)) + qp(2)*cos(q(1))*sin(q(0))*sin(q(2)) + qp(2)*cos(q(2))*sin(q(0))*sin(q(1))) + qp(3)*cos(q(3))*(cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2))) - qp(3)*sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3))) - sin(q(4))*cos(q(1) + q(2) + q(3))*(l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2)))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(4))*sin(q(1) + q(2) + q(3))*(l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2)))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)));
      Jdot(1,5) = (cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)))*(l(5)*sin(q(4))*(cos(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) - sin(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*cos(q(1) + q(2)) - qp(3)*sin(q(3))*sin(q(1) + q(2))) - l(4)*(cos(q(3))*sin(q(1) + q(2))*(qp(1) + qp(2)) + sin(q(3))*cos(q(1) + q(2))*(qp(1) + qp(2)) + qp(3)*cos(q(3))*sin(q(1) + q(2)) + qp(3)*sin(q(3))*cos(q(1) + q(2))) + l(4)*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(5)*qp(4)*cos(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2)))) + (l(4)*(cos(q(3))*cos(q(1) + q(2)) - sin(q(3))*sin(q(1) + q(2))) - l(4)*cos(q(1) + q(2) + q(3)) + l(5)*sin(q(4))*(cos(q(3))*sin(q(1) + q(2)) + sin(q(3))*cos(q(1) + q(2))))*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - sin(q(4))*sin(q(1) + q(2) + q(3))*(l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*(cos(q(3))*(qp(0)*cos(q(1))*sin(q(0))*sin(q(2)) - qp(2)*cos(q(0))*cos(q(1))*cos(q(2)) - qp(1)*cos(q(0))*cos(q(1))*cos(q(2)) + qp(0)*cos(q(2))*sin(q(0))*sin(q(1)) + qp(1)*cos(q(0))*sin(q(1))*sin(q(2)) + qp(2)*cos(q(0))*sin(q(1))*sin(q(2))) + sin(q(3))*(qp(0)*cos(q(1))*cos(q(2))*sin(q(0)) + qp(1)*cos(q(0))*cos(q(1))*sin(q(2)) + qp(1)*cos(q(0))*cos(q(2))*sin(q(1)) + qp(2)*cos(q(0))*cos(q(1))*sin(q(2)) + qp(2)*cos(q(0))*cos(q(2))*sin(q(1)) - qp(0)*sin(q(0))*sin(q(1))*sin(q(2))) - qp(3)*cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) + qp(3)*sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) + l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3))) - qp(4)*cos(q(4))*sin(q(1) + q(2) + q(3))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3))) - sin(q(4))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3))*(l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)));
      Jdot(2,5) = (cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)))*(l(5)*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*(cos(q(3))*(qp(0)*cos(q(1))*sin(q(0))*sin(q(2)) - qp(2)*cos(q(0))*cos(q(1))*cos(q(2)) - qp(1)*cos(q(0))*cos(q(1))*cos(q(2)) + qp(0)*cos(q(2))*sin(q(0))*sin(q(1)) + qp(1)*cos(q(0))*sin(q(1))*sin(q(2)) + qp(2)*cos(q(0))*sin(q(1))*sin(q(2))) + sin(q(3))*(qp(0)*cos(q(1))*cos(q(2))*sin(q(0)) + qp(1)*cos(q(0))*cos(q(1))*sin(q(2)) + qp(1)*cos(q(0))*cos(q(2))*sin(q(1)) + qp(2)*cos(q(0))*cos(q(1))*sin(q(2)) + qp(2)*cos(q(0))*cos(q(2))*sin(q(1)) - qp(0)*sin(q(0))*sin(q(1))*sin(q(2))) - qp(3)*cos(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2))) + qp(3)*sin(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1)))) - l(4)*qp(0)*sin(q(0))*sin(q(1) + q(2) + q(3)) + l(4)*cos(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3))) - (l(5)*(cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) - l(4)*(cos(q(3))*(cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(2))*sin(q(1))) + sin(q(3))*(cos(q(0))*cos(q(1))*cos(q(2)) - cos(q(0))*sin(q(1))*sin(q(2)))) + l(4)*cos(q(0))*sin(q(1) + q(2) + q(3)))*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) + (cos(q(4))*sin(q(0)) - cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)))*(l(5)*(qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3))) - l(4)*(cos(q(3))*(qp(0)*cos(q(0))*cos(q(1))*sin(q(2)) + qp(0)*cos(q(0))*cos(q(2))*sin(q(1)) + qp(1)*cos(q(1))*cos(q(2))*sin(q(0)) + qp(2)*cos(q(1))*cos(q(2))*sin(q(0)) - qp(1)*sin(q(0))*sin(q(1))*sin(q(2)) - qp(2)*sin(q(0))*sin(q(1))*sin(q(2))) - sin(q(3))*(qp(0)*cos(q(0))*sin(q(1))*sin(q(2)) - qp(0)*cos(q(0))*cos(q(1))*cos(q(2)) + qp(1)*cos(q(1))*sin(q(0))*sin(q(2)) + qp(1)*cos(q(2))*sin(q(0))*sin(q(1)) + qp(2)*cos(q(1))*sin(q(0))*sin(q(2)) + qp(2)*cos(q(2))*sin(q(0))*sin(q(1))) + qp(3)*cos(q(3))*(cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2))) - qp(3)*sin(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1)))) + l(4)*sin(q(0))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) + l(4)*qp(0)*cos(q(0))*sin(q(1) + q(2) + q(3))) - (l(5)*(cos(q(0))*cos(q(4)) + sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3))) + l(4)*(cos(q(3))*(cos(q(1))*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(1))) + sin(q(3))*(cos(q(1))*cos(q(2))*sin(q(0)) - sin(q(0))*sin(q(1))*sin(q(2)))) - l(4)*sin(q(0))*sin(q(1) + q(2) + q(3)))*(qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)));
      Jdot(3,5) = qp(0)*cos(q(0))*cos(q(4)) - qp(4)*sin(q(0))*sin(q(4)) + cos(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(4)*cos(q(0))*cos(q(4))*cos(q(1) + q(2) + q(3)) + qp(0)*sin(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3));
      Jdot(4,5) = qp(0)*cos(q(4))*sin(q(0)) + qp(4)*cos(q(0))*sin(q(4)) + sin(q(0))*sin(q(4))*sin(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3)) - qp(0)*cos(q(0))*sin(q(4))*cos(q(1) + q(2) + q(3)) - qp(4)*cos(q(4))*sin(q(0))*cos(q(1) + q(2) + q(3));
      Jdot(5,5) = - qp(4)*cos(q(4))*sin(q(1) + q(2) + q(3)) - sin(q(4))*cos(q(1) + q(2) + q(3))*(qp(1) + qp(2) + qp(3));
      return Jdot;
    }

  }
}



// other way for FK

// UR10Model::computeFKJoint0();
// UR10Model::computeFKJoint1();
// UR10Model::computeFKJoint2();
// UR10Model::computeFKJoint3();
// UR10Model::computeFKJoint4();
// UR10Model::computeFKJoint5();

// computeJacobian();
// Hr_[0] << cos(q(0)), 0,  sin(q(0)),  0,
//           sin(q(0)), 0, -cos(q(0)),  0,
//                 0, 1,        0, l(0),
//                 0, 0,        0,  1;

// Hr_[1] << cos(q(1)), -sin(q(1)), 0, l(1)*cos(q(1)),
//           sin(q(1)),  cos(q(1)), 0, l(1)*sin(q(1)),
//                 0,        0, 1,          0,
//                 0,        0, 0,          1;

// Hr_[2] << cos(q(2)), -sin(q(2)), 0, l(2)*cos(q(2)),
//           sin(q(2)),  cos(q(2)), 0, l(2)*sin(q(2)),
//                 0,        0, 1,          0,
//                 0,        0, 0,          1;

// Hr_[3] << cos(q(3)), 0,  sin(q(3)),  0,
//           sin(q(3)), 0, -cos(q(3)),  0,
//                 0, 1,        0, l(3),
//                 0, 0,        0,  1;

// Hr_[4] << cos(q(4)),  0, -sin(q(4)),  0,
//           sin(q(4)),  0,  cos(q(4)),  0,
//                 0, -1,        0, l(4),
//                 0,  0,        0,  1;

// Hr_[5] << cos(q(5)), -sin(q(5)), 0,  0,
//           sin(q(5)),  cos(q(5)), 0,  0,
//                 0,        0, 1, l(5),
//                 0,        0, 0,  1;
// H_0_[0] = Hr_[0];
// for (int i = 1; i < STD_DOF; ++i)
// {
//   H_0_[i] = H_0_[i-1] * Hr_[i];
// }