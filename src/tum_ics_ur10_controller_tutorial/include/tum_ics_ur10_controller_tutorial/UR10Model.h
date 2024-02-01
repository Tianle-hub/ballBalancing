#ifndef UR10_ROBOR_MODEL_UR10_ROBOT_MODEL
#define UR10_ROBOR_MODEL_UR10_ROBOT_MODEL

#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tum_ics_ur_robot_lli/JointState.h>
#include <tumtools/Math/MathTools.h>

namespace tum_ics_ur_robot_lli
{
  namespace ur10_model
  {
    class UR10Model
    {
      public:

      private:
        int DOF;

        JointState state_;

        Vector6d l;
        Vector6d m;
        Matrix6d I;
        Eigen::Matrix<double,6,3> tcm;

        Vector6d G_;

        Eigen::Matrix<double,167,1> Theta_;

        Eigen::Matrix<double,6,167> Y_;

        std::vector<Matrix4d> H_0_;

        std::vector<Matrix4d> Hr_;

        std::vector<Matrix4d> Hcm_0_;

        std::vector<Matrix6d> J_0_;

        Matrix6d Jef_0_;

        double g;

        Vector6d d;
        Vector6d a;
        Vector6d alpha;

        Eigen::DiagonalMatrix<double,167> Gamma_inv_;

      public:
        UR10Model();

        ~UR10Model();

        void initModel(const JointState &state_init, const int n);

        void updateJointState(const JointState &state_new);

        std::vector<Matrix4d> computeForwardKinematics(const Vector6d &q);

        std::vector<Matrix4d> computeFKCoMs();

        void computeJacobian();

        Vector6d computeGeneralizedGravity(const Vector6d &q);

        Matrix6d computeEEJacobian(const Vector6d &q);

        Matrix6d computeEEJacobainDerivative(const Vector6d &q, const Vector6d &qp);

        Vector6d computeEEPose(const Vector6d &q);

        void computeAllTerms();

        void initTheta();

        Eigen::Matrix<double,6,167> computeRefRegressor(const Vector6d &q, const Vector6d &qp, const Vector6d &qpr, const Vector6d &qppr);

        void updateTheta(const Vector6d &q, const Vector6d &qp, const Vector6d &qpr, const Vector6d &qppr, const Vector6d &Sq);

        Eigen::Matrix<double,167,1> getTheta()
        {
          return Theta_;
        }

      private:
        Matrix4d computeFKJoint0(const Vector6d &q);

        Matrix4d computeFKJoint1(const Vector6d &q);

        Matrix4d computeFKJoint2(const Vector6d &q);

        Matrix4d computeFKJoint3(const Vector6d &q);

        Matrix4d computeFKJoint4(const Vector6d &q);

        Matrix4d computeFKJoint5(const Vector6d &q);

        void computeFKCoM0();

        void computeFKCoM1();

        void computeFKCoM2();

        void computeFKCoM3();

        void computeFKCoM4();

        void computeFKCoM5();

        Matrix4d relativeTrans(const double theta, const double d, const double a, const double alpha);

        Matrix3d rotX(const double alphaX);

        Matrix3d rotY(const double betaY);

        Matrix3d rotZ(const double gammaZ);

    };

  }

}




#endif // UR10_ROBOR_MODEL_UR10_ROBOT_MODEL
