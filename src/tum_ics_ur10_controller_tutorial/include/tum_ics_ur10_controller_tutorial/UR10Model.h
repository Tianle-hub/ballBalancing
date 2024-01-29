#ifndef UR10_ROBOR_MODEL_UR10_ROBOT_MODEL
#define UR10_ROBOR_MODEL_UR10_ROBOT_MODEL

#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <tum_ics_ur_robot_lli/JointState.h>

namespace tum_ics_ur_robot_lli
{
  namespace ur10_model
  {
    struct UR10Parameters
    {
      Vector6d l;
      Vector6d m;
      Eigen::Matrix<double, 2, 3> I0;
      Eigen::Matrix<double, 2, 3> I1;
      Eigen::Matrix<double, 2, 3> I2;
      Eigen::Matrix<double, 2, 3> I3;
      Eigen::Matrix<double, 2, 3> I4;
      Eigen::Matrix<double, 2, 3> I5;
      Vector3d tcm0;
      Vector3d tcm1;
      Vector3d tcm2;
      Vector3d tcm3;
      Vector3d tcm4;
      Vector3d tcm5;
    };

    class UR10Model
    {
      public:

      private:
        JointState state_;

        UR10Parameters ps_;

        Matrix6d M_;
        Matrix6d C_;
        Vector6d G_;

        std::vector<Matrix4d> H_0_;

        std::vector<Matrix6d> J_0_;

      public:
        UR10Model();

        ~UR10Model();

        void initModel(const JointState &state_init, const int n);

        void updateJointState(const JointState &state_new);

        std::vector<Matrix4d> computeForwardKinematics();

        void computeJacobian();

        void computeDynamic();

        void computeInertialMatrix();

        void computeCoriolisMatrix();

        void computeGeneralizedGravity();

        void computeAllTerms();

      private:
        void computeFKJoint0();

        void computeFKJoint1();

        void computeFKJoint2();

        void computeFKJoint3();

        void computeFKJoint4();

        void computeFKJoint5();

      

    };

  }

}




#endif // UR10_ROBOR_MODEL_UR10_ROBOT_MODEL
