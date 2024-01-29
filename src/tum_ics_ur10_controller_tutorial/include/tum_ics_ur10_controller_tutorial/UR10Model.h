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
      Vector6d I0;
      Vector6d I1;
      Vector6d I2;
      Vector6d I3;
      Vector6d I4;
      Vector6d I5;
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

        std::vector<Matrix4d> Hcm_0_;

        std::vector<Matrix6d> J_0_;

        double g;

      public:
        UR10Model();

        ~UR10Model();

        void initModel(const JointState &state_init, const int n);

        void updateJointState(const JointState &state_new);

        std::vector<Matrix4d> computeForwardKinematics();

        std::vector<Matrix4d> computeFKCoMs();

        void computeJacobian();

        void computeDynamic();

        void computeInertialMatrix();

        void computeCoriolisMatrix();

        Vector6d computeGeneralizedGravity();

        void computeAllTerms();

      private:
        void computeFKJoint0();

        void computeFKJoint1();

        void computeFKJoint2();

        void computeFKJoint3();

        void computeFKJoint4();

        void computeFKJoint5();

        void computeFKCoM0();

        void computeFKCoM1();

        void computeFKCoM2();

        void computeFKCoM3();

        void computeFKCoM4();

        void computeFKCoM5();

      

    };

  }

}




#endif // UR10_ROBOR_MODEL_UR10_ROBOT_MODEL
