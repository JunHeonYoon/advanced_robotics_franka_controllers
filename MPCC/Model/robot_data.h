#ifndef MPCC_ROBOT_DATA_H
#define MPCC_ROBOT_DATA_H

#include "Model/robot_model.h"

namespace mpcc
{
// Data containing kinematic of robot wrt given joint angle
struct RobotData
{
    Eigen::Matrix<double,PANDA_DOF,1> q;           // joint angle
    // Eigen::Matrix<double,PANDA_DOF,1> q_dot;       // joint velocity
    
    Eigen::Vector3d EE_position;                   // End-Effector position
    Eigen::Matrix3d EE_orientation;                // End-Effector orientation

    Eigen::Matrix<double,6,PANDA_DOF> J;           // End-Effector Jacobian
    Eigen::Matrix<double,3,PANDA_DOF> Jv;          // End-Effector translation Jacobian
    Eigen::Matrix<double,3,PANDA_DOF> Jw;          // End-Effector rotation Jacobian

    double manipul;                                // Manipullabilty
    Eigen::Matrix<double,PANDA_DOF,1> d_manipul;   // Gradient of Manipullabilty wrt q

    bool is_data_valid = false;

    void setZero()
    {
        q.setZero();
        // q_dot.setZero();
        EE_position.setZero();
        EE_orientation.setZero();
        Jv.setZero();
        Jw.setZero();
        manipul = 0;
        d_manipul.setZero();
    }

    void update(Eigen::Matrix<double,PANDA_DOF,1> q_input, const std::unique_ptr<RobotModel> &robot_model)
    {
        q = q_input;
        EE_position = robot_model->getEEPosition(q);
        EE_orientation = robot_model->getEEOrientation(q);
        J = robot_model->getJacobian(q);
        Jv = J.block(0,0,3,PANDA_DOF);
        Jw = J.block(3,0,3,PANDA_DOF);
        manipul = robot_model->getManipulability(q);
        d_manipul = robot_model->getDManipulability(q);

        is_data_valid = true;
    }

    bool check_valid_data(Eigen::Matrix<double,PANDA_DOF,1> q_input)
    {
        if((q - q_input).norm() < 1E-8 && is_data_valid) return true;
        else return false;
    }
};
}
#endif // MPCC_ROBOT_DATA