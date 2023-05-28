/*
***********************************************************************************************************************************************************************
This file defines math helpers.
Copyright (C) 2023

Authors:
Ruixuan Liu: ruixuanl@andrew.cmu.edu
Changliu Liu : cliu6@andrew.cmu.edu

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************************************************************************************************************
*/
#pragma once
#include "Utils/Common.hpp"
#include "Utils/ErrorHandling.hpp"

#define PI 3.141592653589793

namespace lego_assembly
{
namespace math
{

/* -------------------------------------------------------------------------- */
/*                             Vector definitions                             */
/* -------------------------------------------------------------------------- */
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorJd;

/* -------------------------------------------------------------------------- */
/*                                   Matrix                                   */
/* -------------------------------------------------------------------------- */
Eigen::MatrixXd PInv(const Eigen::MatrixXd& M);
Eigen::MatrixXd EigenVcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2);

/* -------------------------------------------------------------------------- */
/*                               Transformation                               */
/* -------------------------------------------------------------------------- */
template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data);

/* -------------------------------------------------------------------------- */
/*                                 Kinematics                                 */
/* -------------------------------------------------------------------------- */

Eigen::Matrix4d FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad);
Eigen::MatrixXd Jacobian_full(const VectorJd& q_deg, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad);
VectorJd IK_closed_form(const VectorJd& cur_q, const Eigen::Matrix4d& goal_T, const Eigen::MatrixXd& DH, 
                        const Eigen::Matrix4d& T_base_inv, const Eigen::Matrix4d& T_tool_inv,const bool& joint_rad,bool& status);
/**
 * @brief 
 * 
 * @param q [n_joint,]
 * @param cart_goal [3,]
 * @param rot_goal [3,3]
 * @param DH [1 + n_joint + n_ee, 4]
 * @return VectorJd
 */
VectorJd IK(const VectorJd& q, const Eigen::Matrix<double, 3, 1>& cart_goal, const Eigen::Matrix3d rot_goal, 
            const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad, const uint& max_iter, const double& step);

Vector6d get_6d_error(const Eigen::MatrixXd& pos_s, const Eigen::Quaterniond& quat_s, 
                           const Eigen::MatrixXd& pos_e, const Eigen::Quaterniond& quat_e);

/* -------------------------------------------------------------------------- */
/*                             Capsule Geometry                               */
/* -------------------------------------------------------------------------- */
struct Capsule{
    Eigen::MatrixXd p; // 3x2
    double r;

    Eigen::Vector3d vel;

    Capsule at(const double& t) const
    {
        Capsule newcap = *this;
        newcap.p.col(0) += vel*t;
        newcap.p.col(1) += vel*t;

        return newcap;
    }
};

typedef struct{
    Eigen::MatrixXd p1; // 3x1
    Eigen::MatrixXd p2; // 3x1
} lineseg;

template <typename T>
T Clip(const T& val, const T& minval, const T& maxval)
{
    if (val < minval)
    {
        return minval;
    }
    else if (val > maxval)
    {
        return maxval;
    }
    else
    {
        return val;
    }
}

/* -------------------------------------------------------------------------- */
/*                              Collision Check                               */
/* -------------------------------------------------------------------------- */
double DistLineSeg(const lineseg& line1, const lineseg& line2, Eigen::MatrixXd& critical_pts);
double DistCap2Cap(const Capsule& cap1, const Capsule& cap2, Eigen::MatrixXd& critical_pts);

/* -------------------------------------------------------------------------- */
/*                            Common Operations                               */
/* -------------------------------------------------------------------------- */
bool ApproxEqNum(const double& a, const double& b, const double& thres);
bool VecPerp(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2);

}
}