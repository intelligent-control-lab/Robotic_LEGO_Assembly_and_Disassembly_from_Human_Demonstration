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

#include "Utils/Math.hpp"

#undef inverse

namespace lego_assembly
{
namespace math
{

template Eigen::Matrix<float, Eigen::Dynamic, 1> ToEigen<float>(std::vector<float> data);
template Eigen::Matrix<double, Eigen::Dynamic, 1> ToEigen<double>(std::vector<double> data);

/* -------------------------------------------------------------------------- */
/*                                   Matrix                                   */
/* -------------------------------------------------------------------------- */
Eigen::MatrixXd PInv(const Eigen::MatrixXd& M)
{
    auto nrow = M.rows();
    auto ncol = M.cols();
    Eigen::MatrixXd Minv;

    if (nrow > ncol)
    {
        Minv = ( ( M.transpose() * M ).inverse() ) * M.transpose();
    }
    else if (nrow < ncol)
    {
        Minv = M.transpose() * ( ( M * M.transpose() ).inverse() );
    }
    else
    {
        Minv = M.inverse();
    }

    return Minv;
}

Eigen::MatrixXd EigenVcat(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
{
    try
    {
        if (mat1.rows() == 0){
            return mat2;
        }
        else if (mat2.rows() == 0){
            return mat1;
        }
        else{
            if (mat1.cols() != mat2.cols())
            {
                std::ostringstream ss;
                ss << ERR_HEADER << "Expected mat1 and mat2 to have same cols(), got ["
                    << mat1.cols() << "], [" << mat2.cols() << "]\n";
                throw std::runtime_error(ss.str());
            }
            Eigen::MatrixXd new_mat(mat1.rows()+mat2.rows(), mat1.cols());
            new_mat << mat1, mat2;
            return new_mat;
        } 
    }
    catch(const std::exception& e)
    {
        throw;
    }
}


/* -------------------------------------------------------------------------- */
/*                               Transformation                               */
/* -------------------------------------------------------------------------- */

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data)
{
    Eigen::Matrix<T, Eigen::Dynamic, 1> data_eigen;
    size_t n = data.size();
    data_eigen.resize(n, 1);
    for (size_t i = 0; i < n; ++i)
    {
        data_eigen(i) = data.at(i);
    }
    return data_eigen;
}

/* -------------------------------------------------------------------------- */
/*                                 Kinematics                                 */
/* -------------------------------------------------------------------------- */

Eigen::Matrix4d FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad)
{
    Eigen::Matrix3d R;
    Eigen::MatrixXd T(3, 1);
    Eigen::Matrix4d trans_mtx = Eigen::Matrix4d::Identity();
    trans_mtx.col(3) << base_frame, 1;
    Eigen::Matrix4d tmp;
    Eigen::MatrixXd DH_cur = DH;
    VectorJd q_rad = q;

    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<q.rows(); i++)
        {
            q_rad(i) = q(i) * PI / 180;
        }
    }

    DH_cur.col(0) = DH.col(0) + q_rad;
    for(int i=0; i<DH_cur.rows(); i++)
    {
        R << cos(DH_cur.coeff(i, 0)), -sin(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)),  sin(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             sin(DH_cur.coeff(i, 0)),  cos(DH_cur.coeff(i, 0)) * cos(DH_cur.coeff(i, 3)), -cos(DH_cur.coeff(i, 0)) * sin(DH_cur.coeff(i, 3)),
             0,                        sin(DH_cur.coeff(i, 3)),                            cos(DH_cur.coeff(i, 3));

        T << DH_cur.coeff(i, 2) * cos(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 2) * sin(DH_cur.coeff(i, 0)), 
             DH_cur.coeff(i, 1);
        tmp << R, T, 0, 0, 0, 1;
        trans_mtx = trans_mtx * tmp;
    }
    return trans_mtx;
}

Eigen::MatrixXd Jacobian_full(const VectorJd& q_deg, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad)
{
    int njoints = DH.rows();
    Eigen::MatrixXd DH_cur = DH;
    VectorJd q_rad = q_deg;
    Eigen::Matrix<double, 3, Eigen::Dynamic> z = Eigen::MatrixXd::Zero(3, njoints);
    Eigen::Matrix<double, 3, Eigen::Dynamic> r = Eigen::MatrixXd::Zero(3, njoints);
    Eigen::MatrixXd J(6, njoints);
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T.col(3) << base_frame, 1;

    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<q_deg.rows(); i++)
        {
            q_rad(i) = q_deg(i) * PI / 180;
        }
    }
    DH_cur.col(0) = DH.col(0) + q_rad;

    Eigen::MatrixXd a = DH_cur.col(3);
    Eigen::MatrixXd A = DH_cur.col(2);
    Eigen::MatrixXd D = DH_cur.col(1);
    Eigen::MatrixXd q = DH_cur.col(0);
    Eigen::MatrixXd temp(4, 4);
    Eigen::MatrixXd ee(3, 1);
    for(int i=0; i<njoints; i++)
    {
        z.col(i) << T.block(0, 2, 3, 1);
        r.col(i) << T.block(0, 3, 3, 1);
        temp << cos(q(i)), -sin(q(i)) * cos(a(i)),  sin(q(i)) * sin(a(i)),  A(i) * cos(q(i)),
                sin(q(i)),  cos(q(i)) * cos(a(i)), -cos(q(i)) * sin(a(i)),  A(i) * sin(q(i)),
                0,          sin(a(i)),            cos(a(i)),            D(i),
                0,          0,                    0,                    1;
        T = T * temp;
    }
    ee = T.block(0, 3, 3, 1);
    for(int i=0; i<njoints; i++)
    {
        J.col(i) << z.col(i).cross(Eigen::Vector3d{ee - r.col(i)}), z.col(i);
    }
    return J;
}

VectorJd IK_closed_form(const VectorJd& cur_q, const Eigen::Matrix4d& goal_T, const Eigen::MatrixXd& DH, 
                        const Eigen::Matrix4d& T_base_inv, const Eigen::Matrix4d& T_tool_inv,const bool& joint_rad,bool& status)
{
    auto start = high_resolution_clock::now();
    double eps = 0.0000001;
    status = false;
    VectorJd theta = cur_q;
    Eigen::MatrixXd DH_cur = DH;
    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<cur_q.rows(); i++)
        {
            theta(i) = theta(i) * PI / 180;
        }
    }
    VectorJd cur_theta = theta;

    Eigen::Matrix4d T = T_base_inv * goal_T * T_tool_inv;
    Eigen::Matrix3d R = T.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> P = T.block(0, 3, 3, 1);
    double X, Y, Z, r, r2, a1, a12, a2, a22, a3, a32, d4, d42, m, e, c, l, l2, h, f1, f2, t1, t2, t3, k, g1, g2, q1, q2, min_diff,
           th1, th2, th3, th4, th5, th6, tmp;
    X = P(0, 0);
    Y = P(1, 0);
    Z = P(2, 0);
    r = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
    a1 = DH(0, 2);
    a2 = DH(1, 2);
    a3 = DH(2, 2);
    d4 = DH(3, 1);
    r2 = pow(r, 2);
    a12 = pow(a1, 2);
    a22 = pow(a2, 2);
    a32 = pow(a3, 2);
    d42 = pow(d4, 2);
    m = a32 + d42;
    e = 2 * r2;
    c = 4 * a12;
    l = a2 * (2 * m + 2 * a12 + 2 * a22 - c - e);
    l2 = pow(l, 2);
    h = (c + e) * m - pow((m + a12 + a22), 2) + a12 * e + a22 * e + 4 * a12 * a22 - 4 * a12 * pow(Z, 2) - pow(r, 4);

    double cond1, cond2;
    cond1 = 4 * l2 + 16 * a22 * h;
    min_diff = 10000;
    if(cond1 >= 0)
    {
        f1 = (-2 * l + sqrt(cond1)) / (8 * a22);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2 >= 0)
        {
            tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1));
            if(abs(tmp - cur_theta(2)) < min_diff)
            {
                min_diff = abs(tmp - cur_theta(2));
                th3 = tmp;
            }
            tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1));
            if(abs(tmp - cur_theta(2)) < min_diff)
            {
                min_diff = abs(tmp - cur_theta(2));
                th3 = tmp;
            }
        }
        f1 = (-2 * l - sqrt(cond1)) / (8 * a22);
        cond2 = d42 + a32 - pow(f1, 2);
        if(cond2)
        {
           
            tmp = 2 * atan((-d4 + sqrt(cond2)) / (a3 + f1));
            if(abs(tmp - cur_theta(2)) < min_diff)
            {
                min_diff = abs(tmp - cur_theta(2));
                th3 = tmp;
            }
            tmp = 2 * atan((-d4 - sqrt(cond2)) / (a3 + f1));
            if(abs(tmp - cur_theta(2)) < min_diff)
            {
                min_diff = abs(tmp - cur_theta(2));
                th3 = tmp;
            }
        }
    }
    else
    {
        status = false;
        return cur_q;
    }

    f1 = -sin(th3) * d4 + a3 * cos(th3);
    f2 = cos(th3) * d4 + a3 * sin(th3);
    t1 = f1 + a2;
    k = pow(f1, 2) + pow(f2, 2) + 2 * f1 * a2 + a12 + a22;
    t2 = (r2 - k) / (2 * a1);
    cond1 = pow(f2, 2) + pow(t1, 2) - pow(t2, 2);
    double th2_tmp1, th2_tmp2, th2_tmp3;

    th2_tmp1 = 2 * atan((f2 + sqrt(cond1)) / (t1 + t2)) + PI / 2;
    th2_tmp2 = 2 * atan((f2 - sqrt(cond1)) / (t1 + t2)) + PI / 2;

    t1 = f2;
    t2 = -f1-a2;
    t3 = Z;
    th2_tmp3 = 2 * atan((t2 + sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3)) + PI / 2;
    if(ApproxEqNum(th2_tmp1, th2_tmp3, eps) || ApproxEqNum(th2_tmp2, th2_tmp3, eps))
    {
        th2 = th2_tmp3;
    }
    th2_tmp3 = 2 * atan((t2 - sqrt(pow(t2, 2) + pow(t1, 2) - pow(t3, 2))) / (t1 + t3)) + PI / 2;
    if(ApproxEqNum(th2_tmp1, th2_tmp3, eps) || ApproxEqNum(th2_tmp2, th2_tmp3, eps))
    {
        th2 = th2_tmp3;
    }
    double th2_tmp = th2 - PI / 2;

    g1 = f1 * cos(th2_tmp) + f2 * sin(th2_tmp) + a2 * cos(th2_tmp);
    g2 = f1 * sin(th2_tmp) - f2 * cos(th2_tmp) + a2 * sin(th2_tmp);
    q1 = g1+a1;
    q2 = 0;
    cond1 = pow(q2, 2) + pow(q1, 2) - pow(X, 2);
    double th1_tmp1, th1_tmp2, th1_tmp3;
    th1_tmp1 = 2 * atan((q2 + sqrt(cond1)) / (q1 + X));
    th1_tmp2 = 2 * atan((q2 - sqrt(cond1)) / (q1 + X));
    
    q1 = 0;
    q2 = g1+a1;
    th1_tmp3 = 2 * atan((q2 + sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y));
    if(ApproxEqNum(th1_tmp1, th1_tmp3, eps) || ApproxEqNum(th1_tmp2, th1_tmp3, eps))
    {
        th1 = th1_tmp3;
    }
    th1_tmp3 = 2 * atan((q2 - sqrt(pow(q2, 2) + pow(q1, 2) - pow(Y, 2))) / (q1 + Y));
    if(ApproxEqNum(th1_tmp1, th1_tmp3, eps) || ApproxEqNum(th1_tmp2, th1_tmp3, eps))
    {
        th1 = th1_tmp3;
    }

    cur_theta(0) = th1;
    cur_theta(1) = th2;
    cur_theta(2) = th3;
    DH_cur.col(0) = DH.col(0) + cur_theta;
    Eigen::Matrix3d R03 = Eigen::Matrix3d::Identity(3, 3);
    Eigen::MatrixXd a = DH_cur.col(3);
    Eigen::MatrixXd q = DH_cur.col(0);
    Eigen::Matrix3d temp(3, 3); 
    for(int i=0; i<3; i++)
    {
        temp << cos(q(i)), -sin(q(i)) * cos(a(i)),  sin(q(i)) * sin(a(i)),  
                sin(q(i)),  cos(q(i)) * cos(a(i)), -cos(q(i)) * sin(a(i)),  
                0,          sin(a(i)),            cos(a(i));
        R03 = R03 * temp;
    }
    Eigen::Matrix3d R36 = PInv(R03) * R;
    th5 = acos(-R36(2, 2));
    double s5 = sin(th5);

    if(abs(s5) <= 0.000001)
    {
        th4 = 0;
        if(abs(th5) < 0.000001)
        {
            th5 = 0;
            th6 = atan2(R36(0, 1), R36(0, 0));
        }
        else{
            th5 = PI;
            th6 = atan2(R36(1, 0), -R36(1, 1));

        }
    }
    else
    {
        double th4_1 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
        double th6_1 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
        double sum1 = sqrt(pow(th5, 2) + pow(th4_1, 2) + pow(th6_1, 2));
        s5 = sin(-th5);
        double th4_2 = atan2(R36(1, 2) / s5, R36(0, 2) / s5);
        double th6_2 = atan2(R36(2, 1) / s5, R36(2, 0) / s5);
        double sum2 = sqrt(pow(th5, 2) + pow(th4_2, 2) + pow(th6_2, 2));
        if(sum1 < sum2)
        {
            th4 = th4_1;
            th6 = th6_1;
        }
        else
        {
            th5 = -th5;
            th4 = th4_2;
            th6 = th6_2;

        }
    }
    theta << th1, th2, th3, th4, th5, th6;
    status = true;
    // Rad to Deg
    for(int i=0; i<theta.rows(); i++)
    {
        theta(i) = theta(i) * 180 / PI;
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "IK closed form calc time: " << duration.count() / 1000000.0 << " s" << std::endl;
    return theta;
}


VectorJd IK(const VectorJd& q, const Eigen::Matrix<double, 3, 1>& cart_goal, const Eigen::Matrix3d rot_goal, 
            const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, const bool& joint_rad, const uint& max_iter, const double& step)
{
    auto start = high_resolution_clock::now();
    VectorJd theta = q;
    if(!joint_rad)
    {
        // Deg to Rad
        for(int i=0; i<q.rows(); i++)
        {
            theta(i) = theta(i) * PI / 180;
        }
    }
    VectorJd theta_init = theta;
    Eigen::Matrix4d homo_mtx = FK(theta, DH, base_frame, 1);
    Eigen::Matrix3d rot_mtx = homo_mtx.block(0, 0, 3, 3);
    Eigen::MatrixXd pos_s = homo_mtx.block(0, 3, 3, 1);
    Eigen::Quaterniond quat_s(rot_mtx);
    Eigen::MatrixXd pos_e = cart_goal.block(0, 0, 3, 1);
    Eigen::Quaterniond quat_e(rot_goal);
    
    Vector6d eps;
    eps << 0.0001, 0.0001, 0.0001, 0.00001, 0.00001, 0.00001;
    Eigen::MatrixXd J, J_inv, d_th;
    Vector6d error = get_6d_error(pos_s, quat_s, pos_e, quat_e);
    uint iter_num = 0;

    while(abs(error(0)) > eps(0) || abs(error(1)) > eps(1) || abs(error(2)) > eps(2) ||
          abs(error(3)) > eps(3) || abs(error(4)) > eps(4) || abs(error(5)) > eps(5))
    {
        if(iter_num >= max_iter)
        {
            std::cout << "IK failed!" << std::endl;
            theta = theta_init;
            break;
        }
        J = Jacobian_full(theta, DH, base_frame, 1);
        J_inv = PInv(J);
        d_th = J_inv * error;
        theta = theta + step * d_th;

        homo_mtx = FK(theta, DH, base_frame, 1);
        rot_mtx = homo_mtx.block(0, 0, 3, 3);
        pos_s = homo_mtx.block(0, 3, 3, 1);
        quat_s = Eigen::Quaterniond(rot_mtx);
        error = get_6d_error(pos_s, quat_s, pos_e, quat_e);
        iter_num ++;
    }
    std::cout << "IK iter num: " << iter_num << std::endl;
    // Rad to Deg
    for(int i=0; i<q.rows(); i++)
    {
        theta(i) = theta(i) * 180 / PI;
        while(theta(i) > 180)
        {
            theta(i) = theta(i) - 360;
        }
        while(theta(i) < -180)
        {
            theta(i) = theta(i) + 360;
        }
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "IK calc time: " << duration.count() / 1000000.0 << " s" << std::endl;
    return theta;
}

Vector6d get_6d_error(const Eigen::MatrixXd& pos_s, const Eigen::Quaterniond& quat_s, 
                      const Eigen::MatrixXd& pos_e, const Eigen::Quaterniond& quat_e)
{
    Vector6d error;
    error << pos_e - pos_s, (quat_e * quat_s.conjugate()).vec(); 
    return error;
}




/* -------------------------------------------------------------------------- */
/*                              Collision Check                               */
/* -------------------------------------------------------------------------- */
double DistLineSeg(const lineseg& line1, const lineseg& line2, Eigen::MatrixXd& critical_pts)
{
    Eigen::Vector3d s1, e1, s2, e2;
    Eigen::Vector3d d1, d2, d12;

    s1 = line1.p1;
    e1 = line1.p2;
    s2 = line2.p1;
    e2 = line2.p2;
    d1 = e1 - s1;
    d2 = e2 - s2;
    d12 = s2 - s1;
    
    double D1 = d1.dot(d1);
    double D2 = d2.dot(d2);
    double S1 = d1.dot(d12);
    double S2 = d2.dot(d12);
    double R = d1.dot(d2);
    double den = D1 * D2 - pow(R, 2);
    double u = 0, t = 0, uf = 0;

    // One of the segments is a point
    if (D1 == 0 || D2 == 0){
        if (D1 != 0){
            t = Clip(S1 / D1, 0.0, 1.0);
            critical_pts.col(0) << s1 + t * d1;
            critical_pts.col(1) << s2;
        }
        else if (D2 != 0){ 
            u = Clip(-S2 / D2, 0.0, 1.0);
            critical_pts.col(0) << s1;
            critical_pts.col(1) << s2 + u * d2;
        }
    }
    // Segments are parallel 
    else if (den == 0){ 
        u = -S2 / D2;
        uf = Clip(u, 0.0, 1.0);
        if (uf != u){
            t = (uf * R + S1) / D1;
            t = Clip(t, 0.0, 1.0);
            u = uf;
        }
        critical_pts.col(0) << s1 + t * d1;
        critical_pts.col(1) << s2 + u * d2;
    }
    // General case
    else{ 
        t = Clip((S1 * D2 - S2 * R) / den, 0.0, 1.0);
        u = (t * R - S2) / D2;
        uf = Clip(u, 0.0, 1.0);
        if (uf != u){
            t = Clip((uf * R + S1) / D1, 0.0, 1.0);
            u = uf;
        }
        critical_pts.col(0) << s1 + t * d1;
        critical_pts.col(1) << s2 + u * d2;
    }                        
    // Compute the distance, given t and u 
    Eigen::MatrixXd dist = d1 * t - d2 * u - d12;
    return dist.norm();
}

double DistCap2Cap(const Capsule& cap1, const Capsule& cap2, Eigen::MatrixXd& critical_pts)
{
    lineseg axis1, axis2;
    double r1, r2;

    axis1.p1 = cap1.p.col(0);
    axis1.p2 = cap1.p.col(1);
    axis2.p1 = cap2.p.col(0);
    axis2.p2 = cap2.p.col(1);
    r1 = cap1.r;
    r2 = cap2.r;
    
    double dist = DistLineSeg(axis1, axis2, critical_pts);
    return dist - r1 - r2;
}


/* -------------------------------------------------------------------------- */
/*                            Common Operations                               */
/* -------------------------------------------------------------------------- */


bool ApproxEqNum(const double& a, const double& b, const double& thres){
    return (bool) (abs(a-b) < thres);
}

bool VecPerp(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2){
    return (bool) ApproxEqNum(vec1.dot(vec2), 0, 0.0001);
}

}

}