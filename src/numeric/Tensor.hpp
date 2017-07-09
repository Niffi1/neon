
#pragma once

#include "DenseTypes.hpp"

namespace neon
{
inline auto double_dot(Matrix3 const& a, Matrix3 const& b) { return (a.array() * b.array()).sum(); }

/** @return the deviatoric part of the tensor */
inline auto deviatoric(Matrix3 const& a) { return a - Matrix3::Identity() * a.trace() / 3.0; }

inline double von_mises_stress(Matrix3 const& a)
{
    return std::sqrt(3.0 / 2.0) * deviatoric(a).norm();
}

inline auto symmetric(Matrix3 const& a) { return 1.0 / 2.0 * (a.transpose() + a); }

inline auto voigt(Matrix3 const& a)
{
    Eigen::Matrix<double, 6, 1> b;
    b << a(0, 0), a(1, 1), a(2, 2), a(1, 2), a(0, 2), a(0, 1);
    return b;
}

inline Matrix3 voigt_to_matrix(Vector const& a)
{
    Matrix3 b;
    b << a(0), a(5), a(4), //
        a(5), a(1), a(3),  //
        a(4), a(3), a(2);
    return b;
}

/**
 * I1 returns the coefficient I1, the first stress invariant,
 * which is equal to the trace
 * @return First invariant
 */
inline auto I1(Matrix3 const& a) { return a.trace(); }

/**
 * I2 returns the coefficient I2, the second stress invariant,
 * which is calculated by:
 * \f{align*}{
 * I_2 &= \frac{1}{2} \left( (tr \tau)^2 - tr(\tau \tau) \right)
 * \f}
 * @return Second invariant
 */
inline auto I2(Matrix3 const& a) { return 0.5 * (std::pow(a.trace(), 2) - (a * a).trace()); }

/** @return Third invariant, which is the determinant of the tensor */
inline auto I3(Matrix3 const& a) { return a.determinant(); }

inline auto identity_expansion(Matrix const& H, int const nodal_dofs)
{
    assert(H.rows() == H.cols());
    // Create the geometric part of the tangent stiffness matrix
    Matrix K = Matrix::Zero(H.rows() * nodal_dofs, H.rows() * nodal_dofs);
    for (auto i = 0; i < H.rows(); ++i)
        for (auto j = 0; j < H.rows(); ++j)
            for (auto k = 0; k < nodal_dofs; ++k)
                K(i * nodal_dofs + k, j * nodal_dofs + k) = H(i, j);
    return K;
}

inline auto fourth_order_identity()
{
    Matrix I(6, 6);
    I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 0.0, 0.5, 0.0, 0.0,  //
        0.0, 0.0, 0.0, 0.0, 0.5, 0.0,  //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
    return I;
}

inline auto I_outer_I()
{
    Matrix i_o_i(6, 6);
    i_o_i << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, //
        1.0, 1.0, 1.0, 0.0, 0.0, 0.0,      //
        1.0, 1.0, 1.0, 0.0, 0.0, 0.0,      //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      //
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    return i_o_i;
}

inline auto outer_product(Matrix3 const& h)
{
    Matrix h_outer_h(6, 6);
    h_outer_h << std::pow(h(0, 0), 2), //
        h(0, 0) * h(1, 1),             //
        h(0, 0) * h(2, 2),             //
        h(0, 0) * h(1, 2),             //
        h(0, 0) * h(0, 2),             //
        h(0, 0) * h(0, 1),             //
        //
        h(0, 0) * h(1, 1),    //
        std::pow(h(1, 1), 2), //
        h(1, 1) * h(2, 2),    //
        h(1, 1) * h(1, 2),    //
        h(0, 2) * h(1, 1),    //
        h(0, 1) * h(1, 1),    //
                              //
        h(0, 0) * h(2, 2),    //
        h(1, 1) * h(2, 2),    //
        std::pow(h(2, 2), 2), //
        h(1, 2) * h(2, 2),    //
        h(0, 2) * h(2, 2),    //
        h(0, 1) * h(2, 2),    //
                              //
        h(0, 0) * h(1, 2),    //
        h(1, 1) * h(1, 2),    //
        h(1, 2) * h(2, 2),    //
        std::pow(h(1, 2), 2), //
        h(0, 2) * h(1, 2),    //
        h(0, 1) * h(1, 2),    //
                              //
        h(0, 0) * h(0, 2),    //
        h(0, 2) * h(1, 1),    //
        h(0, 2) * h(2, 2),    //
        h(0, 2) * h(1, 2),    //
        std::pow(h(0, 2), 2), //
        h(0, 1) * h(0, 2),    //
                              //
        h(0, 0) * h(0, 1),    //
        h(0, 1) * h(1, 1),    //
        h(0, 1) * h(2, 2),    //
        h(0, 1) * h(1, 2),    //
        h(0, 1) * h(0, 2),    //
        std::pow(h(0, 1), 2); //
    return h_outer_h;
}
}
