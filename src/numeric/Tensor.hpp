
#pragma once

#include "DenseTypes.hpp"

namespace neon
{
inline double double_dot(Matrix3 const& a, Matrix3 const& b)
{
    return (a.array() * b.array()).sum();
}

/** @return the volumetric part of the tensor */
inline Matrix3 volumetric(Matrix3 const& a) { return Matrix3::Identity() * a.trace() / 3.0; }

/** @return the deviatoric part of the tensor */
inline Matrix3 deviatoric(Matrix3 const& a) { return a - volumetric(a); }

/** Compute the von Mises stress based on the full stress tensor */
inline double von_mises_stress(Matrix3 const& a)
{
    return std::sqrt(3.0 / 2.0) * deviatoric(a).norm();
}

inline Matrix3 symmetric(Matrix3 const& a) { return 0.5 * (a.transpose() + a); }

/**
 * Compute the velocity gradient given the time derivative of the deformation
 * gradient and the deformation gradient
 */
inline Matrix3 velocity_gradient(Matrix3 const& Fdot, Matrix const& F)
{
    return Fdot * F.inverse();
}

/** Compute the rate of deformation given the velocity gradient */
inline Matrix3 rate_of_deformation(Matrix3 const& L) { return symmetric(L); }

/** Compute the rate of deformation given the velocity gradient */
inline Matrix3 rate_of_deformation(Matrix3 const& F_dot, Matrix3 const& F)
{
    return symmetric(velocity_gradient(F_dot, F));
}

/**
 * I1 returns the coefficient I1, the first stress invariant,
 * which is equal to the trace
 * @return First invariant
 */
inline double I1(Matrix3 const& a) { return a.trace(); }

/**
 * I2 returns the coefficient I2, the second stress invariant,
 * which is calculated by:
 * \f{align*}{
 * I_2 &= \frac{1}{2} \left( (tr \tau)^2 - tr(\tau \tau) \right)
 * \f}
 * @return Second invariant
 */
inline double I2(Matrix3 const& a) { return 0.5 * (std::pow(a.trace(), 2) - (a * a).trace()); }

/** @return Third invariant, which is the determinant of the tensor */
inline double I3(Matrix3 const& a) { return a.determinant(); }

inline Matrix identity_expansion(Matrix const& H, int const nodal_dofs)
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

/*!
 * Handles the representation of common tensors (deviatoric, identity, etc)
 * in Voigt notation suitable for the computation of tensor operations leveraging
 * matrix-vector or matrix-matrix operations.
 * \addtogroup voigt
 * @{
 */
namespace voigt
{
/**
 * Compute the outer product in Voigt notation according to
 * \f$ \mathbb{\mathbf{1} \otimes \mathbf{1}} = \delta_{ij} \delta_{kl} \f$
 */
inline CMatrix I_outer_I()
{
    // clang-format off
    return (CMatrix(6, 6) << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                             1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                             1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    // clang-format on
}

//! Kinetic description of tensor to voigt notation where off diagonal components
//! are multiplied by a factor of two
namespace kinematic
{
/**
 * Convert second order tensor to Voigt notation according to
 * \f$ \begin{bmatrix} \varepsilon_{11} \\ \varepsilon_{22} \\ \varepsilon_{33} \\ 2\varepsilon_{23}
 * \\ 2\varepsilon_{13} \\
 * 2\varepsilon_{12} \end{bmatrix} = \begin{bmatrix} \varepsilon_{11} & \varepsilon_{12} &
 * \varepsilon_{13} \\ \varepsilon_{21} & \varepsilon_{22} & \varepsilon_{23} \\ \varepsilon_{31} &
 * \varepsilon_{32} & \varepsilon_{33} \end{bmatrix} \f$
 */
inline Vector6 to(Matrix3 const& a)
{
    return (Vector6(6) << a(0, 0), a(1, 1), a(2, 2), 2.0 * a(1, 2), 2.0 * a(0, 2), 2.0 * a(0, 1))
        .finished();
}

/**
 * Convert Voigt notation to second order tensor according to
 * \f$  \begin{bmatrix} \varepsilon_{11} & \varepsilon_{12} &
 * \varepsilon_{13} \\ \varepsilon_{21} & \varepsilon_{22} & \varepsilon_{23} \\ \varepsilon_{31} &
 * \varepsilon_{32} & \varepsilon_{33} \end{bmatrix} = \begin{bmatrix} \varepsilon_{11} \\
 * \varepsilon_{22} \\ \varepsilon_{33} \\ 2\varepsilon_{23}
 * \\ 2\varepsilon_{13} \\ 2\varepsilon_{12} \end{bmatrix} \f$
 */
inline Matrix3 from(Vector6 const& a)
{
    // clang-format off
    return (Matrix3(3, 3) <<     a(0), a(5)/2.0, a(4)/2.0,
                             a(5)/2.0,     a(1),     a(3),
                             a(4)/2.0, a(3)/2.0,     a(2)).finished();
    // clang-format on
}

/**
 * Compute the deviatoric tensor in Voigt notation according to
 * \f$ \mathbb{P} = \frac{1}{2}(\delta_{ik} \delta_{jl} + \delta_{il} \delta_{jk}) -
 * \frac{1}{3}\delta_{ij} \delta_{kl} \f$
 */
inline CMatrix deviatoric()
{
    // clang-format off
    return (CMatrix(6, 6) << 2.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0, 0.0, 0.0, 0.0,
                            -1.0 / 3.0,  2.0 / 3.0, -1.0 / 3.0, 0.0, 0.0, 0.0,
                            -1.0 / 3.0, -1.0 / 3.0,  2.0 / 3.0, 0.0, 0.0, 0.0,
                                   0.0,        0.0,        0.0, 0.5, 0.0, 0.0,
                                   0.0,        0.0,        0.0, 0.0, 0.5, 0.0,
                                   0.0,        0.0,        0.0, 0.0, 0.0, 0.5).finished();
    // clang-format on
}

/**
 * Compute the fourth order symmetric identity tensor in Voigt notation according to
 * \f$ \mathbb{I} = \frac{1}{2}(\delta_{ik} \delta_{jl} + \delta_{il} \delta_{jk}) \f$
 */
inline CMatrix fourth_order_identity()
{
    // clang-format off
    return (CMatrix(6, 6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.5).finished();
    // clang-format on
}

/**
 * Compute the fourth order symmetric identity tensor in Voigt notation according to
 * \f$ \mathbb{I} = \frac{1}{2}(\delta_{ik} \delta_{jl} + \delta_{il} \delta_{jk}) \f$
 */
inline CMatrix identity()
{
    // clang-format off
    return (CMatrix(6, 6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.5).finished();
    // clang-format on
}
}

//! Kinetic description of tensor to voigt notation where off diagonal components
//! are not multiplied by a factor of two
namespace kinetic
{
/**
 * Convert second order tensor to Voigt notation according to
 * \f$ \begin{bmatrix} \sigma_{11} \\ \sigma_{22} \\ \sigma_{33} \\ \sigma_{23} \\ \sigma_{13} \\
 * \sigma_{12} \end{bmatrix} = \begin{bmatrix} \sigma_{11} & \sigma_{12} & \sigma_{13} \\
 * \sigma_{21} & \sigma_{22} & \sigma_{23} \\ \sigma_{31} & \sigma_{32} & \sigma_{33} \end{bmatrix}
 * \f$
 */
inline Vector6 to(Matrix3 const& a)
{
    return (Vector6(6) << a(0, 0), a(1, 1), a(2, 2), a(1, 2), a(0, 2), a(0, 1)).finished();
}

/**
 * Convert Voigt notation to second order tensor according to
 * \f$ \begin{bmatrix} \sigma_{11} & \sigma_{12} & \sigma_{13} \\
 * \sigma_{21} & \sigma_{22} & \sigma_{23} \\ \sigma_{31} & \sigma_{32} & \sigma_{33} \end{bmatrix}
 * = \begin{bmatrix} \sigma_{11} \\ \sigma_{22} \\ \sigma_{33} \\ \sigma_{23} \\ \sigma_{13} \\
 * \sigma_{12} \end{bmatrix}
 * \f$
 */
inline Matrix3 from(Vector6 const& a)
{
    // clang-format off
    return (Matrix3(3, 3) << a(0), a(5), a(4),
                             a(5), a(1), a(3),
                             a(4), a(3), a(2)).finished();
    // clang-format on
}

/**
 * Compute the deviatoric tensor in Voigt notation according to
 * \f$ \mathbb{P} = \frac{1}{2}(\delta_{ik} \delta_{jl} + \delta_{il} \delta_{jk}) -
 * \frac{1}{3}\delta_{ij} \delta_{kl} \f$
 */
inline CMatrix deviatoric()
{
    // clang-format off
    return (CMatrix(6, 6) << 2.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0, 0.0, 0.0, 0.0,
                            -1.0 / 3.0,  2.0 / 3.0, -1.0 / 3.0, 0.0, 0.0, 0.0,
                            -1.0 / 3.0, -1.0 / 3.0,  2.0 / 3.0, 0.0, 0.0, 0.0,
                                   0.0,        0.0,        0.0, 1.0, 0.0, 0.0,
                                   0.0,        0.0,        0.0, 0.0, 1.0, 0.0,
                                   0.0,        0.0,        0.0, 0.0, 0.0, 1.0).finished();
    // clang-format on
}

/**
 * Compute the fourth order symmetric identity tensor in Voigt notation according to
 * \f$ \mathbb{I} = \frac{1}{2}(\delta_{ik} \delta_{jl} + \delta_{il} \delta_{jk}) \f$
 */
inline CMatrix fourth_order_identity()
{
    // clang-format off
    return (CMatrix(6, 6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    // clang-format on
}

/**
 * Compute the fourth order symmetric identity tensor in Voigt notation according to
 * \f$ \mathbb{I} = \frac{1}{2}(\delta_{ik} \delta_{jl} + \delta_{il} \delta_{jk}) \f$
 */
inline CMatrix identity()
{
    // clang-format off
    return (CMatrix(6, 6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
    // clang-format on
}
}
}
/*! @} End of Doxygen Groups */

inline CMatrix outer_product(Matrix3 const& a, Matrix3 const& b)
{
    return voigt::kinetic::to(a) * voigt::kinetic::to(b).transpose();
}

inline CMatrix outer_product(Matrix3 const& h) { return outer_product(h, h); }

inline CMatrix mandel_notation(CMatrix A)
{
    A.block<3, 3>(0, 3) *= std::sqrt(2);
    A.block<3, 3>(3, 0) *= std::sqrt(2);
    A.block<3, 3>(3, 3) *= 2.0;
    return A;
}
}
