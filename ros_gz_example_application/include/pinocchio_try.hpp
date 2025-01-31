#ifndef PINOCCHIO_TRY_HPP
#define PINOCCHIO_TRY_HPP

#include <Eigen/Dense>
#include <cmath>

    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A, double epsilon = 1e-6) {
    // SVD 분해
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // 특이값 행렬의 역수 계산
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv(singularValues.size());

    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > epsilon) {  // 작은 값 무시 (수치적 안정성)
            singularValuesInv(i) = 1.0 / singularValues(i);
        } else {
            singularValuesInv(i) = 0.0;
        }
    }

    // Pseudo-Inverse 계산: V * Σ⁻¹ * Uᵀ
    return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    }
    

#endif // PINOCCHIO_TRY_HPP

