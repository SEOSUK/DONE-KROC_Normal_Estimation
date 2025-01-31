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
    

    // DLS 기반 유사역행렬 계산 함수
    Eigen::MatrixXd dampedLeastSquares(const Eigen::MatrixXd& J, double lambda) {
        int rows = J.rows();
        int cols = J.cols();
        
        if (rows >= cols) {
            // J이 m×n (m >= n, 일반적인 case)
            return (J.transpose() * J + lambda * lambda * Eigen::MatrixXd::Identity(cols, cols)).inverse() * J.transpose();
        } else {
            // J이 m×n (m < n, underactuated case)
            return J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(rows, rows)).inverse();
        }
    }


#endif // PINOCCHIO_TRY_HPP

