#ifndef _SG_FILTER_HPP_
#define _SG_FILTER_HPP_

// Eigen
#include <Eigen/Dense>

class SGFilter {
public:
    SGFilter() = default;
    ~SGFilter() = default;
    Eigen::VectorXd smooth(const Eigen::VectorXd& curve, int window_size, int poly_order) {
        // 1. 参数合法性校验
        if (curve.size() < 2) {
            throw std::invalid_argument("Curve length must be greater than 1");
        }
        if (window_size % 2 == 0) { // 强制窗口为奇数
            window_size += 1;
        }
        if (poly_order >= window_size) {
            throw std::invalid_argument("Polynomial order must be less than window size");
        }

        const int n = curve.size();
        const int m = (window_size - 1) / 2; // 窗口半宽
        Eigen::VectorXd smoothed(n);

        // 2. 构造拟合矩阵A（窗口内的位置权重矩阵）
        Eigen::MatrixXd A(window_size, poly_order + 1);
        for (int i = 0; i < window_size; ++i) {
            const int x = i - m; // 窗口内相对位置（-m ~ +m）
            double pow_x = 1.0;
            for (int j = 0; j <= poly_order; ++j) {
                A(i, j) = pow_x;
                pow_x *= x;
            }
        }

        // 3. 预计算最小二乘系数：(A^T A)^-1 A^T
        const Eigen::MatrixXd AT = A.transpose();
        const Eigen::MatrixXd ATA_inv = (AT * A).inverse();
        const Eigen::MatrixXd coeff_matrix = ATA_inv * AT;

        // 4. 逐点计算平滑值（边界镜像延拓）
        for (int i = 0; i < n; ++i) {
            Eigen::VectorXd window_vals(window_size);
            // 填充窗口数据（镜像延拓处理边界）
            for (int k = 0; k < window_size; ++k) {
                const int idx = i - m + k; // 窗口内绝对索引
                // 镜像延拓：超出边界时取对称位置
                int extended_idx = idx;
                if (extended_idx < 0) {
                    extended_idx = -extended_idx - 1;
                }
                else if (extended_idx >= n) {
                    extended_idx = 2 * n - extended_idx - 1;
                }
                window_vals(k) = curve(extended_idx);
            }

            // 5. 计算拟合值（取常数项系数，对应x=0的拟合值）
            smoothed(i) = coeff_matrix.row(0).dot(window_vals);
        }

        return smoothed;
    }
private:
};

#endif // ! _SG_FILTER_HPP_