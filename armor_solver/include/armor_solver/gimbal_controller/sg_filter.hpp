#ifndef _SG_FILTER_HPP_
#define _SG_FILTER_HPP_

// Eigen
#include <Eigen/Dense>

class SGFilter {
public:
    SGFilter() = default;
    ~SGFilter() = default;
    Eigen::VectorXd savgol_filter(const Eigen::VectorXd& input, int window_len, int poly_order) {
        // 步骤1：计算SG卷积系数
        Eigen::VectorXd sg_coeffs = cal_SG_coefficients(window_len, poly_order);
        const int m = (window_len - 1) / 2; // 窗口半宽

        // 步骤2：边界扩展（镜像法，避免首尾数据失真）
        std::vector<double> extended_input;
        // 前扩展：镜像复制前m个点（如[1,2,3] → [3,2,1,1,2,3]）
        for (int i = m; i > 0; --i) {
            extended_input.push_back(input[i - 1]);
        }
        // 原始数据
        extended_input.insert(extended_input.end(), input.begin(), input.end());
        // 后扩展：镜像复制后m个点
        for (int i = input.size() - 2; i >= static_cast<int>(input.size()) - 1 - m; --i) {
            extended_input.push_back(input[i]);
        }

        // 步骤3：滑动窗口卷积
        Eigen::VectorXd output = Eigen::VectorXd::Zero(input.size());
        const int extended_size = extended_input.size();

        for (int i = 0; i < input.size(); ++i) {
            // 扩展后数据的窗口起始位置（对应原始数据的i位置）
            int start = i;
            double val = 0.0;
            // 卷积计算：系数 × 窗口内数据
            for (int j = 0; j < window_len; ++j) {
                val += sg_coeffs[j] * extended_input[start + j];
            }
            output[i] = val;
        }

        return output;
    }
private:
    Eigen::VectorXd cal_SG_coefficients(int N, int k) {
        // 参数校验
        if (N % 2 == 0) {
            throw std::invalid_argument("窗口长度N必须为奇数");
        }
        if (k >= N) {
            throw std::invalid_argument("多项式阶数k必须小于窗口长度N");
        }

        const int m = (N - 1) / 2; // 窗口半宽（中心到边缘的点数）
        const int poly_size = k + 1; // 多项式系数数量（阶数+1）

        // 1. 构造范德蒙矩阵V：N行 × poly_size列
        Eigen::MatrixXd V(N, poly_size);
        for (int i = 0; i < N; ++i) {
            const int x = i - m; // 窗口内相对位置（-m, -(m-1), ..., 0, ..., m-1, m）
            for (int j = 0; j < poly_size; ++j) {
                V(i, j) = std::pow(x, j); // x^0, x^1, ..., x^k
            }
        }

        // 2. 最小二乘求解：V * a = e (e是中心位置的单位向量)
        // 目标：找到系数a，使得V*a ≈ [0,0,...,1,...,0,0]^T（中心位置为1）
        Eigen::VectorXd e = Eigen::VectorXd::Zero(N);
        e(m) = 1.0; // 中心位置对应单位向量

        // SVD分解求解最小二乘（Ax=b → x = A^+b，A^+是伪逆）
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd coeffs = svd.solve(e);

        // 3. 计算最终的SG卷积系数：V * coeffs（即窗口内各点的权重）
        return V * coeffs;
    }
};

#endif // ! _SG_FILTER_HPP_