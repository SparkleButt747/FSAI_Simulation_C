#include <iostream>
#include <vector>
#include <Eigen/Core>

#include "centroid.hpp" 


int main() {
    int num_cones_to_predict = 0;
    std::cin >> num_cones_to_predict;

    // 用于存储所有 N 个预测结果
    std::vector<Eigen::Vector2d> all_predicted_centers;
    all_predicted_centers.reserve(num_cones_to_predict);

    // 1. 循环读取 N 个锥桶的数据
    for (int i = 0; i < num_cones_to_predict; ++i) {
        
        // 存储当前锥桶的 5 个 3D 样本点
        std::vector<Eigen::Vector3d> observed_points;
        observed_points.reserve(5);
        
        // 2. 循环读取 5 个样本点
        for (int j = 0; j < 5; ++j) {
            double x, y, z;
            std::cin >> x >> y >> z;
            observed_points.push_back(Eigen::Vector3d(x, y, z));
        }

        // 3. 运行拟合 (调用你头文件里的函数)
        // (a) 线性初始猜测
        Eigen::Vector2d linear_guess = Centroid::centroid_linear(observed_points);
        
        // (b) 非线性 LM 优化
        Eigen::Vector2d nonlinear_fit = Centroid::centroid_nonlinear(
            observed_points, 
            linear_guess
        );

        // 4. 存储结果
      //   all_predicted_centers.push_back(nonlinear_fit);
            all_predicted_centers.push_back(linear_guess);
    }

    // 5. 循环结束, 一次性输出所有结果
    for (size_t i = 0; i < all_predicted_centers.size(); ++i) {
        std::cout << all_predicted_centers[i].transpose() << std::endl;
    }

    return 0;
}