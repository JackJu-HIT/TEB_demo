#include "soft_constraint_planner.h"


int main()
{
    // 1. 配置初始化（修改参数只需改这里）
    teb_local_planner::TebConfig cfg;
    cfg.soft_constraint_target = Eigen::Vector2d(1.0, 1.0);  // 软约束目标位置
    cfg.soft_constraint_weight = 10.0;                       // 软约束权重
    cfg.distance_constraint_exp_dist = sqrt(2);              // 距离约束期望距离
    cfg.distance_constraint_weight = 20.0;                  // 距离约束权重
    cfg.max_iterations = 50;                                // 最大迭代次数

    // 2. 创建规划器并运行优化
    teb_local_planner::SoftConstraintPlanner planner(cfg);
    planner.runOptimization();

    return 0;
}