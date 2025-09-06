/**
 * @Author:juchunyu@qq.com
*/

#pragma once 
#include "base_teb_edges.h"
#include  "vertexPoint.h"

// 4.2 距离约束边（继承TEB二元边基类）
namespace teb_local_planner
{
class EdgeDistanceConstraint : public BaseTebBinaryEdge<1, double, VertexPoint2D, VertexPoint2D>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 核心：误差计算（使用基类的cfg_获取配置）
    virtual void computeError() override
    {
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeDistanceConstraint!" << std::endl;
            _error[0] = 0.0;
            return;
        }

        // 获取关联的两个顶点
        const VertexPoint2D* v1 = static_cast<const VertexPoint2D*>(_vertices[0]);
        const VertexPoint2D* v2 = static_cast<const VertexPoint2D*>(_vertices[1]);
        // 计算实际距离
        double act_dist = (v1->estimate() - v2->estimate()).norm();
        // 误差公式：权重 × (实际距离 - 期望距离)（从cfg_读取参数）
        _error[0] = cfg_->distance_constraint_weight * (act_dist - cfg_->distance_constraint_exp_dist);
    }
};
}  // namespace teb_local_planner
