/**
 * @Author:juchunyu@qq.com
*/
#pragma once 
#include "base_teb_edges.h"
#include  "vertexPoint.h"
// -----------------------------------------------------------------------------
// 4. 自定义边（继承TEB基类，专注核心误差计算）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
// 4.1 软约束边（继承TEB一元边基类）
class EdgeSoftConstraint : public BaseTebUnaryEdge<2, Eigen::Vector2d, VertexPoint2D>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 核心：误差计算（直接使用基类的cfg_获取配置）
    virtual void computeError() override
    {
        // 检查配置是否已设置
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeSoftConstraint!" << std::endl;
            _error.setZero();
            return;
        }

        // 获取关联的顶点
        const VertexPoint2D* v = static_cast<const VertexPoint2D*>(_vertices[0]);
        // 误差公式：权重 × (当前位置 - 目标位置)（从cfg_读取参数）
        _error = cfg_->soft_constraint_weight * (v->estimate() - cfg_->soft_constraint_target);
    }
};
}
