/**
 * @Author:juchunyu@qq.com
*/

#pragma once 
#include "base_teb_edges.h"
#include  "vertexPoint.h"
#include "edge_distance_constraint.h"
#include "edge_soft_constraint.h"

// -----------------------------------------------------------------------------
// 5. 优化器管理类（TEB风格：封装配置、顶点、边、优化逻辑）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
class SoftConstraintPlanner
{
public:
    // 构造函数：初始化配置、优化器、顶点容器
    SoftConstraintPlanner(const TebConfig& cfg)
        : cfg_(cfg)
    {
        optimizer_ = initOptimizer();
        pose_vertices_.clear();
    }

    // 析构函数：释放顶点内存
    ~SoftConstraintPlanner()
    {
       clearGraph();
    }

    // 核心：运行优化
    void runOptimization();
private:
    // 初始化优化器（注册类型、配置求解器）
    boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();

    // 注册g2o类型（顶点+边）
    static void registerG2OTypes();

    // 添加顶点（TEB风格：容器管理顶点）
    void AddVertices();


    // 添加边（TEB风格：new边+setTebConfig+关联顶点）
    void AddEdges();

    // 执行优化
    bool optimizeGraph();

    // 清空图（保留顶点，仅删除边）
    void clearGraph()
    {
        if (optimizer_)
        {
            optimizer_->edges().clear();
            optimizer_->clear();
        }
    }

    // 打印优化结果
    void printResult();

private:
    const TebConfig& cfg_;  // 全局配置（只读）
    boost::shared_ptr<g2o::SparseOptimizer> optimizer_;  // 优化器
    std::vector<VertexPoint2D*> pose_vertices_;          // 顶点容器（TEB风格管理）
};
}  // namespace teb_local_planner
