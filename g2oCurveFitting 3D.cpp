#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/factory.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <boost/thread/once.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <boost/make_shared.hpp>
#include <memory>

// -----------------------------------------------------------------------------
// 1. 模拟TEB的TebConfig类（存储全局配置：权重、目标位置等）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
struct TebConfig
{
    // 软约束配置：目标位置 + 权重
    Eigen::Vector3d soft_constraint_target = Eigen::Vector3d(1.0, 1.0,1.0);  //更改维度
    double soft_constraint_weight = 10.0;

    // 距离约束配置：期望距离 + 权重
    double distance_constraint_exp_dist = sqrt(2);  // √2 ≈1.4142
    double distance_constraint_weight = 20.0;

    // 优化器配置
    int max_iterations = 50;
    bool optimization_verbose = true;
};
}  // namespace teb_local_planner

// -----------------------------------------------------------------------------
// 2. TEB风格基类封装（参考TEB的BaseTebUnaryEdge/BaseTebBinaryEdge）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
// 2.1 一元边基类（连接1个顶点）
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
    using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebUnaryEdge() { this->_vertices[0] = nullptr; }

    // 析构函数：从顶点的edges列表中删除当前边（避免野指针）
    virtual ~BaseTebUnaryEdge()
    {
        if (this->_vertices[0])
            this->_vertices[0]->edges().erase(this);
    }

    // 统一误差获取接口：计算误差后返回
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现（满足g2o接口要求，子类可重写）
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口：设置TebConfig
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针（所有子类边可直接访问）
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐
};

// 2.2 二元边基类（连接2个顶点）
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
    using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebBinaryEdge()
    {
        this->_vertices[0] = nullptr;
        this->_vertices[1] = nullptr;
    }

    // 析构函数：从两个顶点的edges列表中删除当前边
    virtual ~BaseTebBinaryEdge()
    {
        if (this->_vertices[0])
            this->_vertices[0]->edges().erase(this);
        if (this->_vertices[1])
            this->_vertices[1]->edges().erase(this);
    }

    // 统一误差获取接口
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace teb_local_planner

// -----------------------------------------------------------------------------
// 3. 自定义顶点（二维点：x,y）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
class VertexPoint2D : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置顶点为原点
    virtual void setToOriginImpl() override { _estimate.setZero(); }

    // 顶点更新：应用增量
    virtual void oplusImpl(const double* update) override
    {
        _estimate += Eigen::Vector3d(update[0], update[1],update[2]);  //更改维度
    }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
};
}  // namespace teb_local_planner

// -----------------------------------------------------------------------------
// 4. 自定义边（继承TEB基类，专注核心误差计算）
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
// 4.1 软约束边（继承TEB一元边基类）
class EdgeSoftConstraint : public BaseTebUnaryEdge<3, Eigen::Vector2d, VertexPoint2D>
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

// 4.2 距离约束边（继承TEB二元边基类）
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
        for (auto& v : pose_vertices_)
            delete v;
        pose_vertices_.clear();
    }

    // 核心：运行优化
    void runOptimization()
    {
        std::cout << "===== TEB Style Optimization Start =====" << std::endl;
        // 1. 创建并添加顶点
        AddVertices();
        // 2. 创建并添加边（传递配置）
        AddEdges();
        // 3. 执行优化
        if (optimizeGraph())
            printResult();
        // 4. 清空图（保留顶点）
        clearGraph();
    }

private:
    // 初始化优化器（注册类型、配置求解器）
    boost::shared_ptr<g2o::SparseOptimizer> initOptimizer()
    {
        // 线程安全注册自定义类型
        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once(&registerG2OTypes, flag);

        // 配置求解器（二维顶点，动态残差维度）
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, -1>> BlockSolverType; //更改维度
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // 创建求解器
        auto linear_solver = std::make_unique<LinearSolverType>();
        auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        // 初始化优化器
        auto optimizer = boost::make_shared<g2o::SparseOptimizer>();
        optimizer->setAlgorithm(solver);
        optimizer->setVerbose(cfg_.optimization_verbose);  // 从配置读取verbose

        return optimizer;
    }

    // 注册g2o类型（顶点+边）
    static void registerG2OTypes()
    {
        g2o::Factory* factory = g2o::Factory::instance();
        factory->registerType("VERTEX_POINT2D", new g2o::HyperGraphElementCreator<VertexPoint2D>);
        factory->registerType("EDGE_SOFT_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeSoftConstraint>);
        factory->registerType("EDGE_DISTANCE_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeDistanceConstraint>);
    }

    // 添加顶点（TEB风格：容器管理顶点）
    void AddVertices()
    {
        // 清空旧顶点
        for (auto& v : pose_vertices_)
            delete v;
        pose_vertices_.clear();

        // 顶点1：初始坐标(0,0)
        VertexPoint2D* v1 = new VertexPoint2D();
        v1->setId(0);
        v1->setEstimate(Eigen::Vector3d(0.0, 0.0,0.0)); //更改维度
        pose_vertices_.push_back(v1);
        optimizer_->addVertex(v1);

        // 顶点2：初始坐标(2,2)
        VertexPoint2D* v2 = new VertexPoint2D();
        v2->setId(1);
        v2->setEstimate(Eigen::Vector3d(2.0, 2.0,2.0)); //更改维度
        pose_vertices_.push_back(v2);
        optimizer_->addVertex(v2);
    }

    // 添加边（TEB风格：new边+setTebConfig+关联顶点）
    void AddEdges()
    {
        // 1. 添加软约束边
        EdgeSoftConstraint* e_soft = new EdgeSoftConstraint();
        e_soft->setId(0);
        e_soft->setVertex(0, pose_vertices_[0]);  // 关联顶点1
        e_soft->setTebConfig(cfg_);              // 传递配置（基类接口）
        e_soft->setInformation(Eigen::Matrix3d::Identity());
        optimizer_->addEdge(e_soft);

        // 2. 添加距离约束边
        EdgeDistanceConstraint* e_dist = new EdgeDistanceConstraint();
        e_dist->setId(1);
        e_dist->setVertex(0, pose_vertices_[0]);  // 关联顶点1
        e_dist->setVertex(1, pose_vertices_[1]);  // 关联顶点2
        e_dist->setTebConfig(cfg_);              // 传递配置
        e_dist->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        optimizer_->addEdge(e_dist);
    }

    // 执行优化
    bool optimizeGraph()
    {
        if (pose_vertices_.empty())
        {
            std::cerr << "Error: No vertices to optimize!" << std::endl;
            return false;
        }

        optimizer_->initializeOptimization();
        int actual_iter = optimizer_->optimize(cfg_.max_iterations);  // 从配置读取最大迭代次数

        if (actual_iter == 0)
        {
            std::cerr << "Optimization failed: No iterations executed!" << std::endl;
            return false;
        }
        return true;
    }

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
    void printResult()
    {
        std::cout << "\n===== Optimization Result =====" << std::endl;
        Eigen::Vector3d v1_res = pose_vertices_[0]->estimate();
        Eigen::Vector3d v2_res = pose_vertices_[1]->estimate();
        double final_dist = (v1_res - v2_res).norm();

        std::cout << "顶点1坐标：(" << std::fixed << std::setprecision(4) << v1_res.x() 
                  << ", " << v1_res.y() << "," <<  v1_res.z()<<  ")" << std::endl;
        std::cout << "顶点2坐标：(" << std::fixed << std::setprecision(4) << v2_res.x() 
                  << ", " << v2_res.y() << "," <<  v2_res.z() << ")" << std::endl;
        std::cout << "实际距离：" << std::fixed << std::setprecision(4) << final_dist << std::endl;
        std::cout << "期望距离：" << std::fixed << std::setprecision(4) << cfg_.distance_constraint_exp_dist << std::endl;
    }

private:
    const TebConfig& cfg_;  // 全局配置（只读）
    boost::shared_ptr<g2o::SparseOptimizer> optimizer_;  // 优化器
    std::vector<VertexPoint2D*> pose_vertices_;          // 顶点容器（TEB风格管理）
};
}  // namespace teb_local_planner

// -----------------------------------------------------------------------------
// 6. 主函数（入口）
// -----------------------------------------------------------------------------
int main()
{
    // 1. 配置初始化（修改参数只需改这里）
    teb_local_planner::TebConfig cfg;
    cfg.soft_constraint_target = Eigen::Vector3d(1.0, 1.0,1.0);  // 软约束目标位置   更改维度
    cfg.soft_constraint_weight = 10.0;                       // 软约束权重
    cfg.distance_constraint_exp_dist = sqrt(2);              // 距离约束期望距离
    cfg.distance_constraint_weight = 20.0;                  // 距离约束权重
    cfg.max_iterations = 50;                                // 最大迭代次数

    // 2. 创建规划器并运行优化
    teb_local_planner::SoftConstraintPlanner planner(cfg);
    planner.runOptimization();

    return 0;
}