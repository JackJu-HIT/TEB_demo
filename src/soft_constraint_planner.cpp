#include "soft_constraint_planner.h"


namespace teb_local_planner
{
    void SoftConstraintPlanner::runOptimization()
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

    boost::shared_ptr<g2o::SparseOptimizer> SoftConstraintPlanner::initOptimizer()
    {
        // 线程安全注册自定义类型
        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once(&registerG2OTypes, flag);

        // 配置求解器（二维顶点，动态残差维度）
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<2, -1>> BlockSolverType;
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
    void SoftConstraintPlanner::registerG2OTypes()
    {
        g2o::Factory* factory = g2o::Factory::instance();
        factory->registerType("VERTEX_POINT2D", new g2o::HyperGraphElementCreator<VertexPoint2D>);
        factory->registerType("EDGE_SOFT_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeSoftConstraint>);
        factory->registerType("EDGE_DISTANCE_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeDistanceConstraint>);
    }



    // 添加顶点（TEB风格：容器管理顶点）
    void SoftConstraintPlanner::AddVertices()
    {
        // 清空旧顶点
        for (auto& v : pose_vertices_)
            delete v;
        pose_vertices_.clear();

        // 顶点1：初始坐标(0,0)
        VertexPoint2D* v1 = new VertexPoint2D();
        v1->setId(0);
        v1->setEstimate(Eigen::Vector2d(0.0, 0.0));
        pose_vertices_.push_back(v1);
        optimizer_->addVertex(v1);

        // 顶点2：初始坐标(2,2)
        VertexPoint2D* v2 = new VertexPoint2D();
        v2->setId(1);
        v2->setEstimate(Eigen::Vector2d(2.0, 2.0));
        pose_vertices_.push_back(v2);
        optimizer_->addVertex(v2);
    }


    // 添加边（TEB风格：new边+setTebConfig+关联顶点）
    void SoftConstraintPlanner::AddEdges()
    {
        // 1. 添加软约束边
        EdgeSoftConstraint* e_soft = new EdgeSoftConstraint();
        e_soft->setId(0);
        e_soft->setVertex(0, pose_vertices_[0]);  // 关联顶点1
        e_soft->setTebConfig(cfg_);              // 传递配置（基类接口）
        e_soft->setInformation(Eigen::Matrix2d::Identity());
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
    bool SoftConstraintPlanner::optimizeGraph()
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



     // 打印优化结果
    void SoftConstraintPlanner::printResult()
    {
        std::cout << "\n===== Optimization Result =====" << std::endl;
        Eigen::Vector2d v1_res = pose_vertices_[0]->estimate();
        Eigen::Vector2d v2_res = pose_vertices_[1]->estimate();
        double final_dist = (v1_res - v2_res).norm();

        std::cout << "顶点1坐标：(" << std::fixed << std::setprecision(4) << v1_res.x() 
                  << ", " << v1_res.y() << ")" << std::endl;
        std::cout << "顶点2坐标：(" << std::fixed << std::setprecision(4) << v2_res.x() 
                  << ", " << v2_res.y() << ")" << std::endl;
        std::cout << "实际距离：" << std::fixed << std::setprecision(4) << final_dist << std::endl;
        std::cout << "期望距离：" << std::fixed << std::setprecision(4) << cfg_.distance_constraint_exp_dist << std::endl;

    }










}