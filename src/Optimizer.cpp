#include "Optimizer.hpp"
#include "ConfigManager.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>

void EdgeGPSRelative::computeError() 
{
    const auto* v_i =
        static_cast<const g2o::VertexSE3*>(_vertices[0]);
    const auto* v_j =
        static_cast<const g2o::VertexSE3*>(_vertices[1]);

    Eigen::Vector3d t_i = v_i->estimate().translation();
    Eigen::Vector3d t_j = v_j->estimate().translation();

    Eigen::Vector3d pred = t_j - t_i;

    _error = pred - _measurement;
}

// --------------------
// EdgeGPS
// --------------------
void EdgeGPS::computeError()
{
    const auto* v =
        static_cast<const g2o::VertexSE3*>(_vertices[0]);

    Eigen::Vector3d est = v->estimate().translation();

    // simple translation difference
    _error = est - _measurement;
}

// --------------------
// GraphOptimizer
// --------------------
GraphOptimizer::GraphOptimizer()
{
    using BlockSolverType = g2o::BlockSolverX;
    using LinearSolverType =
        g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>())
    );

    optimizer_.setAlgorithm(solver);
}

// --------------------
// Add Pose
// --------------------
g2o::VertexSE3* GraphOptimizer::AddPose(
    const g2o::SE3Quat& pose,
    bool fixed)
{
    auto* v = new g2o::VertexSE3();

    v->setId(vertex_id_++);
    v->setEstimate(pose);
    v->setFixed(fixed);

    optimizer_.addVertex(v);
    return v;
}

// --------------------
// Add LIO Edge
// --------------------
void GraphOptimizer::AddLIOEdge(g2o::VertexSE3* v1,
                                g2o::VertexSE3* v2,
                                const g2o::SE3Quat& rel_pose)
{
    auto* e = new g2o::EdgeSE3();

    e->setVertex(0, v1);
    e->setVertex(1, v2);

    e->setMeasurement(rel_pose);

    Eigen::Matrix<double, 6, 6> info =
        Eigen::Matrix<double, 6, 6>::Identity();

    info *= ConfigManager::Get().Optimizer_.lio_edge_weight;

    e->setInformation(info);

    optimizer_.addEdge(e);
}

// --------------------
// Add GPS Edge (NO YAW)
// --------------------
void GraphOptimizer::AddGPSEdge(g2o::VertexSE3* v_pose,
                               const Eigen::Vector3d& gps)
{
    auto* e = new EdgeGPS();

    e->setVertex(0, v_pose);
    e->setMeasurement(gps);

    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    info *= ConfigManager::Get().Optimizer_.gps_edge_weight;
    info(2,2) *= 0.01; // less weight on z
    e->setInformation(info);

    // robust kernel (protect against bad GPS)
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(0.5);
    e->setRobustKernel(rk);

    optimizer_.addEdge(e);
}

void GraphOptimizer::AddGPSRelativeEdge(g2o::VertexSE3* v_pose,
                               g2o::VertexSE3* v_prev_pose,
                               const Eigen::Vector3d& gps_relative)
{
    auto* e = new EdgeGPSRelative();

    e->setVertex(0, v_prev_pose);
    e->setVertex(1, v_pose);
    e->setMeasurement(gps_relative);

    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    info *= ConfigManager::Get().Optimizer_.gps_relative_edge_weight;
    info(2,2) *= 0.001; // less weight on z
    e->setInformation(info);

    // robust kernel (protect against bad GPS)
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(0.5);
    e->setRobustKernel(rk);

    optimizer_.addEdge(e);
}

// --------------------
// Optimize
// --------------------
void GraphOptimizer::Optimize(int iterations)
{
    optimizer_.initializeOptimization();
    optimizer_.optimize(iterations);
}