#include "Optimizer.hpp"
#include "ConfigManager.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
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

void EdgeGPSWithHeading::computeError() 
{
    const auto* v =
        static_cast<const g2o::VertexSE3*>(_vertices[0]);

    Eigen::Vector3d t = v->estimate().translation();
    Mat3 R = v->estimate().rotation();
    Se3 est_pose(R, t);

    _error.head<3>() = (_measurement.so3().inverse() * est_pose.so3()).log();
    _error.tail<3>() = est_pose.translation() - _measurement.translation();
}


// void EdgeGPSWithHeading::linearizeOplus() {
//     const auto* v =
//         static_cast<const g2o::VertexSE3*>(_vertices[0]);

//     Eigen::Vector3d t = v->estimate().translation();
//     Mat3 R = v->estimate().rotation();
//     Se3 est_pose(R, t);
//     // jacobian 6x6
//     _jacobianOplusXi.setZero();
//     _jacobianOplusXi.block<3, 3>(0, 0) = (_measurement.so3().inverse() * est_pose.so3()).jr_inv();  // dR/dR
//     _jacobianOplusXi.block<3, 3>(3, 3) = Mat3::Identity();                                              // dp/dp
// }

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
        g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

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
    info(2,2) = 0.0001; // less weight on z
    e->setInformation(info);

    optimizer_.addEdge(e);
}

// --------------------
// Add Loop Closure Edge
// --------------------
void GraphOptimizer::AddLoopClosureEdge(g2o::VertexSE3* v_hist,
                                        g2o::VertexSE3* v_curr,
                                        const g2o::SE3Quat& rel_pose)
{
    auto* e = new g2o::EdgeSE3();

    e->setVertex(0, v_hist);
    e->setVertex(1, v_curr);
    e->setMeasurement(rel_pose);

    // In the rotational part, x/y/z roughly correspond to roll/pitch/yaw for small errors.
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
    const double t_w = ConfigManager::Get().Optimizer_.loop_closure_edge_weight;
    info(0,0) = t_w * 0.6;  info(1,1) = t_w * 0.6;  info(2,2) = t_w * 0.6;  // roll/pitch low, yaw high
    info(3,3) = t_w;  info(4,4) = t_w;  info(5,5) = t_w * 0.01; // xy strong, z weak
    e->setInformation(info);

    // Huber robust kernel to guard against bad loop closure matches
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(100.0);
    e->setRobustKernel(rk);

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
    info(2,2) = 100.0; // less weight on z
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
    info(2,2) = ConfigManager::Get().Optimizer_.gps_relative_edge_weight * 0.01; // z far less reliable than xy
    e->setInformation(info);

    // robust kernel (protect against bad GPS)
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(0.5);
    e->setRobustKernel(rk);

    optimizer_.addEdge(e);
}

void GraphOptimizer::AddGPSWithHeadingEdge(g2o::VertexSE3* v_pose,
                               const Se3& gps_with_heading)
{
    auto* e = new EdgeGPSWithHeading();

    e->setVertex(0, v_pose);
    e->setMeasurement(gps_with_heading);

    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
    // info *= ConfigManager::Get().Optimizer_.gps_with_heading_edge_weight;
    info(2,2) = 100; // less weight on z
    e->setInformation(info);

    // robust kernel (protect against bad GPS)
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(0.5);
    e->setRobustKernel(rk);

    optimizer_.addEdge(e);
}
// --------------------
// Clear
// --------------------
void GraphOptimizer::Clear()
{
    optimizer_.clear();
    vertex_id_ = 0;
}

// --------------------
// Optimize
// --------------------
void GraphOptimizer::Optimize(int iterations)
{
    std::cout << "[GraphOptimizer] Starting optimization: "
              << optimizer_.vertices().size() << " vertices, "
              << optimizer_.edges().size() << " edges, "
              << iterations << " iterations\n";
    optimizer_.initializeOptimization();
    optimizer_.setVerbose(true);
    optimizer_.optimize(iterations);
    optimizer_.setVerbose(false);
    std::cout << "[GraphOptimizer] Optimization done.\n";
}