#include "Optimizer.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>

// --------------------
// VertexYaw
// --------------------
void VertexYaw::setToOriginImpl() {
    _estimate = 0.0;
}

void VertexYaw::oplusImpl(const double* update) {
    _estimate += update[0];

    // normalize angle
    if (_estimate > M_PI) _estimate -= 2*M_PI;
    if (_estimate < -M_PI) _estimate += 2*M_PI;
}

// --------------------
// EdgeGPSYaw
// --------------------
void EdgeGPSYaw::computeError(){
    const auto* v_pose =
        static_cast<const g2o::VertexSE3*>(_vertices[0]);

    const auto* v_yaw =
        static_cast<const VertexYaw*>(_vertices[1]);

    double theta = v_yaw->estimate();

    Eigen::Matrix3d Rz =
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d gps_rot = Rz * _measurement;

    Eigen::Vector3d lio = v_pose->estimate().translation();

    _error = lio - gps_rot;
}

// --------------------
// GraphOptimizer
// --------------------
GraphOptimizer::GraphOptimizer()
{
    using BlockSolverType = g2o::BlockSolver< g2o::BlockSolverTraits<6,1> >;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
    );

    optimizer_.setAlgorithm(solver);
}

// --------------------
// Add Pose
// --------------------
g2o::VertexSE3* GraphOptimizer::AddPose(const g2o::SE3Quat& pose, bool fixed)
{
    auto* v = new g2o::VertexSE3();

    v->setId(vertex_id_++);
    v->setEstimate(pose);
    v->setFixed(fixed);

    optimizer_.addVertex(v);
    return v;
}

// --------------------
// Add Yaw
// --------------------
VertexYaw* GraphOptimizer::AddYaw(double init_yaw)
{
    yaw_node_ = new VertexYaw();
    yaw_node_->setId(vertex_id_++);
    yaw_node_->setEstimate(init_yaw);

    optimizer_.addVertex(yaw_node_);
    return yaw_node_;
}

// --------------------
// LIO Edge
// --------------------
void GraphOptimizer::AddLIOEdge(g2o::VertexSE3* v1,
                                g2o::VertexSE3* v2,
                                const g2o::SE3Quat& rel_pose)
{
    auto* e = new g2o::EdgeSE3();

    e->setVertex(0, v1);
    e->setVertex(1, v2);

    e->setMeasurement(rel_pose);

    Eigen::Matrix<double,6,6> info = Eigen::Matrix<double,6,6>::Identity();
    info *= 100; // strong

    e->setInformation(info);

    optimizer_.addEdge(e);
}

// --------------------
// GPS Edge
// --------------------
void GraphOptimizer::AddGPSEdge(g2o::VertexSE3* v_pose,
                                VertexYaw* v_yaw,
                                const Eigen::Vector3d& gps)
{
    auto* e = new EdgeGPSYaw();

    e->setVertex(0, v_pose);
    e->setVertex(1, v_yaw);

    e->setMeasurement(gps);

    Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
    info *= 1.0; // weaker than LIO

    e->setInformation(info);

    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(1.0);
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

// --------------------
// Get yaw
// --------------------
double GraphOptimizer::GetYaw() const
{
    if (!yaw_node_) return 0.0;
    return yaw_node_->estimate();
}