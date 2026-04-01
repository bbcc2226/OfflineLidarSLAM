#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>

// --------------------
// Vertex: Yaw (1 DOF)
// --------------------
class VertexYaw : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void setToOriginImpl() override;
    void oplusImpl(const double* update) override;

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

// --------------------
// Edge: GPS (Pose + Yaw)
// --------------------
class EdgeGPSYaw : public g2o::BaseBinaryEdge<
    3, Eigen::Vector3d,
    g2o::VertexSE3,
    VertexYaw>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void computeError() override;

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

// --------------------
// Optimizer wrapper
// --------------------
class GraphOptimizer
{
public:
    GraphOptimizer();

    // add nodes
    g2o::VertexSE3* AddPose(const g2o::SE3Quat& pose, bool fixed = false);
    VertexYaw* AddYaw(double init_yaw);

    // add edges LIO
    void AddLIOEdge(g2o::VertexSE3* v1,
                    g2o::VertexSE3* v2,
                    const g2o::SE3Quat& rel_pose);
    
    // add edges GPS
    void AddGPSEdge(g2o::VertexSE3* v_pose,
                    VertexYaw* v_yaw,
                    const Eigen::Vector3d& gps);

    // optimize
    void Optimize(int iterations = 20);

    // getters
    double GetYaw() const;

private:
    g2o::SparseOptimizer optimizer_;
    int vertex_id_ = 0;

    VertexYaw* yaw_node_ = nullptr;
};