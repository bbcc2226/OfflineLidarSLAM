#pragma once

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <Eigen/Core>



class EdgeGPSRelative: public g2o::BaseBinaryEdge<3, Eigen::Vector3d,
                                 g2o::VertexSE3, g2o::VertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override ;

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }

};
// --------------------
// GPS Edge (NO YAW)
// --------------------
class EdgeGPS : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override;

    bool read(std::istream&) override { return false; }
    bool write(std::ostream&) const override { return false; }
};

// --------------------
// Graph Optimizer
// --------------------
class GraphOptimizer
{
public:
    GraphOptimizer();

    // add pose node
    g2o::VertexSE3* AddPose(const g2o::SE3Quat& pose, bool fixed = false);

    // LIO constraint
    void AddLIOEdge(g2o::VertexSE3* v1,
                    g2o::VertexSE3* v2,
                    const g2o::SE3Quat& rel_pose);

    // GPS constraint (NO YAW)
    void AddGPSEdge(g2o::VertexSE3* v_pose,
                    const Eigen::Vector3d& gps);


    void AddGPSRelativeEdge(g2o::VertexSE3* v_pose,
                    g2o::VertexSE3* v_prev_pose,
                    const Eigen::Vector3d& gps_relative);

    // optimize
    void Optimize(int iterations = 10);

    // remove the vertex
    void RemoveVertex(g2o::VertexSE3* v) {
        if (!v) return;
        // 1. Collect all connected edges
        std::vector<g2o::HyperGraph::Edge*> edges_to_remove;
        for (auto* e : v->edges()) {
            edges_to_remove.push_back(e);
        }
        // 2. Remove and delete all edges
        for (auto* e : edges_to_remove) {
            optimizer_.removeEdge(e);
        }
        // 3. Remove and delete the vertex itself
        optimizer_.removeVertex(v);
        //delete v;
    }

private:
    g2o::SparseOptimizer optimizer_;
    int vertex_id_ = 0;
};