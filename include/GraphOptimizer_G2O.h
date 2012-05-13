#include "GraphOptimizer.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/types_six_dof_quat.h"

/*!This class encapsulates the functionality of the G2O library to perform 6D graph optimization.*/
class GraphOptimizer_G2O : public GraphOptimizer
{
private:
    int vertexIdx;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX * solver_ptr;
public:
  GraphOptimizer_G2O();
  /*!Adds a new vertex to the graph. The provided 4x4 matrix will be considered as the pose of the new added vertex. It returns the index of the added vertex.*/
  int addVertex(Eigen::Matrix4f& pose);
  /*!Adds an edge that defines a spatial constraint between the vertices "fromIdx" and "toIdx" with information matrix that determines the weight of the added edge.*/
  void addEdge(const int fromIdx,const int toIdx,Eigen::Matrix4f& relPose,Eigen::Matrix<double,6,6>& infMatrix);
  /*!Calls the graph optimization process to determine the pose configuration that best satisfies the constraints defined by the edges.*/
  void optimizeGraph();
  /*!Returns a vector with all the optimized poses of the graph.*/
  void getPoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&);
  /*!Saves the graph to file.*/
  void saveGraph(std::string fileName);
};
