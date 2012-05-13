#include <mrpt/base.h>
#include <mrpt/graphslam.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/graphs/CNetworkOfPoses.h>

#include "GraphOptimizer.h"

/*!This class encapsulates the functionality of the MRPT graph-slam module to perform 6D graph optimization.*/
class GraphOptimizer_MRPT : public GraphOptimizer
{
private:
  int vertexIdx;
  mrpt::graphs::CNetworkOfPoses3DInf graph;

public:
  GraphOptimizer_MRPT();
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
