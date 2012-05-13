#ifndef GRAPH_OPTIMIZER
#define GRAPH_OPTIMIZER

#include <vector>
#include <Eigen/Core>
#include <string>

/*!Abstract class that defines the mandatory methods that a 6D GraphSLAM optimizer must implement.*/
class GraphOptimizer
{
public:
  /*!Adds a new vertex to the graph. The provided 4x4 matrix will be considered as the pose of the new added vertex. It returns the index of the added vertex.*/
  virtual int addVertex(Eigen::Matrix4f& pose)=0;
  /*!Adds an edge that defines a spatial constraint between the vertices "fromIdx" and "toIdx" with information matrix that determines the weight of the added edge.*/
  virtual void addEdge(const int fromIdx,const int toIdx,Eigen::Matrix4f& relPose,Eigen::Matrix<double,6,6>& infMatrix)=0;
  /*!Calls the graph optimization process to determine the pose configuration that best satisfies the constraints defined by the edges.*/
  virtual void optimizeGraph()=0;
  /*!Returns a vector with all the optimized poses of the graph.*/
  virtual void getPoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&)=0;
  /*!Saves the graph to file.*/
  virtual void saveGraph(std::string fileName)=0;
};

#endif
