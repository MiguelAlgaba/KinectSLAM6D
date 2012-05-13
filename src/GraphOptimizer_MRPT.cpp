#define ENABLE_DEBUG_GRAPH_MRPT 0 //Don't enable if not necessary

#include "../include/GraphOptimizer_MRPT.h"

#if ENABLE_DEBUG_GRAPH_MRPT
#include "../include/Miscellaneous.h" //Save matrix
#endif

GraphOptimizer_MRPT::GraphOptimizer_MRPT()
{
    //Set the vertex index to 0
    vertexIdx=0;
}

int GraphOptimizer_MRPT::addVertex(Eigen::Matrix4f& vertexPose)
{
    #if ENABLE_DEBUG_GRAPH_MRPT
    char* fileName = (char*) malloc(100);
    std::sprintf(fileName,"../results/matrices/pose_%i.txt",vertexIdx);
    Miscellaneous::saveMatrix(vertexPose,fileName);
    delete fileName;
    #endif

    //Transform the vertex pose from Eigen::Matrix4f to MRPT CPose3D
    mrpt::math::CMatrixDouble44 vertexPoseMRPT;

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            vertexPoseMRPT(i,j)=vertexPose(i,j);
        }
    }
    mrpt::poses::CPose3D p(vertexPoseMRPT);

    //Add the vertex to the graph
    graph.nodes[vertexIdx] = p; vertexIdx++;

    //Return the vertex index
    return vertexIdx-1;
}

void GraphOptimizer_MRPT::addEdge(const int fromIdx,
                                  const int toIdx,
                                  Eigen::Matrix4f& relativePose,
                                  Eigen::Matrix<double,6,6>& informationMatrix)
{

    #if ENABLE_DEBUG_GRAPH_MRPT
    char* fileName = (char*) malloc(100);
    std::sprintf(fileName,"../results/matrices/edge_%i_to_%i.txt",fromIdx,toIdx);
    Miscellaneous::saveMatrix(relativePose,fileName);
    delete fileName;
    #endif

    //Transform the relative pose from Eigen::Matrix4f to MRPT CPose3D
    mrpt::math::CMatrixDouble44 relativePoseMRPT;

    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            relativePoseMRPT(i,j)=relativePose(i,j);
        }
    }
    mrpt::poses::CPose3D rp(relativePoseMRPT);

    //Convert Eigen::Matrix<double,6,6> to mrpt::math::CMatrixDouble66
    mrpt::math::CMatrixDouble66 infMat;
    for(int row=0;row<6;row++)
    {
        for(int col=0;col<6;col++)
        {
            infMat(row,col)=informationMatrix(row,col);
        }
    }

    //Create a 6D pose with uncertainty
    mrpt::poses::CPose3DPDFGaussianInf rpInf = mrpt::poses::CPose3DPDFGaussianInf(rp,infMat);

    //Add the edge to the graph
    /*typename*/ mrpt::graphs::CNetworkOfPoses3DInf::edge_t RelativePose(rpInf);

    graph.insertEdge(mrpt::utils::TNodeID(fromIdx),mrpt::utils::TNodeID(toIdx),RelativePose);
}

void GraphOptimizer_MRPT::optimizeGraph()
{
    // The root node (the origin of coordinates):
    graph.root = mrpt::utils::TNodeID(0);

    mrpt::utils::TParametersDouble  params;
    params["verbose"]  = 0;
    params["profiler"] = 0;
    params["max_iterations"] = 500;
    params["initial_lambda"] = 0.1;
    params["scale_hessian"] = 0.1;
    params["tau"] = 1e-3;

    mrpt::graphslam::TResultInfoSpaLevMarq  levmarq_info;

    // Do the optimization
    mrpt::graphslam::optimize_graph_spa_levmarq(graph,
                                                levmarq_info,
                                                NULL,  // List of nodes to optimize. NULL -> all but the root node.
                                                params);

}

void GraphOptimizer_MRPT::getPoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& poses)
{
    poses.clear();
    poses.resize(graph.nodes.size());

    for(int poseIdx=0;poseIdx<graph.nodes.size();poseIdx++)
    {
        //Transform the vertex pose from MRPT CPose3D to Eigen::Matrix4f
        mrpt::math::CMatrixDouble44 optimizedPoseMRPT;
        graph.nodes[poseIdx].getHomogeneousMatrix(optimizedPoseMRPT);

        Eigen::Matrix4f optimizedPose;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                optimizedPose(i,j)=optimizedPoseMRPT(i,j);
            }
        }

        //Set the optimized pose to the vector of poses
        poses[poseIdx]=optimizedPose;
    }
}

void GraphOptimizer_MRPT::saveGraph(std::string fileName)
{
    //Save the graph to file
    graph.saveToTextFile(fileName);
}
