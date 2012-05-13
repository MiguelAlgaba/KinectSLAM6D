#include "../include/ICPPoseRefiner_StanfordGICP.h"

ICPPoseRefiner_StanfordGICP::ICPPoseRefiner_StanfordGICP()
{
    debug = false;
    gicp_epsilon = 1e-3;
    max_distance = 0.05;
}

void estimateRigidTransform3DStanfordGICP(GICPPointSet& inputGICPPointSet,
                                          GICPPointSet& targetGICPPointSet,
                                          Eigen::Matrix4f& H,
                                          const bool debug,
                                          const double gicp_epsilon,
                                          const double max_distance)
{
    static dgc_transform_t H_GICP_base, H_GICP;
    static int iterations;

    dgc_transform_identity(H_GICP_base);

    // Copy the estimated pose transform matrix from Eigen::Matrix4f to dgc_transform_t
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            H_GICP[i][j]=H(i,j);
        }
    }

    // align the point clouds
    targetGICPPointSet.SetDebug(debug);

    targetGICPPointSet.SetMaxIterationInner(8);

    targetGICPPointSet.SetMaxIteration(70);

    iterations = targetGICPPointSet.AlignScan(&inputGICPPointSet, H_GICP_base, H_GICP, max_distance);

    // Copy the transform matrix from dgc_transform_t to Eigen::Matrix4f
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            H(i,j)=H_GICP[i][j];
        }
    }
}

//ICPPoseRefiner_StanfordGICP::~ICPPoseRefiner_StanfordGICP(){}

void ICPPoseRefiner_StanfordGICP::refinePose(FrameRGBD& frame1,
                                             FrameRGBD& frame2,
                                             Eigen::Matrix4f& H)
{
    estimateRigidTransform3DStanfordGICP(frame1.gicpPointSet,frame2.gicpPointSet,H,debug,gicp_epsilon,max_distance);
}
