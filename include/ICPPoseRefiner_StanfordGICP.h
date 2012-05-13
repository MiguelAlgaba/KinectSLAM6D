#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/tokenizer.hpp>
#include "gicp.h"

#include "ICPPoseRefiner.h"

using namespace std;
using namespace dgc::gicp;
namespace po = boost::program_options;

/*!This class encapsulates the functionality of a 3D rigid transformation refiner using the Generalized ICP algorithm implemented in the Stanford GICP library. The source code of the GICP library could be found here: http://www.stanford.edu/~avsegal/generalized_icp.html*/
class ICPPoseRefiner_StanfordGICP : public ICPPoseRefiner
{
private:
    bool debug;
    double gicp_epsilon;
    double max_distance;
    GICPPointSet gicpPointSet1;
    GICPPointSet gicpPointSet2;

public:
  ICPPoseRefiner_StanfordGICP();
  /*!This method refines the 3D rigid transformation between frame1 and frame2 giving H as an initial guess. The refined rigid transformation will be returned in H.*/
  void refinePose(FrameRGBD& frame1,
                  FrameRGBD& frame2,
                  Eigen::Matrix4f& H);
};
