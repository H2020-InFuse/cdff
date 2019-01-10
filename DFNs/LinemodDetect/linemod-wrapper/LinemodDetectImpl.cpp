/**
 * @addtogroup DFNs
 * @{
 */

#include "LinemodDetectImpl.hpp"

#include <sys/time.h>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/linemod.hpp>

namespace CDFF
{
namespace DFN
{
namespace LinemodDetect
{

LinemodBasedPoseDetector::LinemodBasedPoseDetector(): _num_modalities(1), _ui_numberOfTemplates(0),
    _cvptr_detector(), _x_poseMap(), _winW(640), _winH(480), _fx(700.0), _fy(700.0), _cx(320.0), _cy(240.0)
{
}

LinemodBasedPoseDetector::~LinemodBasedPoseDetector()
{
}

void LinemodBasedPoseDetector::InitAs2D(int T_level0, int T_level1)
{
    int T_levels[] = {T_level0, T_level1};
    std::vector< cv::Ptr<cv::linemod::Modality> > modalities;
    modalities.push_back(cv::makePtr<cv::linemod::ColorGradient>(10,63,55)); // weak_threshold, num_features, strong_threshold
    _cvptr_detector= cv::makePtr<cv::linemod::Detector>(modalities, std::vector<int>(T_levels, T_levels + 2));
    _num_modalities = static_cast<int>(_cvptr_detector->getModalities().size());
}

void LinemodBasedPoseDetector::InitAs3D(int T_level0, int T_level1)
{
    int T_levels[] = {T_level0, T_level1};
    std::vector< cv::Ptr<cv::linemod::Modality> > modalities;
    modalities.push_back(cv::makePtr<cv::linemod::ColorGradient>(10,63,55)); // weak_threshold, num_features, strong_threshold
    modalities.push_back(cv::makePtr<cv::linemod::DepthNormal>(1000000, 500, 63,2)); // distance_threshold, _difference_threshold, _num_features, _extract_threshold
    _cvptr_detector= cv::makePtr<cv::linemod::Detector>(modalities, std::vector<int>(T_levels, T_levels + 2));
    _num_modalities = static_cast<int>(_cvptr_detector->getModalities().size());
}

bool LinemodBasedPoseDetector::LoadTraining(const std::string& str_baseFullPathname)
{
    std::string str_trainingFullpathname = str_baseFullPathname + "_" + std::to_string(_num_modalities) + "_training.dat";
    std::string str_posesFullpathname = str_baseFullPathname + "_" + std::to_string(_num_modalities) + "_poses.dat";

    cv::FileStorage fs(str_trainingFullpathname, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    else
    {
        _cvptr_detector->read(fs.root());
        cv::FileNode fn = fs["classes"];
        for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
            _cvptr_detector->readClass(*i);

        std::vector<cv::String> ids = _cvptr_detector->classIds();
        int num_classes = _cvptr_detector->numClasses();
        printf("Loaded %d classes and %d templates\n", num_classes, _cvptr_detector->numTemplates());
        if (!ids.empty())
        {
            printf("Class ids:\n");
            std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
        }
    }

    _x_poseMap.clear();
    std::ifstream x_ifstream(str_posesFullpathname.c_str());
    if(!x_ifstream.is_open())
        return false;
    else
    {
        boost::archive::text_iarchive x_serializer(x_ifstream);
        x_serializer >> _x_poseMap;
        _ui_numberOfTemplates = static_cast<unsigned int>(_x_poseMap.size());
    }
    return true;
}

void LinemodBasedPoseDetector::drawResponse(const std::vector<cv::linemod::Template>& templates,
                                            int num_modalities, cv::Mat& dst, const cv::Point& offset, int T)
{
    (void)(T);

    static const cv::Scalar COLORS[5] = { CV_RGB(255, 0, 0),
                                          CV_RGB(0, 0, 255),
                                          CV_RGB(255, 255, 0),
                                          CV_RGB(255, 140, 0),
                                          CV_RGB(255, 0, 0) };

    for (int m = 0; m < num_modalities; m++)
    {
        cv::Scalar color = COLORS[m];

        for (size_t i = 0; i < templates[m].features.size(); i++)
        {
            cv::linemod::Feature f = templates[m].features[i];
            cv::Point pt(f.x + offset.x, f.y + offset.y);
            cv::circle(dst, pt, 2, color);
        }
    }
}

int LinemodBasedPoseDetector::getT(int pyramid_level) const {
    return _cvptr_detector->getT(pyramid_level);
}

const std::vector<cv::linemod::Template>& LinemodBasedPoseDetector::getTemplates (const cv::String &class_id, int template_id) const {
    return _cvptr_detector->getTemplates(class_id,template_id);
}

bool LinemodBasedPoseDetector::Detect(const std::vector<cv::Mat>& sources,
                                      float matching_threshold,
                                      cv::linemod::Match& x_match,
                                      cv::Rect &x_bb,
                                      cv::Vec3d& vec_R, cv::Vec3d& vec_T)
{
    std::vector<cv::linemod::Match> matches;
    std::vector<cv::String> class_ids;

    _cvptr_detector->match(sources, matching_threshold, matches, class_ids);

    if (matches.empty()) {
        std::cout <<"No matches found ... \n" <<std::endl;
        return false;
    }

    float maxSimilarity = 0.0;
    for (size_t i = 0; i < matches.size(); i++)
    {
        cv::linemod::Match m = matches[i];
        if (m.similarity > maxSimilarity)
        {
            x_match = m;
            maxSimilarity = m.similarity;
        }
    }

    CPoseRecord pose = _x_poseMap[x_match.template_id];
    x_bb = cv::Rect(pose.i_bbX, pose.i_bbY, pose.i_bbWidth, pose.i_bbHeight);
    vec_R = cv::Vec3d(pose.d_R1, pose.d_R2, pose.d_R3);
    vec_T = cv::Vec3d(pose.d_T1, pose.d_T2, pose.d_T3);

    return true;
}

}
}
}

/** @} */
