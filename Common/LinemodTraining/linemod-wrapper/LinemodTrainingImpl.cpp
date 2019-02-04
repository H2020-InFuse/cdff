/**
 * @addtogroup LinemodTraining
 * @{
 */

#include "LinemodTrainingImpl.hpp"
#include "LinemodTemplateGenerator.hpp"
#include "LinemodTemplateGeneratorIteratorSinCos.hpp"

#include <sys/time.h>
#include <iterator>
#include <opencv2/imgcodecs.hpp>

namespace CDFF
{
namespace Common
{
namespace LinemodTraining
{

static void progress_bar(float progress)
{
    int barWidth = 70;
    progress /= 100;
    std::cout << "[";
    int pos = static_cast<int>(barWidth * progress);
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << static_cast<int>(progress * 100) << " %\r";
    std::cout.flush();
}

LinemodBasedPoseDetector::LinemodBasedPoseDetector(): _num_modalities(1), _ui_numberOfTemplates(0),
    _cvptr_detector(), _x_poseMap(), _winW(640), _winH(480), _fx(700.0), _fy(700.0), _cx(320.0), _cy(240.0),
    _saveDir("")
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

void LinemodBasedPoseDetector::SaveTraining(const std::string& str_baseFullPathname)
{
    std::string str_trainingFullpathname = str_baseFullPathname + "_" + std::to_string(_num_modalities) + "_training.dat";
    std::string str_posesFullpathname = str_baseFullPathname + "_" + std::to_string(_num_modalities) + "_poses.dat";

    cv::FileStorage x_cvFileStorage(str_trainingFullpathname, cv::FileStorage::WRITE);

    _cvptr_detector->write(x_cvFileStorage);

    std::vector<cv::String> x_classIds = _cvptr_detector->classIds();
    x_cvFileStorage << "classes" << "[";
    for (unsigned long i = 0; i < x_classIds.size(); ++i)
    {
        x_cvFileStorage << "{";
        _cvptr_detector->writeClass(x_classIds[i], x_cvFileStorage);
        x_cvFileStorage << "}"; // current class
    }
    x_cvFileStorage << "]"; // classes

    std::ofstream x_ofstream(str_posesFullpathname.c_str());
    boost::archive::text_oarchive x_serializer(x_ofstream);
    x_serializer << _x_poseMap;
}

bool LinemodBasedPoseDetector::AddToTemplateBase(const std::string& str_objName, unsigned int ui_objectId, const std::vector<cv::Mat>& sources,
                                                 const cv::Mat& cvmat_mask, const cv::Vec3d& vec_R, const cv::Vec3d& vec_T )
{
    cv::Rect x_bb;
    int i_res = _cvptr_detector->addTemplate(sources, str_objName, cvmat_mask, &x_bb);
    if (i_res < 0){
        return false;
    }
    // store the pose of each template in the SPoseRecord map
    CPoseRecord x_record;
    x_record.ui_objectId = ui_objectId;
    x_record.ui_templateId = static_cast<unsigned int>(i_res);
    x_record.i_bbX = x_bb.x;
    x_record.i_bbY = x_bb.y;
    x_record.i_bbWidth = x_bb.width;
    x_record.i_bbHeight = x_bb.height;
    x_record.d_R1 = vec_R(0);
    x_record.d_R2 = vec_R(1);
    x_record.d_R3 = vec_R(2);
    x_record.d_T1 = vec_T(0);
    x_record.d_T2 = vec_T(1);
    x_record.d_T3 = vec_T(2);
    _x_poseMap[i_res] = x_record;

    return true;
}

void LinemodBasedPoseDetector::Train(const std::string& ply_path, bool isPLY,
                                     float f_lonMin, float f_lonMax, float f_lonStep,
                                     float f_latMin, float f_latMax, float f_latStep,
                                     float f_angleMin, float f_angleMax, float f_angleStep,
                                     float f_radiusMin, float f_radiusMax, float f_radiusStep)
{
    LinemodTemplateGenerator x_tg(ply_path, isPLY, _winW, _winH, _fx, _fy, _cx, _cy);
    LinemodTemplateGeneratorIteratorSinCos x_renderIterator(&x_tg,
                                                            f_lonMin, f_lonMax, f_lonStep,
                                                            f_latMin, f_latMax, f_latStep,
                                                            f_angleMin, f_angleMax, f_angleStep,
                                                            f_radiusMin, f_radiusMax, f_radiusStep);
    std::vector<cv::Mat> sources;
    cv::Mat cvmat_mask;
    cv::Vec3d R;
    cv::Vec3d T;

    int iterationsCount = 0;
    while (!x_renderIterator.isDone())
    {
        ++iterationsCount;
        x_renderIterator.render(sources, cvmat_mask,_num_modalities);
        cv::Affine3d pose = x_renderIterator.getPoseMatrix4();
        LinemodTemplateGenerator::matrix4_to_RT(pose,R,T);
        std::string class_id = "class_MESH";

        bool b_ok = AddToTemplateBase(class_id, 1, sources, cvmat_mask, R, T);
        if (b_ok && !_saveDir.empty())
        {
            std::ostringstream filename;
            filename << _saveDir << "/" << cv::format("Linemod_color_%04d.png", iterationsCount-1);
            cv::imwrite(filename.str(), sources[0]);

            if (_num_modalities == 2) {
                cv::Mat depth_exr;
                sources[1].convertTo(depth_exr, CV_32F);

                filename.str("");
                filename << _saveDir << "/" << cv::format("Linemod_depth_%04d.exr", iterationsCount-1);
                cv::imwrite(filename.str(), depth_exr);
            }
        }

        if (!b_ok){
            std::cout << "Error in inserting template number "<< iterationsCount << "  : we skip this one." << std::endl;
        }
        progress_bar(100*iterationsCount/x_renderIterator.getSize());
        ++x_renderIterator;
    }
}

}
}
}
/** @} */
