/**
 * @addtogroup LinemodTraining
 * @{
 */

#ifndef LINEMODTRAINING_LINEMODTRAINIMPL_HPP
#define LINEMODTRAINING_LINEMODTRAINIMPL_HPP

#include <opencv2/core.hpp>
#include <opencv2/rgbd/linemod.hpp>

#include <boost/serialization/map.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace CDFF
{
namespace Common
{
namespace LinemodTraining
{

/*!
 * @brief This class implements the Linemod algorithm [Hinterstoisser2012].
 * It is used to detect an object and retrieve its pose w.r.t. to the camera.
 */
class LinemodBasedPoseDetector
{
public:
    class CPoseRecord{
        public:
            unsigned int ui_objectId;
            unsigned int ui_templateId;
            int i_bbX, i_bbY, i_bbWidth, i_bbHeight; // bounding box generated by addTemplate
            double d_R1,d_R2,d_R3; // templates rotations in rodrigues form to save mem
            double d_T1,d_T2,d_T3; // translations
    private:
           friend class boost::serialization::access; // needed by Boost
           // When the class Archive corresponds to an output archive, the
           // & operator is defined similar to <<.  Likewise, when the class Archive
           // is a type of input archive the & operator is defined similar to >>.
           template<class Archive>
           void serialize(Archive& ar, const unsigned int version)
           {
               (void)(version);
               ar & ui_objectId;
               ar & ui_templateId;
               ar & i_bbX;
               ar & i_bbY;
               ar & i_bbWidth;
               ar & i_bbHeight;
               ar & d_R1;
               ar & d_R2;
               ar & d_R3;
               ar & d_T1;
               ar & d_T2;
               ar & d_T3;
           }
     };
     typedef std::map<int, CPoseRecord> TMapRecord;
    /*!
     * @brief Constructor
     */
    LinemodBasedPoseDetector();

    /*!
     * @brief Destructor
     */
    virtual ~LinemodBasedPoseDetector();

    /*!
     * @brief Initialize as a 2D only detector (only the color gradient modality is used)
     */
    void InitAs2D(int T_level0=5, int T_level1=8);

    /*!
     * @brief Initialize as a 3D detector (the color gradient modality and depth map is used)
     */
    void InitAs3D(int T_level0=5, int T_level1=8);

    /*!
     * @brief Save a training set
     * generate 2 files: one with "_training.dat" added to the argument and the other
     * with "_poses.dat" added to the argument
     * @param str_baseFullPathname Full base pathname for the files to be saved
     */
    void SaveTraining(const std::string& str_baseFullPathname);

    /*!
     * @brief Generate training views sampled at the desired intervals
     * @param ply_path Path to the CAD model name, without extension
     * @param isPLY True if .ply, otherwise .obj is assumed
     */
    void Train(const std::string& ply_path, bool isPLY,
               float f_lonMin, float f_lonMax, float f_lonStep,
               float f_latMin, float f_latMax, float f_latStep,
               float f_angleMin, float f_angleMax, float f_angleStep,
               float f_radiusMin, float f_radiusMax, float f_radiusStep);

    /*!
     * @brief AddToTemplateBase
     * @param str_objName Object name / class id (for multi-objects detection)
     * @param ui_objectId Object id
     * @param sources source modalities (RGB [+ depth])
     * @param cvmat_mask Mask to segment the object
     * @param vec_R Rotation vector used to render the object
     * @param vec_T Translation vector used to render the object
     * @return
     */
    bool AddToTemplateBase(const std::string &str_objName, unsigned int ui_objectId, const std::vector<cv::Mat> &sources,
                           const cv::Mat& cvmat_mask, const cv::Vec3d &vec_R, const cv::Vec3d &vec_T);

    int _num_modalities;
    unsigned int _ui_numberOfTemplates;
    cv::Ptr<cv::linemod::Detector> _cvptr_detector;
    TMapRecord _x_poseMap;
    double _winW;
    double _winH;
    double _fx;
    double _fy;
    double _cx;
    double _cy;
    std::string _saveDir;
};

}
}
}
#endif // LINEMODTRAINING_LINEMODTRAINIMPL_HPP

/** @} */
