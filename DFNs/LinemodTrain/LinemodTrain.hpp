/**
 * @addtogroup DFNs
 * @{
 */

#ifndef LINEMODTRAIN_LINEMODTRAIN_HPP
#define LINEMODTRAIN_LINEMODTRAIN_HPP

#include "LinemodTrainInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"


namespace CDFF
{
namespace DFN
{
namespace LinemodTrain
{
    class LinemodTrain : public LinemodTrainInterface
    {
        public:

            LinemodTrain();
            virtual ~LinemodTrain();

            virtual void configure();
            virtual void process();

            struct LinemodTrainParams
            {
                int T_level0;
                int T_level1;
                int renderWindowWidth;
                int renderWindowHeight;
                double fx;
                double fy;
                double cx;
                double cy;
                std::string cadObjectName;
                bool isPLY;
                bool useDepthModality;
                float longMin;
                float longStep;
                float longMax;
                float latMin;
                float latStep;
                float latMax;
                float angleMin;
                float angleStep;
                float angleMax;
                float radiusMin;
                float radiusStep;
                float radiusMax;
                std::string saveTrainingImg;
            };

            Helpers::ParametersListHelper parametersHelper;
            LinemodTrainParams parameters;
            static const LinemodTrainParams DEFAULT_PARAMETERS;
            void ValidateParameters();

    };
}
}
}

#endif // LINEMODTRAIN_LINEMODTRAIN_HPP

/** @} */
