/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING2D_FLANNMATCHER_HPP
#define FEATURESMATCHING2D_FLANNMATCHER_HPP

#include "FeaturesMatching2DInterface.hpp"

#include <CorrespondenceMap2D.hpp>
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching2D
{
	/**
	 * 2D feature matching using FLANN (provided by OpenCV).
	 *
	 * Processing steps: (i) initialization of a FLANN matcher using the
	 * provided parameters, (ii) matching and selection of the best matches,
	 * (iii) filtering of the best matches: only those that satisfy the
	 * acceptanceRatio are kept.
	 *
	 * @param distanceThreshold
	 * @param numberOfChecks
	 * @param epsilon
	 * @param sortedSearch
	 * @param acceptanceRatio
	 *        a value between 0 and 1 that defines whether a good match is kept
	 *        and returned
	 * @param matcherMethod
	 *        method used for matching: KdTreeSearch, KMeansClustering,
	 *        AutotunedSearch, HierarchichalClustering, LocalitySensitiveHashing,
	 *        CompositeSearch, or LinearSearch
	 *        method-specific parameters are listed below
	 *
	 * @param kdTreeSearchOptionsSet.numberOfTrees
	 *
	 * @param kMeansClusteringOptionsSet.branching
	 * @param kMeansClusteringOptionsSet.iterations
	 * @param kMeansClusteringOptionsSet.centersInitialization
	 * @param kMeansClusteringOptionsSet.convertibleBoundIndex
	 *
	 * @param autotunedOptionsSet.targetPrecision
	 * @param autotunedOptionsSet.buildWeight
	 * @param autotunedOptionsSet.memoryWeight
	 * @param autotunedOptionsSet.sampleFraction
	 *
	 * @param hierarchicalClusteringOptionsSet.branching
	 * @param hierarchicalClusteringOptionsSet.centersInitialization
	 * @param hierarchicalClusteringOptionsSet.numberOfTrees
	 * @param hierarchicalClusteringOptionsSet.leafSize
	 *
	 * @param localitySensitiveHashingOptionsSet.tableNumber
	 * @param localitySensitiveHashingOptionsSet.keySize
	 * @param localitySensitiveHashingOptionsSet.multiProbeLevel
	 *
	 * @param compositeSearchOptionsSet.branching
	 * @param compositeSearchOptionsSet.iterations
	 * @param compositeSearchOptionsSet.centersInitialization
	 * @param compositeSearchOptionsSet.convertibleBoundIndex
	 * @param compositeSearchOptionsSet.numberOfTrees
	 */
	class FlannMatcher : public FeaturesMatching2DInterface
	{
		public:

			FlannMatcher();
			virtual ~FlannMatcher();

			virtual void configure();
			virtual void process();

		private:

			typedef cvflann::flann_centers_init_t CenterInitializationMethod;
			class CenterInitializationMethodHelper : public Helpers::ParameterHelper<CenterInitializationMethod, std::string>
			{
				public:
					CenterInitializationMethodHelper(const std::string& parameterName, CenterInitializationMethod& boundVariable, const CenterInitializationMethod& defaultValue);
				private:
					CenterInitializationMethod Convert(const std::string& value);
			};

			enum MatcherMethod
			{
				KD_TREE_SEARCH,
				K_MEANS_CLUSTERING,
				AUTOTUNED_SEARCH,
				HIERARCHICAL_CLUSTERING,
				LOCALITY_SENSITIVE_HASHING,
				COMPOSITE_SEARCH,
				LINEAR_SEARCH
			};
			class MatcherMethodHelper : public Helpers::ParameterHelper<MatcherMethod, std::string>
			{
				public:
					MatcherMethodHelper(const std::string& parameterName, MatcherMethod& boundVariable, const MatcherMethod& defaultValue);
				private:
					MatcherMethod Convert(const std::string& value);
			};

			struct GeneralOptionsSet
			{
				float distanceThreshold;
				int numberOfChecks;
				float epsilon;
				bool sortedSearch;
				MatcherMethod matcherMethod;
				float acceptanceRatio;
			};

			struct KdTreeSearchOptionsSet
			{
				int numberOfTrees;
			};

			struct KMeansClusteringOptionsSet
			{
				int branching;
				int iterations;
				CenterInitializationMethod centersInitialization;
				float convertibleBoundIndex;
			};

			struct AutotunedOptionsSet
			{
				float targetPrecision;
				float buildWeight;
				float memoryWeight;
				float sampleFraction;
			};

			struct HierarchicalClusteringOptionsSet
			{
				int branching;
				CenterInitializationMethod centersInitialization;
				int numberOfTrees;
				int leafSize;
			};

			struct LocalitySensitiveHashingOptionsSet
			{
				int tableNumber;
				int keySize;
				int multiProbeLevel;
			};

			struct CompositeSearchOptionsSet
			{
				int branching;
				int iterations;
				CenterInitializationMethod centersInitialization;
				float convertibleBoundIndex;
				int numberOfTrees;
			};

			struct FlannMatcherOptionsSet
			{
				GeneralOptionsSet generalOptionsSet;
				KdTreeSearchOptionsSet kdTreeSearchOptionsSet;
				KMeansClusteringOptionsSet kMeansClusteringOptionsSet;
				AutotunedOptionsSet autotunedOptionsSet;
				HierarchicalClusteringOptionsSet hierarchicalClusteringOptionsSet;
				LocalitySensitiveHashingOptionsSet localitySensitiveHashingOptionsSet;
				CompositeSearchOptionsSet compositeSearchOptionsSet;
			};

			Helpers::ParametersListHelper parametersHelper;
			FlannMatcherOptionsSet parameters;
			static const FlannMatcherOptionsSet DEFAULT_PARAMETERS;

			Converters::VisualPointFeatureVector2DToMatConverter visualPointFeatureVector2DToMat;

			cv::Ptr<cv::flann::IndexParams> ConvertParameters();
			std::vector< cv::DMatch > ComputeMatches(cv::Mat sourceDescriptorsMatrix, cv::Mat sinkDescriptorsMatrix);
			void CleanLowScoringMatches(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, 
				CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr cleanMap);
			cv::Mat ConvertToValidType(cv::Mat floatDescriptorsMatrix);
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr Convert(std::vector<cv::DMatch> matchesVector, cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix);

			void ValidateParameters();
			void ValidateInputs(cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix);
	};
}
}
}

#endif // FEATURESMATCHING2D_FLANNMATCHER_HPP

/** @} */
