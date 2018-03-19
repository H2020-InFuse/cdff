/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FlannMatcher.hpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Flann Matcher.
 *  
 *
 * @{
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN applies the Flann Matcher algorithm as provided by OpenCV for matching 2d features in 2d images.
 *  
 * This DFN implementation operates according the following steps:
 * (i) initialization of the Flann matcher according to the DFN parameters, (ii) application of the flann matcher to the inputs and extraction of the best matches, (iii) filtering of the best matches:
 * only those matches that satisfy the acceptanceRatio are retained.
 * 
 * This DFN implementation requires the following parameters:
 * @param distanceThreshold,
 * @param numberOfChecks,
 * @param epsilon,
 * @param sortedSearch, 
 * @param acceptanceRatio, the acceptanceRatio between 0 and 1 that defines whether a good match is retained and provided as output.
 * @param matcherMethod, the matching method applied, it can be one of the following: KdTreeSearch, KMeansClustering, AutotunedSearch, HierarchichalClustering, LocalitySensitiveHashing, CompositeSearch
 * 			or LinearSearch. Depending on the matcher method other parameters apply.
 *
 * @param kdTreeSearchOptionsSet.numberOfTrees,
 *
 * @param kMeansClusteringOptionsSet.branching, 
 * @param kMeansClusteringOptionsSet.iterations, 
 * @param kMeansClusteringOptionsSet.centersInitialization, 
 * @param kMeansClusteringOptionsSet.convertibleBoundIndex, 
 *
 * @param autotunedOptionsSet.targetPrecision,
 * @param autotunedOptionsSet.buildWeight,
 * @param autotunedOptionsSet.memoryWeight,
 * @param autotunedOptionsSet.sampleFraction,
 *
 * @param hierarchicalClusteringOptionsSet.branching,
 * @param hierarchicalClusteringOptionsSet.centersInitialization,
 * @param hierarchicalClusteringOptionsSet.numberOfTrees,
 * @param hierarchicalClusteringOptionsSet.leafSize,
 *
 * @param localitySensitiveHashingOptionsSet.tableNumber,
 * @param localitySensitiveHashingOptionsSet.keySize,
 * @param localitySensitiveHashingOptionsSet.multiProbeLevel,
 *
 * @param compositeSearchOptionsSet.branching,
 * @param compositeSearchOptionsSet.iterations,
 * @param compositeSearchOptionsSet.centersInitialization,
 * @param compositeSearchOptionsSet.convertibleBoundIndex,
 * @param compositeSearchOptionsSet.numberOfTrees,
 *
 * @{
 */

#ifndef FLANN_MATCHER_HPP
#define FLANN_MATCHER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include "opencv2/features2d/features2d.hpp"
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FlannMatcher : public FeaturesMatching2DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FlannMatcher();
            ~FlannMatcher();
            void process();
            void configure();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
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

		cv::Ptr<cv::flann::IndexParams> ConvertParameters();
		std::vector< cv::DMatch > ComputeMatches(cv::Mat sourceDescriptorsMatrix, cv::Mat sinkDescriptorsMatrix);
		cv::Mat ConvertToValidType(cv::Mat floatDescriptorsMatrix);
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr Convert(std::vector< cv::DMatch > matchesVector, cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix);

		void ValidateParameters();
		void ValidateInputs(cv::Mat sourceFeaturesMatrix, cv::Mat sinkFeaturesMatrix);
    };
}
#endif
/* FlannMatcher.hpp */
/** @} */
