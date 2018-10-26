/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_MATCHLIST_HPP
#define FEATURESMATCHING3D_MATCHLIST_HPP

#include "FeaturesMatching3DInterface.hpp"

#include <VisualPointFeatureVector3D.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Pose.hpp>
#include <SupportTypes.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{

	/**
	 * This class contains list of matches, and it offers an iterator that iterates
	 * among all best combinations of matches
	 *
	 */
	class MatchList
	{
		public:
			struct BestMatch
				{
				std::vector<int> sourceIndexList;
				std::vector<int> sinkIndexList;
				}

			MatchList();
			virtual ~MatchList();

			void AddMatch(int sourceIndex, int sinkIndex, float distance);
			BestMatch GetBestMatch();
			BestMatch GetNextBestMatch();
			BestMatch GetStableMatch(int numberOfMatches);

		private:
			
			struct Match
				{
				int sourceIndex;
				int sinkIndex;
				float distance;
				Match(int sr, int sk, float d) 
					{ sourceIndex = sr; sinkIndex = sk; distance = d; }
				};

			int numberOfDistinctSources;
			int numberOfDistinctSinks;

			std::vector<Match> listByDistance;
			std::vector<int> currentBestSelection;
			std::set<int> currentSelectedSources;
			std::set<int> currentSelectedSinks; 

			void ExtendSelection(int startMatchIndex);
			BestMatch GetMatchFromSelection();
	};
}
}
}

#endif // FEATURESMATCHING3D_MATCHLIST_HPP

/** @} */
