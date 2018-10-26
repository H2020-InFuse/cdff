/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "MatchList.hpp"

#include <EigenTransformToTransform3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>
#include <yaml-cpp/yaml.h>

using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{

MatchList::MatchList()
	{
	numberOfDistinctSources = 0;
	numberOfDistinctSinks = 0;
	}

MatchList::~MatchList()
	{

	}

void MatchList::AddMatch(int sourceIndex, int sinkIndex, float distance)()
	{
	std::vector<Match>::iterator insertPosition;
	bool insertPositionFound = false;
	bool sourceAlreadyInList = false;
	bool sinkAlreadyInList = false;

	for (std::vector<Match>::iterator matchIterator = listByDistance.begin(); matchIterator != listByDistance.end(); matchIterator++)
		{
		if (!insertPositionFound && matchIterator->distance > distance)
			{
			insertPositionFound = true;
			}
		sourceAlreadyInList = sourceAlreadyInList || (matchIterator->sourceIndex == sourceIndex);
		sinkAlreadyInList = sinkAlreadyInList || (matchIterator->sinkIndex == sinkIndex);			
		}

	Match match(sourceIndex, sinkIndex, distance);
	listByDistance.insert(matchIterator, match);

	if (!sourceAlreadyInList)
		{
		numberOfDistinctSources++;
		}
	if (!sinkAlreadyInList)
		{
		numberOfDistinctSinks++;
		}
	}

BestMatch MatchList::GetBestMatch()
	{
	currentSelectedSources.clear();
	currentSelectedSinks.clear();
	currentBestSelection.clear();

	ExtendSelection( 0 );
	
	return GetMatchFromSelection();
	}

BestMatch MatchList::GetNextBestMatch()
	{
	int lastSelectionIndex = currentBestSelection.at( currentBestSelection.size() - 1 );
	currentBestSelection.pop_back();
	
	const Match& lastSelection = listByDistance.at(lastSelectionIndex);
	currentSelectedSources.erase(lastSelection.sourceIndex);
	currentSelectedSinks.erase(lastSelection.sinkIndex);

	ExtendSelection( lastSelectionIndex+1 );
	
	return GetMatchFromSelection();	
	}

void MatchList::ExtendSelection(int startMatchIndex)
	{
	int numberOfMatches = listByDistance.size();
	for(int matchIndex = startMatchIndex; matchIndex < numberOfMatches; matchIndex++)
		{
		const Match& match = listByDistance.at(matchIndex);
		if (currentSelectedSources.find( match.sourceIndex ) == currentSelectedSources.end() && currentSelectedSinks.find( match.sinkIndex ) == currentSelectedSinks.end() )
			{
			currentSelectedSources.insert(match.sourceIndex);
			currentSelectedSources.insert(match.sinkIndex);
			currentBestSelection.insert(matchIndex);
			}
		}
	}

BestMatch MatchList::GetStableMatch(int numberOfMatches)
	{
	BestMatch bestMatch;

	return bestMatch;
	}

BestMatch MatchList::GetMatchFromSelection()
	{
	BestMatch bestMatch;
	for(std::set<int>::iterator matchIterator = currentBestSelection.begin(); matchIterator != currentBestSelection.end(); matchIterator++)
		{
		const Match& selection = listByDistance.at( *matchIterator );
		bestMatch.sourceIndexList.push_back( selection.sourceIndex );
		bestMatch.sinkIndexList.push_back( selection.sinkIndex );
		}
	return bestMatch;
	}

}
}
}

/** @} */
