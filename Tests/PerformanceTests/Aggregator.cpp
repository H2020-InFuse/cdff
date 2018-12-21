/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Aggregator.cpp
 * @date 26/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the Aggregator class.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Aggregator.hpp"
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <vector>
#include <Errors/Assert.hpp>


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Aggregator::Aggregator(AggregatorOperation operation)
	{
	this->operation = operation;
	}

void Aggregator::AddMeasure(double measure, unsigned channel)
	{
	while (measuresList.size() <= channel)
		{
		measuresList.push_back( std::vector<double>() );
		} 

	measuresList.at(channel).push_back(measure);
	}

std::vector< double > Aggregator::Aggregate()
	{
	std::vector<double> result(measuresList.size());
	switch(operation)
		{
		case SUM:
			{
			Sum(result);
			break;
			}
		case AVERAGE:
			{
			Average(result);
			break;
			}
		default:
			{
			ASSERT(false, "Unhandled Aggregator Operation");
			}
		}
	return result;
	}

void Aggregator::Clear()
	{
	measuresList.clear();
	}

unsigned Aggregator::GetNumberOfChannels()
	{
	return measuresList.size();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void Aggregator::Sum(std::vector< double >& sum)
	{
	for(unsigned channel = 0; channel < measuresList.size(); channel++)
		{
		sum.at(channel) = 0;
		
		for(std::vector<double>::iterator measure = measuresList.at(channel).begin(); measure != measuresList.at(channel).end(); ++measure)
			{
			sum.at(channel) = sum.at(channel) + (*measure);
			}
		}
	}

void Aggregator::Average(std::vector< double >& average)
	{
	for(unsigned channel = 0; channel < measuresList.size(); channel++)
		{
		average.at(channel) = 0;
		
		for(std::vector<double>::iterator measure = measuresList.at(channel).begin(); measure != measuresList.at(channel).end(); ++measure)
			{
			average.at(channel) = average.at(channel) + (*measure);
			}
		average.at(channel) = average.at(channel) / ( (double) (measuresList.at(channel).size()) );
		}
	}

/** @} */
