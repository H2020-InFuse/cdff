/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Aggregator.hpp
 * @date 26/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * @brief This interface is used for aggregating measures of a performance test
 *
 * 
 * @{
 */

#ifndef AGGREGATOR_HPP
#define AGGREGATOR_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class Aggregator
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		enum AggregatorOperation
			{
			SUM,
			AVERAGE
			};

		/*
		* @brief the constructor requires the type of aggregation that needs to be performed.
		*
		*/
		Aggregator(AggregatorOperation operation);

		/*
		* @brief the method records a measure for future aggregation.
		*
		* @param measure, the value of the measure to be aggregated
		* @param channel, identifier of the set of data, measures are aggregated only within the same channel
		*
		*/
		void AddMeasure(double measure, unsigned channel = 0);

		/*
		* @brief the method performs the aggregation and computes a value for each channel.
		*
		* @output aggregation, a vector of doubles of size equal to the largest input channel + 1, each element at position i is the result of the aggregation of the data in channel i.
		*
		*/
		std::vector< double > Aggregate();

		/*
		* @brief Clears the stored measures.
		*
		*/	
		void Clear();

		/*
		* @brief Retrieves the number of channels.
		*
		*/
		unsigned GetNumberOfChannels();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:	
		std::vector< std::vector< double> > measuresList;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:	
		AggregatorOperation operation;
		void Sum(std::vector< double >& sum);
		void Average(std::vector< double >& average);

	};

#endif

/* Aggregator.hpp */
/** @} */
