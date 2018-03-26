/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CalibrateSingleCamera.cpp
 * @date 23/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Main files for the calibration of the camera.
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
#include "SingleCameraCalibrator.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

using namespace DataGenerators;

int main(int argc, char** argv)
	{
	unsigned numberOfRows;
	unsigned numberOfColumns;
	float chessboardSquareSize;
	std::string filesPath;
	std::string fileName;
	std::string filesExtension;
	SingleCameraCalibrator::FilesReadingMode filesReadingMode;

	if (argc < 7)
		{
		std::stringstream messageStream;
		messageStream << "You have to provide 6 or 7 parameters, in the following order: (i) number of rows, (ii) number of columns, (iii) chessboard square size, " <<
			" (iv) the file reading mode: either SEQUENTIAL_NAMES or IN_FILE, (v) the path of the folder containing the images or the container file, " <<
			" (vi) the base name of the images without extension if you selected SEQUENTIAL_NAMES or the name of the container file with extension if you selected IN_FILE, " <<
			" (vii) it is needed only if you selected SEQUENTIAL_NAMES, this is the extension of the image files ";
		std::string message = messageStream.str();
		PRINT_TO_LOG(message, ""); 
		}

	numberOfRows = std::stoi( argv[1] );
	numberOfColumns = std::stoi( argv[2] );
	chessboardSquareSize = std::stof( argv[3] );
	std::string filesReadingModeString = argv[4];
	if (filesReadingModeString == "SEQUENTIAL_NAMES" )
		{
		filesReadingMode = SingleCameraCalibrator::SEQUENTIAL_NAMES;
		}
	else
		{
		filesReadingMode = SingleCameraCalibrator::NAMES_IN_LIST_FILE;
		}
	filesPath = argv[5];
	fileName = argv[6];

	if ( filesReadingMode == SingleCameraCalibrator::SEQUENTIAL_NAMES )
		{
		ASSERT(argc > 7, "If file reading mode is SEQUENTIAL_NAMES, you need to provide 7 parameters");
		filesExtension = argv[7];
		}

	SingleCameraCalibrator* calibrator;
	if (filesReadingMode == SingleCameraCalibrator::SEQUENTIAL_NAMES)
		{
		calibrator = new SingleCameraCalibrator(chessboardSquareSize, numberOfRows, numberOfColumns, filesPath, fileName, filesExtension);
		}
	else	
		{
		calibrator = new SingleCameraCalibrator(chessboardSquareSize, numberOfRows, numberOfColumns, filesPath, fileName);
		}
	calibrator->CalibrateCamera();

	delete(calibrator);
	return 0;
	}

/** @} */
