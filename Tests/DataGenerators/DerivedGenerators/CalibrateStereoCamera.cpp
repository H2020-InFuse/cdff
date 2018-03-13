/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CalibrateStereoCamera.cpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Main files for the calibration of the stereo camera.
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
#include "StereoCameraCalibrator.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

using namespace DataGenerators;

const unsigned NUMBER_OF_IMAGES = 11;
const unsigned NUMBER_OF_ROWS = 7;
const unsigned NUMBER_OF_COLUMNS = 11;
const float CHESSBOARD_SQUARE_SIZE = 0.02;
const std::string FILES_PATH = "../tests/Data/Images/";
const std::string FILE_NAME = "chessboard";
const std::string FILES_EXTENSION = ".jpg";
StereoCameraCalibrator::FilesMode FILES_MODE = StereoCameraCalibrator::ONE_IMAGE_FILE;

int main(int argc, char** argv)
	{
	unsigned numberOfImages = NUMBER_OF_IMAGES;
	unsigned numberOfRows = NUMBER_OF_ROWS;
	unsigned numberOfColumns = NUMBER_OF_COLUMNS;
	float chessboardSquareSize = CHESSBOARD_SQUARE_SIZE;
	std::string filesPath = FILES_PATH;
	std::string fileName = FILE_NAME;
	std::string filesExtension = FILES_EXTENSION;
	StereoCameraCalibrator::FilesMode filesMode = FILES_MODE;
	std::string rightFileName;

	if (argc > 1 )
		{
		numberOfImages = std::stoi( argv[1] );
		}
	if (argc > 2 )
		{
		numberOfRows = std::stoi( argv[2] );
		}
	if (argc > 3 )
		{
		numberOfColumns = std::stoi( argv[3] );
		}
	if (argc > 4 )
		{
		chessboardSquareSize = std::stof( argv[4] );
		}
	if (argc > 5 )
		{
		filesPath = argv[5];
		}
	if (argc > 6 )
		{
		fileName = argv[6];
		}
	if (argc > 7 )
		{
		filesExtension = argv[7];
		}
	if (argc > 8 )
		{
		std::string filesModeString = argv[8];
		if (filesModeString == "one" )
			{
			filesMode = StereoCameraCalibrator::ONE_IMAGE_FILE;
			}
		else
			{
			filesMode = StereoCameraCalibrator::TWO_IMAGE_FILES;
			}
		}
	if (argc > 9 )
		{
		rightFileName = argv[9];
		}

	StereoCameraCalibrator calibrator(numberOfImages, chessboardSquareSize, numberOfRows, numberOfColumns, filesPath, fileName, filesExtension, filesMode, rightFileName);
	calibrator.CalibrateCameras();
	calibrator.RectifyCameras();

	return 0;
	}

/** @} */
