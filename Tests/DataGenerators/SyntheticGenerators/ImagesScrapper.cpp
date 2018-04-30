/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImagesScrapper.cpp
 * @date 27/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This program is used to produce a list of images contained in a directory.
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
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

const std::vector<std::string> imageExtensionsList =
	{
	".png",
	".ppm",
	".jpg"
	};

const std::vector<std::string> leftPatternsList =
	{
	"left",
	"Left",
	"LEFT",
	".0.",
	"_0_",
	"_0.",
	".0_"
	};

const std::vector<std::string> rightPatternsList =
	{
	"right",
	"Right",
	"RIGHT",
	".1.",
	"_1_",
	"_1.",
	".1_"
	};

void ReadFilesInDirectory(std::string folderPath, std::vector<std::string>& filesList)
	{
	DIR* folder;
	struct dirent* directorySearch;

	folder = opendir(folderPath.c_str());
	ASSERT(folder != NULL, "Error: directory not found");

	bool moreFiles = true;
	while(moreFiles)
		{
		directorySearch = readdir(folder);
		if (directorySearch != NULL)
			{
			filesList.push_back(directorySearch->d_name);
			}
		else
			{
			moreFiles = false;
			}
		}

	closedir(folder);
	}

bool IsFileImage(std::string fileName)
	{
	for(unsigned extensionIndex = 0; extensionIndex < imageExtensionsList.size(); extensionIndex++)
		{
		std::string extension = imageExtensionsList.at(extensionIndex);
		if (fileName.find(extension) != std::string::npos)
			{
			return true;
			}
		}
	return false;
	}

bool IsLeftImage(std::string fileName, unsigned& patternIndex)
	{
	for(unsigned leftIndex = 0; leftIndex < leftPatternsList.size(); leftIndex++)
		{
		std::string leftPattern = leftPatternsList.at(leftIndex);
		if (fileName.find(leftPattern) != std::string::npos)
			{
			patternIndex = leftIndex;
			return true;
			}
		}
	return false;
	}

bool IsRightImage(std::string fileName, unsigned& patternIndex)
	{
	for(unsigned rightIndex = 0; rightIndex < rightPatternsList.size(); rightIndex++)
		{
		std::string rightPattern = rightPatternsList.at(rightIndex);
		if (fileName.find(rightPattern) != std::string::npos)
			{
			patternIndex = rightIndex;
			return true;
			}
		}
	return false;
	}

std::string ReplaceLeftPatternWithRightPattern(std::string leftFileName, unsigned patternIndex)
	{
	std::string leftPattern = leftPatternsList.at(patternIndex);
	unsigned position = leftFileName.find(leftPattern);
	unsigned leftPatternSize = leftPattern.size();

	std::string firstPart = leftFileName.substr(0, position);
	std::string secondPart = leftFileName.substr(position+leftPatternSize, std::string::npos);
	std::string rightPattern = rightPatternsList.at(patternIndex);
	
	std::stringstream rightFileName;
	rightFileName << firstPart << rightPattern << secondPart;
	return rightFileName.str();
	}


int main(int argc, char** argv)
	{
	ASSERT(argc == 4, "You need three arguments: baseFolder, folder, fileName");
	std::string baseFolder = argv[1];
	std::string folder = argv[2];
	std::string fileName = argv[3];
	
	std::stringstream folderPath, filePath;
	folderPath << baseFolder << "/" << folder;
	filePath << baseFolder << "/" << fileName;

	std::vector<std::string> filesList;
	ReadFilesInDirectory(folderPath.str(), filesList);

	std::ofstream file(filePath.str().c_str());

	unsigned time = 0;
	for(unsigned fileIndex = 0; fileIndex < filesList.size(); fileIndex++)
		{
		std::string imageName = filesList.at(fileIndex);
		if (!IsFileImage(imageName))
			{
			continue;
			}

		unsigned patternIndex;
		if (IsLeftImage(imageName, patternIndex))
			{
			std::string rightImageName = ReplaceLeftPatternWithRightPattern(imageName, patternIndex);
			file << time * 0.1 << " " << folder << "/" << imageName << " " << folder << "/" << rightImageName << std::endl;
			time++;
			}
		else if (!IsRightImage(imageName, patternIndex))
			{
			file << time * 0.1 << " " << folder << "/" << imageName << std::endl;
			time++;
			}
		}


	file.close();

	return 0;
	}

/** @} */
