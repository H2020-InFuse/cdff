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
 * This program is used to produce a list of images contained in a directory. It takes 3 mondatory paramters and an optional 1:
 * @param baseFolder: the base folder where you will write the image list file;
 * @param folder: the relative path from baseFolder to the folder containing the image files;
 * @param fileName: the name of the image list file that will be created in the baseFolder;
 * @param timestampFile: the name of the timestamp file, located in the image folder. The timestamp file has one line for each image in the format "id timestamp".
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
	".jpg",
	".pgm"
	};

const std::vector<std::string> leftPatternsList =
	{
	"left",
	"Left",
	"LEFT",
	".0.",
	"_0_",
	"_0.",
	".0_",
	"camL"
	};

const std::vector<std::string> rightPatternsList =
	{
	"right",
	"Right",
	"RIGHT",
	".1.",
	"_1_",
	"_1.",
	".1_",
	"camR"
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

void ReadTimestamps(std::string timestampFilePath, std::vector<std::string>& timestampList)
	{
	std::ifstream file(timestampFilePath.c_str());
	std::string id, newStamp;
	while(file.good())
		{
		file >> id;
		file >> newStamp;
		if (file.good())
			{
			timestampList.push_back(newStamp);
			}
		}
	file.close();
	}


int main(int argc, char** argv)
	{
	ASSERT(argc == 4 || argc == 5, "You need three arguments: baseFolder, folder, fileName, optionally timestamp file as contained in folder");
	std::string baseFolder = argv[1];
	std::string folder = argv[2];
	std::string fileName = argv[3];
	std::string timestampFileName = (argc == 5) ? argv[4] : "Nothing";
	
	std::stringstream folderPath, filePath;
	folderPath << baseFolder << "/" << folder;
	filePath << baseFolder << "/" << fileName;

	std::vector<std::string> filesList;
	std::vector<std::string> timestampList;
	ReadFilesInDirectory(folderPath.str(), filesList);
	if (argc >= 5)
		{
		std::stringstream timestampFilePath;
		timestampFilePath << baseFolder << "/" << folder << "/" << timestampFileName;
		ReadTimestamps(timestampFilePath.str(), timestampList);
		}

	std::ofstream file(filePath.str().c_str());
	file << "#\n#\n#\n";

	unsigned time = 0;
	unsigned timestampIndex = 0;
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
			if (argc == 4)
				{
				file << time * 0.1;
				}
			else	
				{
				ASSERT( timestampIndex < timestampList.size(), "The number of timestamps does not match the number of image files");
				file << timestampList.at(timestampIndex);
				timestampIndex++;
				}
			file << " " << folder << "/" << imageName << " " << folder << "/" << rightImageName << std::endl;
			time++;
			}
		else if (!IsRightImage(imageName, patternIndex))
			{
			if (argc == 4)
				{
				file << time * 0.1;
				}
			else	
				{
				ASSERT( timestampIndex < timestampList.size(), "The number of timestamps does not match the number of image files");
				file << timestampList.at(timestampIndex);
				timestampIndex++;
				}
			file << " " << folder << "/" << imageName << std::endl;
			time++;
			}
		}


	file.close();

	return 0;
	}

/** @} */
