#ifndef STL_TRIG_MESH_FILE_INCLUDED
#define STL_TRIG_MESH_FILE_INCLUDED

#include <fstream>
#include <string>
#include <algorithm> 
#include <cctype>
#include <locale>
#include <vector>
#include <sstream>
#include <utility>
#include <cstdint>
#include <memory>

#include "TypeDef.hpp"
#include "StringFuncs.hpp"

template<class T>
class StlTrigMeshFile 
{
public:
	bool init_;
	U32 trigCount_;

	std::shared_ptr< T > vertexData_;

	std::shared_ptr< U32 > trigMeterialIndex_;

	std::shared_ptr< T > normalData_;

	static std::string vertexStartStr_;
	static std::string vertexEndStr_;
	static std::string trigStartStr_;
	static std::string normStartStr_;

public:

	StlTrigMeshFile() :
		init_(false),
		trigCount_(0),
		vertexData_(),
		trigMeterialIndex_(),
		normalData_()
	{

	}

	~StlTrigMeshFile()
	{

	}

	void Reset()
	{
		if (init_)
		{
			init_ = false;
			trigCount_ = 0;
			vertexData_.reset();
			trigMeterialIndex_.reset();
			normalData_.reset();
		}
	}

	bool Load(const std::string& filePath, U32 type)
	{
		using namespace StringFuncs;

		Reset();

		std::fstream stlFile(filePath, std::ios::in);

		std::string lineStr = "";

		while (std::getline(stlFile,lineStr))
		{
			trigCount_++;
		}
		trigCount_ = (trigCount_ - 2) / 7;

		//std::cout << "Load Fail. STL is not standard." << std::endl;

		normalData_.reset(new T[3 * trigCount_], [](T* ptr) { delete[] ptr; });
		vertexData_.reset(new T[3 * 3 * trigCount_], [](T* ptr) { delete[] ptr; });
		trigMeterialIndex_.reset(new U32[trigCount_], [](U32* ptr) { delete[] ptr; });
		T* normalDataPtr = normalData_.get();
		T* vertexDataPtr = vertexData_.get();
		U32* trigMeterialIndexPtr = trigMeterialIndex_.get();

		stlFile.clear();
		stlFile.seekg(0, ios::beg);
		std::getline(stlFile, lineStr);

		for (U32 i = 0; i < trigCount_; i++)
		{
			std::getline(stlFile, lineStr);
			Trim(lineStr);
			if (i == trigCount_ - 1) {
				int l = 0;
				l++;
			}
			vector<string> splitStr = Explode(lineStr, ' ');
			normalDataPtr[i*3] = (T)(std::stod(splitStr[2]));
			normalDataPtr[i * 3 + 1] = (T)(std::stod(splitStr[3]));
			normalDataPtr[i * 3 + 2] = (T)(std::stod(splitStr[4]));
			trigMeterialIndexPtr[i] = type;
			std::getline(stlFile, lineStr);

			for (U32 j = 0; j < 3; j++)
			{
				std::getline(stlFile, lineStr);
				Trim(lineStr);
				vector<string> vCoords = Explode(lineStr, ' ');
				vertexDataPtr[i * 9 + j * 3] = (T)(std::stod(vCoords[1]));
				vertexDataPtr[i * 9 + j * 3 + 1] = (T)(std::stod(vCoords[2]));
				vertexDataPtr[i * 9 + j * 3 + 2] = (T)(std::stod(vCoords[3]));
			}
			std::getline(stlFile, lineStr);
			std::getline(stlFile, lineStr);
		}

		stlFile.close();

		init_ = true;

		return true;
	}

};
template< class T >
std::string StlTrigMeshFile< T >::vertexStartStr_ = "vertex";

template< class T >
std::string StlTrigMeshFile< T >::trigStartStr_ = "outer";

template< class T >
std::string StlTrigMeshFile< T >::normStartStr_ = "facet";
#endif