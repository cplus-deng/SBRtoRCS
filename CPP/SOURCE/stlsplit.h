#ifndef STL_SPLIT_INCLUDE
#define STL_SPLIT_INCLUDE

#include<vector>
#include<string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "StringFuncs.hpp"
class stlsplit
{
public:
	stlsplit();
	std::vector<std::string> solids_;
	std::vector<std::string> solidPaths_;
private:
	bool validFile_;
public:
	void LoadAndWrite(const std::string& fileName);
};
#endif // !STL_SPLIT_INCLUDE

