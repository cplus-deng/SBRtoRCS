#include "stlsplit.h"
#include "StringFuncs.hpp"
stlsplit::stlsplit()
{
    if (solids_.size() != 0)
    {
        solids_.clear();
    }
}

void stlsplit::LoadAndWrite(const std::string& content)
{
    using namespace StringFuncs;
    std::istringstream iss(content);
    std::string line;
    std::string solid;
    std::ofstream outFile;
    std::ostringstream sectionContent;
    std::string path = "..\\SBRApp\\splitMesh\\";

    try
    {
        while (std::getline(iss, line)) {
            sectionContent << line << '\n'; // Store content of the section
            if (line.find("solid") != std::string::npos) {
                Trim(line);
                /*vector<string> splitStr = Explode(line, ' ');
                solid = Explode(splitStr.at(0), ':').at(1);*///solid:object1 (no space)
                solid = Explode(line, ' ').at(1);
            }
            if (line.find("endsolid") != std::string::npos) {
                std::string filename = path + solid + ".stl";
                solids_.push_back(solid);
                solidPaths_.push_back(filename);
                outFile.open(filename);
                if (outFile.is_open()) {
                    outFile << sectionContent.str();
                    outFile.close();
                    sectionContent.str("");
                }
            }
        }
    }
    catch (std::out_of_range& e)
    {
        std::istringstream issSingle(content);
        std::string lineSingle;
        std::ofstream outFileSingle;
        std::ostringstream singleContent;
        int lineCnt = 0;
        while (std::getline(issSingle, lineSingle)) {
            singleContent << lineSingle << '\n';
            lineCnt++;
        }
        if ((lineCnt - 2) % 7 != 0)
        {
            throw std::out_of_range("Unable to process this type of STL file.: " + std::string(e.what()));
        }
        solid = "Object";
        solids_.push_back(solid);
        std::string filename = path + "Object.stl";
        solidPaths_.push_back(filename);
        outFileSingle.open(filename);
        if (outFileSingle.is_open()) {
            outFileSingle << sectionContent.str();
            outFileSingle.close();
            singleContent.str("");
        }
    }
    
}
