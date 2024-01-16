#ifndef _SILENCE_AMP_DEPRECATION_WARNINGS
#define _SILENCE_AMP_DEPRECATION_WARNINGS
#endif // !_SILENCE_AMP_DEPRECATION_WARNINGS

#include <iostream>
#include "StlTrigMeshFile.h"
#include "TypeDef.hpp"
#include "Triangle.hpp"
#include "TriangleMesh.hpp"
#include "MortonManager.hpp"
#include "BvhGenerator.hpp"
#include "BvhNodeTypes.hpp"
#include "ReducedBvhArray.hpp"
#include "RayPool.hpp"
#include "DepthMapGenerator.hpp"
#include "SbrSolver.hpp"
#include "Excitation.h"


int main()
{
    #if _OPENMP
        std::cout << " support openmp " << std::endl;
    #else
        std::cout << " not support openmp" << std::endl;
    #endif

    const Float pi_ = 3.1415926;
    std::cout << "###SBR to RCS###\n";
    auto tStart = Clock::now();

    //std::string stlFilePath = std::string("E:\\Files\\STL\\sphere_r02m_f10G_anscii.stl");
    std::string stlFilePath = std::string("E:\\Files\\STL\\luoyin_plate_ANSCII.stl");
    StlTrigMeshFile<Float> stlFile;
    stlFile.Load(stlFilePath, 1);

    TriangleMesh< Float > trigMesh;
    trigMesh.ImportFromStlTrigMeshFile(stlFile);

    MortonManager< Float > mortonManager(&trigMesh);
    mortonManager.GenerateMortonArray();

    BvhGenerator< Float > bvhGenerator(&mortonManager);
    bvhGenerator.GenerateBvhArray();
    bvhGenerator.SqueezeBvhArray();
    bvhGenerator.RemoveEmptyNodes();

    ReducedBvhArray< Float > reducedBvhArray;
    bvhGenerator.PopulateReducedBvhArray(reducedBvhArray);

    std::cout << "Triangle Count: " << trigMesh.trigCount_ << std::endl;
    std::cout << "BVH Node Count: " << reducedBvhArray.nodeCount_ << std::endl;

    //wave in params
    Float thetaIn = pi_;
    Float phiIn = 0.0;
    Float alpha = pi_ / 2;  //polar
    /*Float fH = 25e9;
    Float fL = 5e9;*/
    Float fH = 25e9;
    Float fL = 5e9;
    Float f0 = (fH + fL) / 2;

    //observation
    Float thetaS = pi_ - thetaIn;
    Float phiS = pi_ + phiIn;
    Float beta = pi_ / 2;  //polar

    LUV::Vec3< Float >obsDirection(sin(thetaS) * cos(phiS), 
                                sin(thetaS) * sin(phiS), 
                                cos(thetaS));

    LUV::Vec3< Float >eTheta(cos(thetaIn) * cos(phiIn),
                             cos(thetaIn) * sin(phiIn),
                             -sin(thetaIn));
    LUV::Vec3< Float >ePhi(-sin(phiIn), cos(phiIn), 0);
    LUV::Vec3< Float >obsPol = cos(beta) * eTheta + sin(beta) * ePhi;
    LUV::Vec3< Float >rayPol = cos(alpha) * eTheta + sin(alpha) * ePhi;
    Observation< Float >observation(obsDirection, obsPol, rayPol, f0, fH, 30);

    Excitation< Float > guassPulse(fH, fL);
    guassPulse.Initialize(reducedBvhArray);

    RcsArray< Float > rcsArray;
    rcsArray.Initialize(guassPulse.sampleNums_);

    SbrSolver< Float > sbrSolver;
    /*RayTubeDie< Float > ray;
    ray.dir_ = LUV::Vec3< Float >(0.6124, 0.6124, 0.5000);
    ray.pol_ = LUV::Vec3< Float >(0.3536, 0.3536, -0.8660);
    ray.pos_ = LUV::Vec3< Float >(0.0986, -0.1739, 0.0035);
    LUV::Vec3< Float > normal(0.4954, -0.8686, 0.0070);
    Param< Float > param(std::complex< Float >(3.0, 0.0), std::complex< Float >(1.0, 0.0));
    RayPoolDie< Float > rayPool;
    rayPool.rayArea_ = 5.4932e-05;
    Float txd = 0.0f;
    Float hitArea = 2.7772e-04;
    sbrSolver.TraceInDieleAndEs(ray, normal, param, guassPulse, reducedBvhArray, rayPool, observation, txd, hitArea);*/
    sbrSolver.MonostaticRcsGpuAccDieMemoryLess(trigMesh, reducedBvhArray, observation, rcsArray, guassPulse);



    auto tTotal = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart).count();

    std::cout << "## Finished in " << tTotal << " ms. ##" << std::endl;


}
