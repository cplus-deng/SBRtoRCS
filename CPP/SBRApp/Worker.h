#ifndef _SILENCE_AMP_DEPRECATION_WARNINGS
#define _SILENCE_AMP_DEPRECATION_WARNINGS
#endif // !_SILENCE_AMP_DEPRECATION_WARNINGS
#pragma once
#include <qobject.h>
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
class Worker :public QObject
{
	Q_OBJECT
public:
	Worker(const std::vector< std::string >& metalMeshVecs,
		const std::vector< std::string >& coatingMeshVecs,
		const std::vector< std::string >& dielectMeshVecs,
		const std::vector< ParamCoat< Float > >& coatingParams,
		const std::vector< Param< Float > >& dielectParams,
		const Float& thetaIn, const Float& phiIn, const Float& polIn, 
		const Float& polObs, const Float& rayPerLambda, 
		const Float& fH, const Float& fL,
		const bool& enterInTr);
	Worker(const std::vector< std::string >& metalMeshVecs,
		const std::vector< std::string >& coatingMeshVecs,
		const std::vector< std::string >& dielectMeshVecs,
		const std::vector< ParamCoat< Float > >& coatingParams,
		const std::vector< Param< Float > >& dielectParams,
		const Float& thetaIn, const Float& phiIn, const Float& polIn,
		const Float& polObs, const Float& polObsX, const Float& rayPerLambda,
		const Float& fH, const Float& fL,
		const bool& enterInTr);
	void LoadMesh();
	void GenBvh();
	void GenObsPolPulse();
	void SBRtoRCS();

public:
	static Float pi_;
private:
	std::vector< std::string > metalMeshVecs_;
	std::vector< std::string > coatingMeshVecs_;
	std::vector< std::string > dielectMeshVecs_;
	std::vector< ParamCoat< Float > > coatingParams_;
	std::vector< Param< Float > > dielectParams_;
	Float thetaIn_;
	Float phiIn_;
	Float polIn_;
	Float polObs_;
	Float polObsX_;
	Float rayPerLambda_;
	U32 polObsCnt_;

	Float fH_;
	Float fL_;

	bool enterInTr_;

	std::vector< StlTrigMeshFile< Float > > stlFileVec_;

	//StlTrigMeshFile<Float> stlFile;
	TriangleMesh< Float > trigMesh_;
	ReducedBvhArray< Float > reducedBvhArray_;
	Observation< Float >observation_;
	Excitation< Float > guassPulse_;
	RcsArray< Float > rcsArray_;

signals:
	//void calculationCompleted(const QString& result);
	void calculationCompleted(const std::vector<Float>& result,const U32&);
public slots:
	void doWork();

};

