#include "Worker.h"
Worker::Worker(const std::vector< std::string >& metalMeshVecs,
	const std::vector< std::string >& coatingMeshVecs,
	const std::vector< std::string >& dielectMeshVecs,
	const std::vector< ParamCoat< Float > >& coatingParams,
	const std::vector< Param< Float > >& dielectParams,
	const Float& thetaIn, const Float& phiIn, const Float& polIn,
	const Float& polObs, const Float& rayPerLambda,
	const Float& fH, const Float& fL,
	const bool& enterInTr)
{
	metalMeshVecs_ = metalMeshVecs;
	coatingMeshVecs_ = coatingMeshVecs;
	dielectMeshVecs_ = dielectMeshVecs;
	coatingParams_ = coatingParams;
	dielectParams_ = dielectParams;
	thetaIn_ = thetaIn;
	phiIn_ = phiIn;
	polIn_ = polIn;
	polObs_ = polObs;
	polObsCnt_ = 1;
	rayPerLambda_ = rayPerLambda;
	fH_ = fH;
	fL_ = fL;
	enterInTr_ = enterInTr;
}
Worker::Worker(const std::vector< std::string >& metalMeshVecs,
	const std::vector< std::string >& coatingMeshVecs,
	const std::vector< std::string >& dielectMeshVecs,
	const std::vector< ParamCoat< Float > >& coatingParams,
	const std::vector< Param< Float > >& dielectParams,
	const Float& thetaIn, const Float& phiIn, const Float& polIn,
	const Float& polObs, const Float& polObsX, const Float& rayPerLambda,
	const Float& fH, const Float& fL,
	const bool& enterInTr)
{
	metalMeshVecs_ = metalMeshVecs;
	coatingMeshVecs_ = coatingMeshVecs;
	dielectMeshVecs_ = dielectMeshVecs;
	coatingParams_ = coatingParams;
	dielectParams_ = dielectParams;
	thetaIn_ = thetaIn;
	phiIn_ = phiIn;
	polIn_ = polIn;
	polObs_ = polObs;
	polObsX_ = polObsX;
	polObsCnt_ = 2;
	rayPerLambda_ = rayPerLambda;
	fH_ = fH;
	fL_ = fL;
	enterInTr_ = enterInTr;
}

void Worker::LoadMesh()
{
	if (!metalMeshVecs_.empty()) 
	{
		for (U32 i = 0; i < metalMeshVecs_.size(); i++)
		{
			StlTrigMeshFile<Float> stlFile;
			stlFile.Load(metalMeshVecs_[i], 0);
			stlFileVec_.push_back(stlFile);
		}
	}
	if (!coatingMeshVecs_.empty())
	{
		int coatKind = 100;
		for (U32 i = 0; i < coatingMeshVecs_.size(); i++)
		{
			StlTrigMeshFile<Float> stlFile;
			stlFile.Load(coatingMeshVecs_[i], coatKind);
			stlFileVec_.push_back(stlFile);
			coatKind++;
		}
	}

	if (!dielectMeshVecs_.empty())
	{
		int dieleKind = 200;
		for (U32 i = 0; i < dielectMeshVecs_.size(); i++)
		{
			StlTrigMeshFile<Float> stlFile;
			stlFile.Load(dielectMeshVecs_[i], dieleKind);
			stlFileVec_.push_back(stlFile);
			dieleKind++;
		}
	}
	trigMesh_.ImportFromStlTrigMeshFileVec(stlFileVec_);
}
void Worker::GenBvh()
{
	MortonManager< Float > mortonManager(&trigMesh_);
	mortonManager.GenerateMortonArray();

	BvhGenerator< Float > bvhGenerator(&mortonManager);
	bvhGenerator.GenerateBvhArray();
	bvhGenerator.SqueezeBvhArray();
	bvhGenerator.RemoveEmptyNodes();

	bvhGenerator.PopulateReducedBvhArray(reducedBvhArray_);
}

void Worker::GenObsPolPulse()
{
	//wave in params
	/*Float thetaIn = pi_ / 4;
	Float phiIn = pi_ / 4;*/
	Float alpha = polIn_;  //polar
	/*Float fH = 4e9;
	Float fL = 2e9;*/
	Float f0 = (fH_ + fL_) / 2;

	//observation
	Float thetaS = pi_ - thetaIn_;
	Float phiS = pi_ + phiIn_;
	Float beta = polObs_;  //polar

	LUV::Vec3< Float >obsDirection(sin(thetaS) * cos(phiS),
		sin(thetaS) * sin(phiS),
		cos(thetaS));

	LUV::Vec3< Float >eTheta(cos(thetaIn_) * cos(phiIn_),
		cos(thetaIn_) * sin(phiIn_),
		-sin(thetaIn_));
	LUV::Vec3< Float >ePhi(-sin(phiIn_), cos(phiIn_), 0);
	LUV::Vec3< Float >obsPol = cos(beta) * eTheta + sin(beta) * ePhi;
	LUV::Vec3< Float >rayPol = cos(alpha) * eTheta + sin(alpha) * ePhi;
	if (polObsCnt_ == 1)
	{
		observation_ = Observation< Float >(obsDirection, obsPol, rayPol, f0, rayPerLambda_);
	}
	else //2
	{
		Float betaX = polObsX_;
		LUV::Vec3< Float >obsPolX = cos(betaX) * eTheta + sin(betaX) * ePhi;
		observation_ = Observation< Float >(obsDirection, obsPol, obsPolX, rayPol, f0, rayPerLambda_);
	}

	guassPulse_ = Excitation< Float >(fH_, fL_);
	guassPulse_.Initialize(reducedBvhArray_);

	rcsArray_.Initialize(guassPulse_.sampleNums_);
}
void Worker::SBRtoRCS()
{
	SbrSolver< Float > sbrSolver;
	if (polObsCnt_ == 1)
	{
		sbrSolver.MonostaticRcsGpuAccDieGUI(trigMesh_, enterInTr_, reducedBvhArray_, observation_, rcsArray_, guassPulse_, dielectParams_, coatingParams_);
	}
	else
	{
		sbrSolver.MonostaticRcsGpuAccDieGUI2Pol(trigMesh_, enterInTr_, reducedBvhArray_, observation_, rcsArray_, guassPulse_, dielectParams_, coatingParams_);
	}
	//sbrSolver.MonostaticRcsGpuAccDieGUI(enterInTr_,reducedBvhArray_, observation_, rcsArray_, guassPulse_, dielectParams_, coatingParams_);

}
void Worker::doWork()
{
	LoadMesh();
	GenBvh();
	GenObsPolPulse();
	SBRtoRCS();
	std::vector< Float >result(rcsArray_.rcsArray_.get(), rcsArray_.rcsArray_.get() + rcsArray_.rcsCount_);
	emit calculationCompleted(result, guassPulse_.sampleNums_);
	
}

Float Worker::pi_= 3.14159265359;