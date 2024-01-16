#ifndef SBR_SOLVER_INCLUDED
#define SBR_SOLVER_INCLUDED

#include <complex>
#include "omp.h"

#include "TypeDef.hpp"
#include "Triangle.hpp"
#include "TriangleMesh.hpp"
#include "MortonManager.hpp"
#include "BvhNodeTypes.hpp"
#include "ReducedBvhArray.hpp"
#include "RayPool.hpp"
#include "RcsArray.hpp"
#include "ObservationArray.hpp"
#include "Excitation.h"
#include "FFT.h"

template< class T >
class SbrSolver
{

public:
	static T c0;
	static T mu0;
	static T eps0;
	static T z0;
	static T pi;
	static T eps;
	static int N_Tracing_Max;
	static int N_Tracing_Max_In;
	//static T epsilonR;
	//static T muR;
	//static std::complex< T > epsilonRC;
	//static std::complex< T > muRC;
	
public:

	SbrSolver()
	{

	}

	~SbrSolver()
	{

	}
	enum class InDiele{ BEFOREIN = 0, INDIE = 1};
	void MonostaticRcsGpuAccDieGUI(TriangleMesh< T >& trigMesh, const bool& enterInTr, const ReducedBvhArray< T >& bvhArray, const Observation< T >& obs, RcsArray< T >& rcsArray, Excitation< T >& excite, std::vector< Param< Float > >& dieParams, std::vector< ParamCoat< Float > >& coatParams)
	{
		//bool enterInTr = false;
		//Param< T > param(std::complex< T >(3.0, -10.0), std::complex< T >(1.0, 0.0));
		RayPoolDie< T > rayPool;
		PopulateRayPoolAccDie(bvhArray, obs, rayPool);
		//RayValid(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		RayValid2CondDie(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		ShootAndBounceRaysAccDieGUI(bvhArray, rayPool, obs, rcsArray, excite, enterInTr, dieParams, coatParams);
		RcsTimeToFreqDomainDie(trigMesh, obs, rayPool, rcsArray, excite);
	}
	void MonostaticRcsGpuAccDieGUI2Pol(TriangleMesh< T >& trigMesh, const bool& enterInTr, const ReducedBvhArray< T >& bvhArray, const Observation< T >& obs, RcsArray< T >& rcsArray, Excitation< T >& excite, std::vector< Param< Float > >& dieParams, std::vector< ParamCoat< Float > >& coatParams)
	{
		//bool enterInTr = false;
		//Param< T > param(std::complex< T >(3.0, -10.0), std::complex< T >(1.0, 0.0));
		RayPoolDie< T > rayPool;
		PopulateRayPoolAccDie(bvhArray, obs, rayPool);
		//RayValid(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		RayValid2CondDie(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		ShootAndBounceRaysAccDieGUI2Pol(bvhArray, rayPool, obs, rcsArray, excite, enterInTr, dieParams, coatParams);
		RcsTimeToFreqDomainDie(trigMesh, obs, rayPool, rcsArray, excite, 2);
	}
	//void MonostaticRcsGpuAcc(const ReducedBvhArray< T >& bvhArray, const Observation< T >& obs, RcsArray< T >& rcsArray, Excitation< T >& excite)
	//{
	//	RayPool< T > rayPool;
	//	PopulateRayPoolAcc(bvhArray, obs, rayPool);
	//	RayValid(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
	//	//RayValid2Cond(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
	//	ShootAndBounceRaysAcc(bvhArray, rayPool, obs, rcsArray, excite);
	//	RcsTimeToFreqDomain(rayPool, rcsArray, excite);
	//}
	void MonostaticRcsGpuAccDie(TriangleMesh< T >& trigMesh, const ReducedBvhArray< T >& bvhArray, const Observation< T >& obs, RcsArray< T >& rcsArray, Excitation< T >& excite)
	{
		
		bool enterInTr = false;
		Param< T > param(std::complex< T >(3.0, -300.0), std::complex< T >(1.0, 0.0));
		RayPoolDie< T > rayPool;
		PopulateRayPoolAccDie(bvhArray, obs, rayPool);
		//RayValid(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		auto tStart1 = Clock::now();
		//RayValid2CondDie(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		RayValidGPUNew(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		auto tTotal1 = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart1).count();
		std::cout << "### Ray Tube Valid Judge Done###\n";
		std::cout << "## Finished in " << tTotal1 << " ms. ##" << std::endl;
		auto tStart2 = Clock::now();
		ShootAndBounceRaysAccDie(bvhArray, rayPool, obs, rcsArray, excite, enterInTr, param);
		auto tTotal2 = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart2).count();
		std::cout << "### Shoot And Bounce Done###\n";
		std::cout << "## Finished in " << tTotal2 << " ms. ##" << std::endl;
		auto tStart3 = Clock::now();
		RcsTimeToFreqDomainDie(trigMesh, obs, rayPool, rcsArray, excite);
		auto tTotal3 = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart3).count();
		std::cout << "### Rcs TimeFreqDomain Done###\n";
		std::cout << "## Finished in " << tTotal3 << " ms. ##" << std::endl;
		std::cout << "SIGNAL\n";
		std::exit(0);
	}
	void MonostaticRcsGpuAccDieMemoryLess(TriangleMesh< T >& trigMesh, const ReducedBvhArray< T >& bvhArray, const Observation< T >& obs, RcsArray< T >& rcsArray, Excitation< T >& excite)
	{

		bool enterInTr = false;
		Param< T > param(std::complex< T >(3.0, -300.0), std::complex< T >(1.0, 0.0));
		RayPoolDie< T > rayPool;
		PopulateRayPoolAccDie(bvhArray, obs, rayPool);
		//RayValid(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		auto tStart1 = Clock::now();
		//RayValid2CondDie(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		RayValidGPUNew(bvhArray, rayPool, 0.8, 0.3 * c0 / obs.frequency_);
		auto tTotal1 = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart1).count();
		std::cout << "### Ray Tube Valid Judge Done###\n";
		std::cout << "## Finished in " << tTotal1 << " ms. ##" << std::endl;
		auto tStart2 = Clock::now();
		ShootAndBounceRaysAccDieMemoryLess(bvhArray, rayPool, obs, rcsArray, excite, enterInTr, param);
		auto tTotal2 = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart2).count();
		std::cout << "### Shoot And Bounce Done###\n";
		std::cout << "## Finished in " << tTotal2 << " ms. ##" << std::endl;
		auto tStart3 = Clock::now();
		RcsTimeToFreqDomainDieMemoryLess(trigMesh, obs, rayPool, rcsArray, excite);
		auto tTotal3 = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tStart3).count();
		std::cout << "### Rcs TimeFreqDomain Done###\n";
		std::cout << "## Finished in " << tTotal3 << " ms. ##" << std::endl;
		std::cout << "SIGNAL\n";
		std::exit(0);
	}
	/*void MonostaticRcsGpu( const ReducedBvhArray< T >& bvhArray, const ObservationArray< T >& obsArray, RcsArray< T >& rcsArray )
	{
		U32 obsCount = obsArray.obsCount_;
		Observation< T >* obsPtr = obsArray.observationArray_.get();
		T* rcsPtr = rcsArray.rcsArray_.get();
		RayPool< T > rayPool;

		for( U32 idx = 0; idx < obsCount; ++idx )
		{
			PopulateRayPool( bvhArray, obsPtr[ idx ], rayPool );
			ShootAndBounceRaysGpu( bvhArray, rayPool );
			PhysicalOpticsIntegral( rayPool, obsPtr[ idx ], rcsPtr[ idx ] );
		}
	}*/

	void TraceInDieleAndEs(RayTubeDie< T >& ray, 
		const LUV::Vec3< T >& normal, 
		const Param< T >& param, 
		const Excitation< T >& excite, 
		const ReducedBvhArray< T >& bvhArray, 
		RayPoolDie< T >& rayPool, 
		const Observation< T >& obs,
		const T& txd,
		const T& hitArea)
	{
		U32 stateRay = 0;
		RayTubeDie< T > rayInDiele;
		U32 sampleNums = excite.sampleNums_;
		std::complex< T > zeroComplex(0.0, 0.0);
		rayInDiele.es_.reset(new std::complex< T >[sampleNums], [](std::complex< T >* ptr) { delete[] ptr; });
		std::fill(rayInDiele.es_.get(), rayInDiele.es_.get() + sampleNums, zeroComplex);

		LUV::Vec3< T > emptyVec(1.0, 1.0, 1.0);
		LUV::Vec3< T > ktAlpha;
		T area3 = hitArea;
		T thetaI1;
		T thetaT1;
		DielectricTr(InDiele::BEFOREIN, ray, rayInDiele, ktAlpha, normal, obs, param, 1.0, emptyVec, rayPool.rayArea_, txd, excite, area3, thetaI1, thetaT1);

		//Begin Ray Trace

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();

		T* stdt = excite.DeriveSeriesArray_.get();
		/*T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;*/

		bool isHitAtAll = true;

		while (isHitAtAll && rayInDiele.refCount_ < N_Tracing_Max_In)
		{
			U32 meterialType = 1;
			isHitAtAll = false;

			U32 stackArray[32];
			I32 stackIdx = 0;
			stackArray[stackIdx] = 0;

			T hitDistMin = 1E32;
			U32 hitIdx = -1;
			LUV::Vec3< T > hitPointMin;

			while (stackIdx >= 0)
			{
				bool isHit = false;
				T hitDist = 0;

				const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

				if (node.status_ == BRANCH || node.status_ == ROOT)
				{
					rayInDiele.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
					if (isHit && hitDist < hitDistMin)
					{
						stackArray[stackIdx++] = node.data_.leftChildIdx_;
						stackArray[stackIdx++] = node.data_.rightChildIdx_;
					}
				}
				else if (stackArray[stackIdx] != ray.lastHitIdx_)
				{
					LUV::Vec3< T > hitPoint;
					rayInDiele.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
					if (isHit && hitDist < hitDistMin)
					{
						isHitAtAll = true;
						hitDistMin = hitDist;
						hitPointMin = hitPoint;
						hitIdx = stackArray[stackIdx];
						meterialType = node.trig_.mtype_;
					}
				}

				--stackIdx;
			}

			if (isHitAtAll)
			{
				const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
				LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();

				//rayInDiele.dir_ should be unit vector.
				//T thetaI = LUV::_Acos(LUV::Dot(hitNormal, -rayInDiele.dir_));
				if (std::abs(LUV::Length(rayInDiele.pol_)) <= 0.0001)
				{
					break;
				}
				DielectricTr(InDiele::INDIE, ray, rayInDiele, ktAlpha, hitNormal, obs, param, hitDistMin, hitPointMin, rayPool.rayArea_, txd, excite, area3, thetaI1, thetaT1);
			}

		}
		std::complex< T >* esRayPtr = ray.es_.get();
		std::complex< T >* esRayInDiePtr = rayInDiele.es_.get();
		for (U32 i = 0; i < sampleNums; i++)
		{
			esRayPtr[i] += esRayInDiePtr[i];
		}
	}
	void DielectricTr(const InDiele& state, 
		const RayTubeDie< T >& ray, 
		RayTubeDie< T >& rayInDiele, 
		LUV::Vec3< T >& ktAlpha, 
		const LUV::Vec3< T >& normal_, 
		const Observation< T >& obs, 
		const Param< T >& param, 
		const T& hitDistMin, 
		const LUV::Vec3< T >& hitPointMin, 
		const T& rayArea, const T& txd, 
		const Excitation< T >& excite,
		T& area3,
		T& thetaI1,
		T& thetaT1)
	{
		LUV::Vec3< T >kin;
		LUV::Vec3< std::complex< T > > Ein;
		const T fre = obs.frequency_;
		T k0 = 2 * pi * fre / c0;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const T deltaT = excite.deltaTime_;
		T* stdt = excite.DeriveSeriesArray_.get();
		std::complex< T > epsilonR, muR, epsilonRC, muRC;
		LUV::Vec3< T > normal;
		if (state == InDiele::BEFOREIN) 
		{
			epsilonR = 1.0;
			muR = 1.0;

			epsilonRC = param.epsilon_;
			muRC = param.mu_;

			kin = ray.dir_;
			Ein = ray.pol_;

			rayInDiele.delay_ = ray.delay_;
			rayInDiele.dist_ = 0.0;
			rayInDiele.pos_ = ray.pos_;
			rayInDiele.lastHitIdx_ = ray.lastHitIdx_;
			rayInDiele.refCount_ = 0;

			normal = normal_;

			ktAlpha = kin;
		}
		else if(state == InDiele::INDIE)
		{
			epsilonR = param.epsilon_;
			muR = param.mu_;

			epsilonRC = 1.0;
			muRC = 1.0;

			kin = rayInDiele.dir_;
			Ein = rayInDiele.pol_;
			normal = -normal_;
		}

		std::complex< T >j(0.0, 1.0);
		//hitNormal's direction, when INDIE,
		//T thetaIn = LUV::_Acos(LUV::Dot(normal, (-kin)));
		//T thetaIn = LUV::_Acos(std::max(-1.0, std::min(LUV::Dot(normal, (-kin)), 1.0)));
		//T thetaIn = std::max(-1.0, std::min(LUV::Dot(normal, (-kin)), 1.0))
		//std::max(-1.0, std::min(cos_theta, 1.0));
		T val = LUV::Dot(normal, (-kin));
		T thetaIn = LUV::_Acos(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
		T kexi1 = thetaIn;
		T kexi1_kexi;
		std::complex< T > rV, rP, tV, tP;
		T thetaT, thetaTAlpha;
		//if (param.epsilon_.imag() == 0.0f && param.mu_.imag() == 0.0f)
		//{
		//	if (state == InDiele::BEFOREIN)
		//	{
		//		thetaT = LUV::_Asin(LUV::_Sin(thetaIn) / std::sqrtf(param.epsilon_.real() * param.mu_.real()));
		//	}
		//	else
		//	{
		//		thetaT = LUV::_Asin(LUV::_Sin(thetaIn) * std::sqrtf(param.epsilon_.real() * param.mu_.real()));
		//	}
		//	T eta1 = std::sqrtf(muR.real() / epsilonR.real());
		//	T eta2 = std::sqrtf(muRC.real() / epsilonRC.real());

		//	rP = (eta1 * LUV::_Cos(thetaIn) - eta2 * LUV::_Cos(thetaT)) / (eta1 * LUV::_Cos(thetaIn) + eta2 * LUV::_Cos(thetaT));  
		//	//R_p=(eta1*cos(theta_i)-eta2*cos(theta_t))/(eta1*cos(theta_i)+eta2*cos(theta_t));
		//	rV = (eta2 * LUV::_Cos(thetaIn) - eta1 * LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaIn) + eta1 * LUV::_Cos(thetaT));  
		//	//R_v=(eta2*cos(theta_i)-eta1*cos(theta_t))/(eta2*cos(theta_i)+eta1*cos(theta_t))

		//	tP = (2.0 * eta2 * LUV::_Cos(thetaIn)) / (eta1 * LUV::_Cos(thetaIn) + eta2 * LUV::_Cos(thetaT));
		//	//T_p=2*eta2*cos(theta_i)/(eta1*cos(theta_i)+eta2*cos(theta_t))
		//	tV = (2.0 * eta2 * LUV::_Cos(thetaIn)) / (eta2 * LUV::_Cos(thetaIn) + eta1 * LUV::_Cos(thetaT));
		//	//T_v=2*eta2*cos(theta_i)/(eta2*cos(theta_i)+eta1*cos(theta_t))
		//	thetaTAlpha = thetaT;

		//	kexi1_kexi = LUV::_Asin(k0 / (k0 * std::sqrtf(param.epsilon_.real() * param.mu_.real())));
		//	//kexi1_kexi=asin(k0/(k0*(mu_r1*epsilon_r1)^0.5))
		//	if (std::isnan(tV.real()))
		//	{
		//		int dy = 6;
		//	}
		//}
		if(true)
		{
			//T zeta1 = kexi1;
			val = LUV::Dot(normal, (-ktAlpha));
			T zeta1 = LUV::_Acos(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
			T rho1 = zeta1 - kexi1;

			std::complex< T > gama01 = j * k0 * std::sqrt(epsilonR * muR);
			T alpha01 = gama01.real();
			T beta01 = gama01.imag();

			T alpha1 = std::sqrtf((pow(beta01, 2) - pow(alpha01, 2)) / 2);
			alpha1 = alpha1 * std::sqrtf(std::sqrtf(1 + pow(2 * alpha01 * beta01 / ((pow(beta01, 2) - pow(alpha01, 2)) * LUV::_Cos(rho1)), 2)) - 1); //---change--- LUV::_Cos(rho1)

			T beta1 = std::sqrtf((pow(beta01, 2) - pow(alpha01, 2)) / 2);
			beta1 = beta1 * std::sqrtf(std::sqrtf(1 + pow(2 * alpha01 * beta01 / ((pow(beta01, 2) - pow(alpha01, 2)) * LUV::_Cos(rho1)), 2)) + 1); //---change--- LUV::_Cos(rho1)

			std::complex< T > gama1t = alpha1 * LUV::_Sin(zeta1) + j * beta1 * LUV::_Sin(kexi1);

			std::complex< T > gama02 = j * k0 * std::sqrt(epsilonRC * muRC);
			T alpha02 = gama02.real();
			T beta02 = gama02.imag();

			T alpha2; T beta2;
			//T g1 = pow(std::abs(gama1t), 2);
			//T g2 = pow(alpha02, 2) - pow(beta02, 2);
			//T g3 = std::abs(pow(gama1t, 2) - pow(gama02, 2));

			//alpha2 = std::sqrtf((g1 + g2 + g3) / 2);  //(g1 + g2 + g3) / 2 < 0 ->alpha2 = nan
			//beta2 = std::sqrtf((g1 - g2 + g3) / 2);
			//if ((std::isnan(alpha2)))
			//{
			//	alpha2 = 0.0;
			//}
			if (state == InDiele::BEFOREIN)
			{
				T g1 = pow(std::abs(gama1t), 2);
				T g2 = pow(alpha02, 2) - pow(beta02, 2);
				T g3 = std::abs(pow(gama1t, 2) - pow(gama02, 2));

				alpha2 = std::sqrtf((g1 + g2 + g3) / 2);  //(g1 + g2 + g3) / 2 < 0 ->alpha2 = nan
				beta2 = std::sqrtf((g1 - g2 + g3) / 2);

				/*if ((std::isnan(alpha2)))
				{
					alpha2 = 0.0;
				}
				if ((std::isnan(beta2)))
				{
					beta2 = 0.0;
				}*/
			}
			else if (state == InDiele::INDIE)
			{
				alpha2 = 0.0;
				beta2 = beta02;
				/*if ((std::isnan(alpha2)))
				{
					alpha2 = 0.0;
				}
				if ((std::isnan(beta2)))
				{
					beta2 = 0.0;
				}*/
			}
			/*T kexi1_kexi;
			if (beta2 > beta1)
			{
				kexi1_kexi = pi / 2;
			}
			else
			{
				kexi1_kexi = LUV::_Asin(beta2 / beta1);
			}

			T kexi1_zeta;
			if (alpha2 > alpha1)
			{
				kexi1_zeta = pi / 2;
			}
			else
			{
				if (alpha1 == 0)
				{
					kexi1_zeta = kexi1_kexi;
				}
				else
				{
					kexi1_zeta = LUV::_Asin(alpha2 / alpha1) - rho1;
				}
			}

			T kexi2;
			if (kexi1 < kexi1_kexi)
			{
				kexi2 = LUV::_Asin(beta1 * LUV::_Sin(kexi1) / beta2);
			}
			else
			{
				kexi2 = pi - LUV::_Asin(beta1 * LUV::_Sin(kexi1) / beta2);
			}

			T zeta2;
			if (alpha2 == 0)
			{
				zeta2 = kexi2;
			}
			else
			{
				if (kexi1 < kexi1_zeta)
				{
					zeta2 = LUV::_Asin(alpha1 * LUV::_Sin(zeta1) / alpha2);
				}
				else
				{
					zeta2 = pi - LUV::_Asin(alpha1 * LUV::_Sin(zeta1) / alpha2);
					if (std::isnan(zeta2))
					{
						zeta2 = 0.0;
					}
				}
			}*/
			val = beta1 * LUV::_Sin(kexi1) / beta2;
			T kexi2 = LUV::_Asin(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
			//T kexi2 = LUV::_Asin(beta1 * LUV::_Sin(kexi1) / beta2);
			T zeta2;
			if (state == InDiele::BEFOREIN)
			{
				if (abs(alpha2) < eps)
				{
					zeta2 = kexi2;
				}
				else
				{
					zeta2 = LUV::_Asin(alpha1 * LUV::_Sin(zeta1) / alpha2);
				}
			}
			else if (state == InDiele::INDIE)
			{
				zeta2 = kexi2;
			}
			if (std::isnan(zeta2))
			{
				zeta2 = 0.0;
			}
			T rho2 = zeta2 - kexi2;
			std::complex< T > twoComplx(2.0, 0.0);
			std::complex< T > rTmp1 = muRC * (alpha1 * LUV::_Cos(zeta1) + j * beta1 * LUV::_Cos(kexi1));
			std::complex< T > rTmp2 = muR * (alpha2 * LUV::_Cos(zeta2) + j * beta2 * LUV::_Cos(kexi2));
			rV = (rTmp1 - rTmp2) / (rTmp1 + rTmp2);
			tV = twoComplx * rTmp1 / (rTmp1 + rTmp2);

			std::complex< T > rTmp1p = epsilonRC * (alpha1 * LUV::_Cos(zeta1) + j * beta1 * LUV::_Cos(kexi1));
			std::complex< T > rTmp2p = epsilonR * (alpha2 * LUV::_Cos(zeta2) + j * beta2 * LUV::_Cos(kexi2));
			rP = (rTmp1p - rTmp2p) / (rTmp1p + rTmp2p);  //change  -(rTmp1p - rTmp2p) -->(rTmp1p - rTmp2p)
			tP = twoComplx * rTmp1p / (rTmp1p + rTmp2p) * std::sqrt(muRC * epsilonR / (muR * epsilonRC));
			kexi1_kexi = LUV::_Asin(beta2 / beta1);

			thetaT = kexi2;

			thetaTAlpha = zeta2;

			if (state == InDiele::INDIE)
			{
				T attenuation = std::exp(-alpha1 * LUV::Dot(ktAlpha, hitDistMin * kin));
				Ein = Ein * attenuation;
			}
		}

		LUV::Vec3< std::complex< T > > polR;
		LUV::Vec3< std::complex< T > > polT;

		LUV::Vec3< T > vi = LUV::Cross(kin, normal);
		LUV::Vec3< T > vt = LUV::Unit(LUV::Cross(normal, vi));

		LUV::Vec3< T > kt;

		if (abs(LUV::Dot(-kin, normal) - 1.0) > eps)
		{
			kt = LUV::Unit(vt * LUV::_Sin(thetaT) - normal * LUV::_Cos(thetaT));
			LUV::Vec3< std::complex< T > > ei_v = LUV::Unit(LUV::CrossX(normal, kin));
			LUV::Vec3< std::complex< T > > ei_p = LUV::CrossX(kin, ei_v);

			LUV::Vec3< T > kr = LUV::Unit(kin - normal * (2.0 * LUV::Dot(kin, normal)));
			LUV::Vec3< std::complex< T > > er_v = ei_v;
			LUV::Vec3< std::complex< T > > er_p = LUV::CrossX(kr, er_v);
			std::complex< T > polV = LUV::Dot(Ein, ei_v);
			std::complex< T > polP = LUV::Dot(Ein, ei_p);

			LUV::Vec3< std::complex< T > > et_v = ei_v;
			LUV::Vec3< std::complex< T > > et_p = LUV::CrossX(kt, et_v);
			std::complex< T > polTP = polP * tP;
			std::complex< T > polTV = polV * tV;
			//polT = polTV * et_v + polTP * et_p;
			polT = et_v * polTV + et_p * polTP;

			std::complex< T > polRP = polP * rP;
			std::complex< T > polRV = polV * rV;
			//polR = polRV * er_v + polRP * er_p;
			polR = er_v * polRV + er_p * polRP;

			/*LUV::Vec3< T > kr = LUV::Unit(kin - 2 * normal * LUV::Dot(kin, normal));
			LUV::Vec3< T > er_h = LUV::Cross(kr, er_v);*/
			if (state == InDiele::BEFOREIN)
			{
				ktAlpha = LUV::Unit(vt * LUV::_Sin(thetaTAlpha) - normal * LUV::_Cos(thetaTAlpha));
			}
			else if (state == InDiele::INDIE)
			{
				ktAlpha = LUV::Unit(ktAlpha - 2.0f * normal * LUV::Dot(ktAlpha, normal));
			}
		}
		else
		{
			polR = rV * Ein;
			polT = tV * Ein;
			kt = kin;
			ktAlpha = kin;
		}
		if (std::isnan(polR[0].real()) || std::isnan(polR[0].imag()) || std::isnan(polT[0].real()) || std::isnan(polT[0].imag())) {

			int dy = 6;
		}
		if (state == InDiele::BEFOREIN)
		{
			rayInDiele.dir_ = kt;
			rayInDiele.pol_ = polT;

			thetaI1 = thetaIn;
			thetaT1 = thetaT;
			if (std::isnan(polT[0].real()))
			{
				int dy = 6;
			}
		}
		else if( state == InDiele::INDIE)
		{
			LUV::Vec3< T > refDir = LUV::Unit(rayInDiele.dir_ - normal * (2.0 * LUV::Dot(rayInDiele.dir_, normal)));
			rayInDiele.dir_ = refDir;
			rayInDiele.pol_ = polR;
			rayInDiele.refCount_ += 1;
			rayInDiele.pos_ = hitPointMin;

			if (std::isnan(polR[0].real()))
			{
				int dy = 6;
			}
			if (kexi1 >= kexi1_kexi)
			{
				return;
			}
			if (std::isnan(polR[0].real()))
			{
				int dy = 6;
			}
			if (rayInDiele.refCount_ == 1)
			{
				T area1 = area3;
				T area2 = area1 * LUV::_Cos(thetaT1);
				area3 = area2 / LUV::_Cos(thetaIn);
				thetaI1 = thetaIn;
				thetaT1 = thetaT;
			}
			else
			{
				T area1 = area3;
				T area2 = area1 * LUV::_Cos(thetaI1);
				area3 = area2 / LUV::_Cos(thetaIn);
				thetaI1 = thetaIn;
				thetaT1 = thetaT;
			}

			//T DF = std::sqrt(1.0 / LUV::_Cos(thetaIn));
			T DF = std::sqrtf(rayArea / area3);
			T sigma = 0.5 * (LUV::_Atan(-param.epsilon_.imag() / param.epsilon_.real())
				+ LUV::_Atan(-param.mu_.imag() / param.mu_.real()));
			T vInDie = c0 / (std::sqrt(std::abs(param.epsilon_) * std::abs(param.mu_)) * LUV::_Cos(sigma));
			rayInDiele.delay_ += hitDistMin / vInDie;

			/*std::complex< T > oneMy(1.0, 0.0);
			std::complex< T > etaDie = std::sqrt(param.mu_ / param.epsilon_) * z0;*/
			LUV::Vec3< std::complex< T > > Ht = (1.0 / z0) * LUV::CrossX(kt, polT);

			LUV::Vec3< std::complex< T > > eTotal = polT * DF;
			LUV::Vec3< std::complex< T > > hTotal = Ht * DF;

			LUV::Vec3< std::complex< T > > EQ = LUV::CrossX(ki,
				LUV::CrossX(ki, LUV::CrossX(normal, hTotal))
				+ LUV::CrossX(normal, eTotal) / z0);
			std::complex< T > EQPol = LUV::Dot(EQ, esPol);
			EQPol *= (rayArea * DF * DF);

			T delay2 = LUV::Dot(ki, hitPointMin);
			delay2 /= c0;
			//tx_d=-2*x_r_min 入射离散点平面的位置距离包围盒的最小距
			U32 nDelay = std::ceil((rayInDiele.delay_ + delay2 + txd / c0) / deltaT);

			std::complex< T >* es = rayInDiele.es_.get();
			//quanfanshe bujiruyuanchang
			for (int kk = 0; kk < excite.sampleNums_ ; kk++)
			{
				int kkn = kk - nDelay;
				if (kkn >= 0 && kkn < excite.sampleNums_)
				{
					es[kk] += stdt[kkn] * EQPol;
					if (std::isnan(es[kk].real()) || std::isnan(es[kk].imag()))
					{
						int j = kk;
					}
				}
			}

		}
		
	}

	void PopulateRayPoolAccDie(const ReducedBvhArray< T >& bvhArray, const Observation< T >& observation, RayPoolDie< T >& rayPool)
	{
		BoundBox< T > boundBox = bvhArray.bvhNodeArray_.get()[0].data_.boundBox_;
		//T lambdaWidthMeter = c0 / observation.frequency_;
		T lambdaWidthMeter = c0 / observation.frequencyHigh_;
		T objWidthMeter = boundBox.GetRadius() * 2;
		T objWidthLambda = objWidthMeter / lambdaWidthMeter;
		U32 rayCountSqrt = (U32)(std::ceil(objWidthLambda * (T)(observation.rayPerLam_) + 1.0)) + 1;
		rayPool.Initialize(rayCountSqrt);
		rayPool.ReGenerateRaysAcc(boundBox, observation.direction_, observation.rayPol_);
	}

	void PopulateRayPoolAcc(const ReducedBvhArray< T >& bvhArray, const Observation< T >& observation, RayPool< T >& rayPool)
	{
		BoundBox< T > boundBox = bvhArray.bvhNodeArray_.get()[0].data_.boundBox_;
		T lambdaWidthMeter = c0 / observation.frequency_;
		T objWidthMeter = boundBox.GetRadius() * 2;
		T objWidthLambda = objWidthMeter / lambdaWidthMeter;
		U32 rayCountSqrt = (U32)(std::ceil(objWidthLambda * (T)(observation.rayPerLam_) + 1.0)) + 1;
		rayPool.Initialize(rayCountSqrt);
		rayPool.ReGenerateRaysAcc(boundBox, observation.direction_, observation.rayPol_);
	}

	void PopulateRayPool( const ReducedBvhArray< T >& bvhArray, const Observation< T >& observation, RayPool< T >& rayPool )
	{
		BoundBox< T > boundBox = bvhArray.bvhNodeArray_.get()[ 0 ].data_.boundBox_;
		T lambdaWidthMeter = c0 / observation.frequency_;
		T objWidthMeter = boundBox.GetRadius() * 2;
		T objWidthLambda = objWidthMeter / lambdaWidthMeter;
		U32 rayCountSqrt = (U32)( std::ceil( objWidthLambda * (T)( observation.rayPerLam_ ) + 1.0 ) ) + 1;
		rayPool.Initialize( rayCountSqrt );
		rayPool.ReGenerateRays( boundBox, observation.direction_, observation.polarization_ );
	}

	void RayValidGPUNew(const ReducedBvhArray< T >& bvhArray, RayPoolDie< T >& rayPool, T minDiffNormal, T maxDiffD)
	{
		using namespace concurrency;
		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		//RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();

		array_view< const ReducedBvhNode< T >, 1 > bvhGpu(bvhArray.nodeCount_, bvhPtr);
		array_view< RayTubeCorDie< T >, 1 > rayCornerGpu(rayPool.rayCornerCount_, rayCornerPtr);

		parallel_for_each(rayCornerGpu.extent, [=](index< 1 > idRay) restrict(amp)
			{
				RayTubeCorDie< T >& rayCorner = rayCornerGpu[idRay];
				//auto& refCoordsPtr = rayCorner.refCoords_;
				//auto& hitIdxPtr = rayCorner.hitIdx_;
				bool isHitAtAll = true;

				rayCorner.dist_ = 0;

				while (isHitAtAll && rayCorner.refCount_ < N_Corner)
				{
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhGpu[index< 1 >(stackArray[stackIdx])];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							rayCorner.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != rayCorner.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							rayCorner.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhGpu[index< 1 >(hitIdx)];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						//**LUV::Vec3< T > dirCrossNormal = LUV::Cross(rayCorner.dir_, hitNormal);
						LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, rayCorner.dir_);
						LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
						LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(rayCorner.dir_, polU));

						LUV::Vec3< T > refDir = LUV::Unit(rayCorner.dir_ - hitNormal * (2.0 * LUV::Dot(rayCorner.dir_, hitNormal)));

						//**LUV::Vec3< T > refPolU = -polU;
						LUV::Vec3< T > refPolU = polU;
						LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

						T polCompU = LUV::Dot(rayCorner.pol_, polU);
						T polCompR = LUV::Dot(rayCorner.pol_, polR);
						rayCorner.pos_ = hitPointMin;
						rayCorner.dir_ = refDir;
						rayCorner.pol_ = -polCompR * refPolR + polCompU * refPolU;
						rayCorner.dist_ += hitDistMin;
						rayCorner.refNormal_ = hitNormal;
						//*****Valid judge
						//----------------
						rayCorner.refCoords_[rayCorner.refCount_] = hitPointMin;
						rayCorner.hitIdx_[rayCorner.refCount_] = hitIdx;

						rayCorner.refCount_ += 1;

						rayCorner.lastHitIdx_ = hitIdx;
					}

				}
			}
		);
		rayCornerGpu.synchronize();

		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* cornerPtr = rayPool.rayTubeCornerArray_.get();
		U32 msize = rayPool.rayCountSqrt_;
		U32 cmsize = msize + 1;
		for (U32 i = 0; i < msize; i++)
		{
			for (U32 j = 0; j < msize; j++)
			{
				/*if (i == 25 && j == 31) {
					int dy = 99;
					dy++;
				}*/
				U32 refCount1 = cornerPtr[i * cmsize + j].refCount_;
				U32 refCount2 = cornerPtr[i * cmsize + j + 1].refCount_;
				U32 refCount3 = cornerPtr[(i + 1) * cmsize + j + 1].refCount_;
				U32 refCount4 = cornerPtr[(i + 1) * cmsize + j].refCount_;
				if (((refCount1 & refCount2 & refCount3 & refCount4) == refCount1)
					&& ((refCount1 | refCount2 | refCount3 | refCount4) == refCount1))
				{
					if (refCount1 == 0) {
						continue;
					}
					else
					{

						auto& hitIdxPtr1 = cornerPtr[i * cmsize + j].hitIdx_;
						auto& hitIdxPtr2 = cornerPtr[i * cmsize + j + 1].hitIdx_;
						auto& hitIdxPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].hitIdx_;
						auto& hitIdxPtr4 = cornerPtr[(i + 1) * cmsize + j].hitIdx_;

						/*LUV::Vec3< T >* coordsPtr1 = cornerPtr[i * cmsize + j].refCoords_.get();
						LUV::Vec3< T >* coordsPtr2 = cornerPtr[i * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr4 = cornerPtr[(i + 1) * cmsize + j].refCoords_.get();*/

						for (U32 n = 0; n < refCount1; n++)
						{
							if (((hitIdxPtr1[n] & hitIdxPtr2[n] & hitIdxPtr3[n] & hitIdxPtr4[n]) == hitIdxPtr1[n])
								&& ((hitIdxPtr1[n] | hitIdxPtr2[n] | hitIdxPtr3[n] | hitIdxPtr4[n]) == hitIdxPtr1[n])) {

								rayPtr[i * msize + j].refCount_ = refCount1;
							}
							else
							{
								rayPtr[i * msize + j].refCount_ = 0;
								//T diagDiff = LUV::Length(coordsPtr1[n] - coordsPtr3[n]) + LUV::Length(coordsPtr2[n] - coordsPtr4[n]);
								LUV::Vec3< T > hitNormal1 = bvhPtr[hitIdxPtr1[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal2 = bvhPtr[hitIdxPtr2[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal3 = bvhPtr[hitIdxPtr3[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal4 = bvhPtr[hitIdxPtr4[n]].trig_.GetLoadNormal();
								T normalDiff = LUV::Dot(hitNormal1, hitNormal2) + LUV::Dot(hitNormal2, hitNormal3) + LUV::Dot(hitNormal3, hitNormal4) + LUV::Dot(hitNormal4, hitNormal1);
								normalDiff /= 4;
								if (normalDiff > minDiffNormal)//*******************0.3*Lambda
								{
									rayPtr[i * msize + j].refCount_ = refCount1;
								}
								else
								{
									break;
								}
							}

						}
					}
				}
			}
		}
	}

	void RayValid2CondDie(const ReducedBvhArray< T >& bvhArray, RayPoolDie< T >& rayPool, T minDiffNormal, T maxDiffD)
	{
		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		//RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		int procsNums = omp_get_num_procs();
		#pragma omp parallel for num_threads(procsNums*2-1)
		for (int idRay = 0; idRay < rayPool.rayCornerCount_; idRay++)
		{
			RayTubeCorDie< T >& rayCorner = rayCornerPtr[idRay];
			/*LUV::Vec3< T >* refCoordsPtr = rayCorner.refCoords_.get();
			U32* hitIdxPtr = rayCorner.hitIdx_.get();*/
			auto& refCoordsPtr = rayCorner.refCoords_;
			auto& hitIdxPtr = rayCorner.hitIdx_;
			bool isHitAtAll = true;

			rayCorner.dist_ = 0;

			while (isHitAtAll && rayCorner.refCount_ < N_Tracing_Max)
			{
				isHitAtAll = false;

				U32 stackArray[32];
				I32 stackIdx = 0;
				stackArray[stackIdx] = 0;

				T hitDistMin = 1E32;
				U32 hitIdx = -1;
				LUV::Vec3< T > hitPointMin;

				while (stackIdx >= 0)
				{
					bool isHit = false;
					T hitDist = 0;

					const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

					if (node.status_ == BRANCH || node.status_ == ROOT)
					{
						rayCorner.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
						if (isHit && hitDist < hitDistMin)
						{
							stackArray[stackIdx++] = node.data_.leftChildIdx_;
							stackArray[stackIdx++] = node.data_.rightChildIdx_;
						}
					}
					else if (stackArray[stackIdx] != rayCorner.lastHitIdx_)
					{
						LUV::Vec3< T > hitPoint;
						rayCorner.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
						if (isHit && hitDist < hitDistMin)
						{
							isHitAtAll = true;
							hitDistMin = hitDist;
							hitPointMin = hitPoint;
							hitIdx = stackArray[stackIdx];
						}
					}

					--stackIdx;
				}

				if (isHitAtAll)
				{
					const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
					LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
					//**LUV::Vec3< T > dirCrossNormal = LUV::Cross(rayCorner.dir_, hitNormal);
					LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, rayCorner.dir_);
					LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
					LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(rayCorner.dir_, polU));

					LUV::Vec3< T > refDir = LUV::Unit(rayCorner.dir_ - hitNormal * (2.0 * LUV::Dot(rayCorner.dir_, hitNormal)));

					//**LUV::Vec3< T > refPolU = -polU;
					LUV::Vec3< T > refPolU = polU;
					LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

					T polCompU = LUV::Dot(rayCorner.pol_, polU);
					T polCompR = LUV::Dot(rayCorner.pol_, polR);
					rayCorner.pos_ = hitPointMin;
					rayCorner.dir_ = refDir;
					rayCorner.pol_ = -polCompR * refPolR + polCompU * refPolU;
					rayCorner.dist_ += hitDistMin;
					rayCorner.refNormal_ = hitNormal;
					//*****Valid judge
					refCoordsPtr[rayCorner.refCount_] = hitPointMin;
					hitIdxPtr[rayCorner.refCount_] = hitIdx;
					rayCorner.refCount_ += 1;

					rayCorner.lastHitIdx_ = hitIdx;
				}

			}
		}

		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* cornerPtr = rayPool.rayTubeCornerArray_.get();
		U32 msize = rayPool.rayCountSqrt_;
		U32 cmsize = msize + 1;
		for (U32 i = 0; i < msize; i++)
		{
			for (U32 j = 0; j < msize; j++)
			{
				/*if (i == 25 && j == 31) {
					int dy = 99;
					dy++;
				}*/
				U32 refCount1 = cornerPtr[i * cmsize + j].refCount_;
				U32 refCount2 = cornerPtr[i * cmsize + j + 1].refCount_;
				U32 refCount3 = cornerPtr[(i + 1) * cmsize + j + 1].refCount_;
				U32 refCount4 = cornerPtr[(i + 1) * cmsize + j].refCount_;
				if (((refCount1 & refCount2 & refCount3 & refCount4) == refCount1)
					&& ((refCount1 | refCount2 | refCount3 | refCount4) == refCount1))
				{
					if (refCount1 == 0) {
						continue;
					}
					else
					{

						auto& hitIdxPtr1 = cornerPtr[i * cmsize + j].hitIdx_;
						auto& hitIdxPtr2 = cornerPtr[i * cmsize + j + 1].hitIdx_;
						auto& hitIdxPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].hitIdx_;
						auto& hitIdxPtr4 = cornerPtr[(i + 1) * cmsize + j].hitIdx_;

						/*LUV::Vec3< T >* coordsPtr1 = cornerPtr[i * cmsize + j].refCoords_.get();
						LUV::Vec3< T >* coordsPtr2 = cornerPtr[i * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr4 = cornerPtr[(i + 1) * cmsize + j].refCoords_.get();*/

						for (U32 n = 0; n < refCount1; n++)
						{
							if (((hitIdxPtr1[n] & hitIdxPtr2[n] & hitIdxPtr3[n] & hitIdxPtr4[n]) == hitIdxPtr1[n])
								&& ((hitIdxPtr1[n] | hitIdxPtr2[n] | hitIdxPtr3[n] | hitIdxPtr4[n]) == hitIdxPtr1[n])) {

								rayPtr[i * msize + j].refCount_ = refCount1;
							}
							else
							{
								rayPtr[i * msize + j].refCount_ = 0;
								//T diagDiff = LUV::Length(coordsPtr1[n] - coordsPtr3[n]) + LUV::Length(coordsPtr2[n] - coordsPtr4[n]);
								LUV::Vec3< T > hitNormal1 = bvhPtr[hitIdxPtr1[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal2 = bvhPtr[hitIdxPtr2[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal3 = bvhPtr[hitIdxPtr3[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal4 = bvhPtr[hitIdxPtr4[n]].trig_.GetLoadNormal();
								T normalDiff = LUV::Dot(hitNormal1, hitNormal2) + LUV::Dot(hitNormal2, hitNormal3) + LUV::Dot(hitNormal3, hitNormal4) + LUV::Dot(hitNormal4, hitNormal1);
								normalDiff /= 4;
								if (normalDiff > minDiffNormal)//*******************0.3*Lambda
								{
									rayPtr[i * msize + j].refCount_ = refCount1;
								}
								else
								{
									break;
								}
							}

						}
					}
				}
			}
		}
	}
	void RayValid2Cond(const ReducedBvhArray< T >& bvhArray, RayPool< T >& rayPool, T minDiffNormal, T maxDiffD)
	{
		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		//RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		for (U32 idRay = 0; idRay < rayPool.rayCornerCount_; idRay++)
		{
			RayTubeCorner< T >& rayCorner = rayCornerPtr[idRay];
			LUV::Vec3< T >* refCoordsPtr = rayCorner.refCoords_.get();
			U32* hitIdxPtr = rayCorner.hitIdx_.get();
			bool isHitAtAll = true;

			rayCorner.dist_ = 0;

			while (isHitAtAll && rayCorner.refCount_ < 5)
			{
				isHitAtAll = false;

				U32 stackArray[32];
				I32 stackIdx = 0;
				stackArray[stackIdx] = 0;

				T hitDistMin = 1E32;
				U32 hitIdx = -1;
				LUV::Vec3< T > hitPointMin;

				while (stackIdx >= 0)
				{
					bool isHit = false;
					T hitDist = 0;

					const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

					if (node.status_ == BRANCH || node.status_ == ROOT)
					{
						rayCorner.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
						if (isHit && hitDist < hitDistMin)
						{
							stackArray[stackIdx++] = node.data_.leftChildIdx_;
							stackArray[stackIdx++] = node.data_.rightChildIdx_;
						}
					}
					else if (stackArray[stackIdx] != rayCorner.lastHitIdx_)
					{
						LUV::Vec3< T > hitPoint;
						rayCorner.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
						if (isHit && hitDist < hitDistMin)
						{
							isHitAtAll = true;
							hitDistMin = hitDist;
							hitPointMin = hitPoint;
							hitIdx = stackArray[stackIdx];
						}
					}

					--stackIdx;
				}

				if (isHitAtAll)
				{
					const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
					LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
					//**LUV::Vec3< T > dirCrossNormal = LUV::Cross(rayCorner.dir_, hitNormal);
					LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, rayCorner.dir_);
					LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
					LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(rayCorner.dir_, polU));

					LUV::Vec3< T > refDir = LUV::Unit(rayCorner.dir_ - hitNormal * (2.0 * LUV::Dot(rayCorner.dir_, hitNormal)));

					//**LUV::Vec3< T > refPolU = -polU;
					LUV::Vec3< T > refPolU = polU;
					LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

					T polCompU = LUV::Dot(rayCorner.pol_, polU);
					T polCompR = LUV::Dot(rayCorner.pol_, polR);
					rayCorner.pos_ = hitPointMin;
					rayCorner.dir_ = refDir;
					rayCorner.pol_ = -polCompR * refPolR + polCompU * refPolU;
					rayCorner.dist_ += hitDistMin;
					rayCorner.refNormal_ = hitNormal;
					//*****Valid judge
					refCoordsPtr[rayCorner.refCount_] = hitPointMin;
					hitIdxPtr[rayCorner.refCount_] = hitIdx;
					rayCorner.refCount_ += 1;

					rayCorner.lastHitIdx_ = hitIdx;
				}

			}
		}

		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* cornerPtr = rayPool.rayTubeCornerArray_.get();
		U32 msize = rayPool.rayCountSqrt_;
		U32 cmsize = msize + 1;
		for (U32 i = 0; i < msize; i++)
		{
			for (U32 j = 0; j < msize; j++)
			{
				/*if (i == 25 && j == 31) {
					int dy = 99;
					dy++;
				}*/
				U32 refCount1 = cornerPtr[i * cmsize + j].refCount_;
				U32 refCount2 = cornerPtr[i * cmsize + j + 1].refCount_;
				U32 refCount3 = cornerPtr[(i + 1) * cmsize + j + 1].refCount_;
				U32 refCount4 = cornerPtr[(i + 1) * cmsize + j].refCount_;
				if (((refCount1 & refCount2 & refCount3 & refCount4) == refCount1)
					&& ((refCount1 | refCount2 | refCount3 | refCount4) == refCount1))
				{
					if (refCount1 == 0) {
						continue;
					}
					else
					{

						U32* hitIdxPtr1 = cornerPtr[i * cmsize + j].hitIdx_.get();
						U32* hitIdxPtr2 = cornerPtr[i * cmsize + j + 1].hitIdx_.get();
						U32* hitIdxPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].hitIdx_.get();
						U32* hitIdxPtr4 = cornerPtr[(i + 1) * cmsize + j].hitIdx_.get();

						/*LUV::Vec3< T >* coordsPtr1 = cornerPtr[i * cmsize + j].refCoords_.get();
						LUV::Vec3< T >* coordsPtr2 = cornerPtr[i * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr4 = cornerPtr[(i + 1) * cmsize + j].refCoords_.get();*/

						for (U32 n = 0; n < refCount1; n++)
						{
							if (((hitIdxPtr1[n] & hitIdxPtr2[n] & hitIdxPtr3[n] & hitIdxPtr4[n]) == hitIdxPtr1[n])
								&& ((hitIdxPtr1[n] | hitIdxPtr2[n] | hitIdxPtr3[n] | hitIdxPtr4[n]) == hitIdxPtr1[n])) {

								rayPtr[i * msize + j].refCount_ = refCount1;
							}
							else
							{
								rayPtr[i * msize + j].refCount_ = 0;
								//T diagDiff = LUV::Length(coordsPtr1[n] - coordsPtr3[n]) + LUV::Length(coordsPtr2[n] - coordsPtr4[n]);
								LUV::Vec3< T > hitNormal1 = bvhPtr[hitIdxPtr1[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal2 = bvhPtr[hitIdxPtr2[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal3 = bvhPtr[hitIdxPtr3[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal4 = bvhPtr[hitIdxPtr4[n]].trig_.GetLoadNormal();
								T normalDiff = LUV::Dot(hitNormal1, hitNormal2) + LUV::Dot(hitNormal2, hitNormal3) + LUV::Dot(hitNormal3, hitNormal4) + LUV::Dot(hitNormal4, hitNormal1);
								normalDiff /= 4;
								if ( normalDiff > minDiffNormal )//*******************0.3*Lambda
								{
									rayPtr[i * msize + j].refCount_ = refCount1;
								}
								else
								{
									break;
								}
							}

						}
					}
				}
			}
		}
	}

	void RayValid(const ReducedBvhArray< T >& bvhArray, RayPool< T >& rayPool, T minDiffNormal, T maxDiffD) 
	{
		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		//RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		for (U32 idRay = 0; idRay < rayPool.rayCornerCount_; idRay++)
		{
			RayTubeCorner< T >& rayCorner = rayCornerPtr[idRay];
			LUV::Vec3< T >* refCoordsPtr = rayCorner.refCoords_.get();
			U32* hitIdxPtr = rayCorner.hitIdx_.get();
			bool isHitAtAll = true;

			rayCorner.dist_ = 0;

			while (isHitAtAll && rayCorner.refCount_ < 5)
			{
				isHitAtAll = false;

				U32 stackArray[32];
				I32 stackIdx = 0;
				stackArray[stackIdx] = 0;

				T hitDistMin = 1E32;
				U32 hitIdx = -1;
				LUV::Vec3< T > hitPointMin;

				while (stackIdx >= 0)
				{
					bool isHit = false;
					T hitDist = 0;

					const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

					if (node.status_ == BRANCH || node.status_ == ROOT)
					{
						rayCorner.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
						if (isHit && hitDist < hitDistMin)
						{
							stackArray[stackIdx++] = node.data_.leftChildIdx_;
							stackArray[stackIdx++] = node.data_.rightChildIdx_;
						}
					}
					else if (stackArray[stackIdx] != rayCorner.lastHitIdx_)
					{
						LUV::Vec3< T > hitPoint;
						rayCorner.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
						if (isHit && hitDist < hitDistMin)
						{
							isHitAtAll = true;
							hitDistMin = hitDist;
							hitPointMin = hitPoint;
							hitIdx = stackArray[stackIdx];
						}
					}

					--stackIdx;
				}

				if (isHitAtAll)
				{
					const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
					LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
					//**LUV::Vec3< T > dirCrossNormal = LUV::Cross(rayCorner.dir_, hitNormal);
					LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, rayCorner.dir_);
					LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
					LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(rayCorner.dir_, polU));

					LUV::Vec3< T > refDir = LUV::Unit(rayCorner.dir_ - hitNormal * (2.0 * LUV::Dot(rayCorner.dir_, hitNormal)));

					//**LUV::Vec3< T > refPolU = -polU;
					LUV::Vec3< T > refPolU = polU;
					LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

					T polCompU = LUV::Dot(rayCorner.pol_, polU);
					T polCompR = LUV::Dot(rayCorner.pol_, polR);
					rayCorner.pos_ = hitPointMin;
					rayCorner.dir_ = refDir;
					rayCorner.pol_ = -polCompR * refPolR + polCompU * refPolU;
					rayCorner.dist_ += hitDistMin;
					rayCorner.refNormal_ = hitNormal;
					//*****Valid judge
					refCoordsPtr[rayCorner.refCount_] = hitPointMin;
					hitIdxPtr[rayCorner.refCount_] = hitIdx;
					rayCorner.refCount_ += 1;

					rayCorner.lastHitIdx_ = hitIdx;
				}

			}
		}

		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* cornerPtr = rayPool.rayTubeCornerArray_.get();
		U32 msize = rayPool.rayCountSqrt_;
		U32 cmsize = msize + 1;
		for (U32 i = 0; i < msize; i++)
		{
			for (U32 j = 0; j < msize; j++)
			{
				/*if (i == 25 && j == 31) {
					int dy = 99;
					dy++;
				}*/
				U32 refCount1 = cornerPtr[i * cmsize + j].refCount_;
				U32 refCount2 = cornerPtr[i * cmsize + j + 1].refCount_;
				U32 refCount3 = cornerPtr[(i + 1) * cmsize + j + 1].refCount_;
				U32 refCount4 = cornerPtr[(i + 1) * cmsize + j].refCount_;
				if (((refCount1 & refCount2 & refCount3 & refCount4) == refCount1)
					&& ((refCount1 | refCount2 | refCount3 | refCount4) == refCount1))
				{
					if (refCount1 == 0) {
						continue;
					}
					else
					{

						U32* hitIdxPtr1 = cornerPtr[i * cmsize + j].hitIdx_.get();
						U32* hitIdxPtr2 = cornerPtr[i * cmsize + j + 1].hitIdx_.get();
						U32* hitIdxPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].hitIdx_.get();
						U32* hitIdxPtr4 = cornerPtr[(i + 1) * cmsize + j].hitIdx_.get();

						LUV::Vec3< T >* coordsPtr1 = cornerPtr[i * cmsize + j].refCoords_.get();
						LUV::Vec3< T >* coordsPtr2 = cornerPtr[i * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr4 = cornerPtr[(i + 1) * cmsize + j].refCoords_.get();

						for (U32 n = 0; n < refCount1; n++)
						{
							if (((hitIdxPtr1[n] & hitIdxPtr2[n] & hitIdxPtr3[n] & hitIdxPtr4[n]) == hitIdxPtr1[n])
								&& ((hitIdxPtr1[n] | hitIdxPtr2[n] | hitIdxPtr3[n] | hitIdxPtr4[n]) == hitIdxPtr1[n])) {

								rayPtr[i * msize + j].refCount_ = refCount1;
							}
							else
							{
								rayPtr[i * msize + j].refCount_ = 0;
								T diagDiff = LUV::Length(coordsPtr1[n] - coordsPtr3[n]) + LUV::Length(coordsPtr2[n] - coordsPtr4[n]);
								LUV::Vec3< T > hitNormal1 = bvhPtr[hitIdxPtr1[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal2 = bvhPtr[hitIdxPtr2[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal3 = bvhPtr[hitIdxPtr3[n]].trig_.GetLoadNormal();
								LUV::Vec3< T > hitNormal4 = bvhPtr[hitIdxPtr4[n]].trig_.GetLoadNormal();
								T normalDiff = LUV::Dot(hitNormal1, hitNormal2) + LUV::Dot(hitNormal2, hitNormal3) + LUV::Dot(hitNormal3, hitNormal4) + LUV::Dot(hitNormal4, hitNormal1);
								normalDiff /= 4;
								if ((diagDiff < maxDiffD) && (normalDiff > minDiffNormal))//*******************0.3*Lambda
								{
									rayPtr[i * msize + j].refCount_ = refCount1;
								}
								else
								{
									break;
								}
							}

						}
					}
				}
			}
		}
	}
	void RayValidGpu(const ReducedBvhArray< T >& bvhArray, RayPool< T >& rayPool, T minDiffNormal,T maxDiffD)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		//RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();


		array_view< const ReducedBvhNode< T >, 1 > bvhGpu(bvhArray.nodeCount_, bvhPtr);
		array_view< RayTubeCorner< T >, 1 > rayCornerGpu(rayPool.rayCornerCount_, rayCornerPtr);
		parallel_for_each(rayCornerGpu.extent, [=](index< 1 > idRay) restrict(amp)
			{
				RayTubeCorner& rayCorner = rayCornerGpu[idRay];
				LUV::Vec3< T >* refCoordsPtr = rayCorner.refCoords_.get();
				U32* hitIdxPtr = rayCorner.hitIdx_.get();
				bool isHitAtAll = true;

				rayCorner.dist_ = 0;

				while (isHitAtAll && rayCorner.refCount_ < 5)
				{
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhGpu[index< 1 >(stackArray[stackIdx])];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							rayCorner.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != rayCorner.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							rayCorner.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhGpu[index< 1 >(hitIdx)];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						//**LUV::Vec3< T > dirCrossNormal = LUV::Cross(rayCorner.dir_, hitNormal);
						LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, rayCorner.dir_);
						LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
						LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(rayCorner.dir_, polU));

						LUV::Vec3< T > refDir = LUV::Unit(rayCorner.dir_ - hitNormal * (2.0 * LUV::Dot(rayCorner.dir_, hitNormal)));

						//**LUV::Vec3< T > refPolU = -polU;
						LUV::Vec3< T > refPolU = polU;
						LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

						T polCompU = LUV::Dot(rayCorner.pol_, polU);
						T polCompR = LUV::Dot(rayCorner.pol_, polR);
						rayCorner.pos_ = hitPointMin;
						rayCorner.dir_ = refDir;
						rayCorner.pol_ = -polCompR * refPolR + polCompU * refPolU;
						rayCorner.dist_ += hitDistMin;
						rayCorner.refNormal_ = hitNormal;
						//*****Valid judge
						refCoordsPtr[rayCorner.refCount_] = hitPointMin;
						hitIdxPtr[rayCorner.refCount_] = hitIdx;
						rayCorner.refCount_ += 1;

						rayCorner.lastHitIdx_ = hitIdx;
					}

				}

			});
		rayCornerGpu.synchronize();

		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* cornerPtr = rayPool.rayTubeCornerArray_.get();
		U32 msize = rayPool.rayCountSqrt_;
		U32 cmsize = msize + 1;
		for (U32 i = 0; i < msize; i++)
		{
			for (U32 j = 0; j < msize; j++)
			{
				rayPtr[i * msize + j].refCount_ = 0;
				U32 refCount1 = cornerPtr[i * cmsize + j].refCount_;
				U32 refCount2 = cornerPtr[i * cmsize + j+1].refCount_;
				U32 refCount3 = cornerPtr[(i + 1) * cmsize + j+1].refCount_;
				U32 refCount4 = cornerPtr[(i + 1) * cmsize + j].refCount_;
				if (((refCount1 & refCount2 & refCount3 & refCount4) == refCount1)
					&& ((refCount1 | refCount2 | refCount3 | refCount4) == refCount1))
				{
					if (refCount1 == 0) {
						continue;
					}
					else
					{

						U32* hitIdxPtr1 = cornerPtr[i * cmsize + j].hitIdx_.get();
						U32* hitIdxPtr2 = cornerPtr[i * cmsize + j + 1].hitIdx_.get();
						U32* hitIdxPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].hitIdx_.get();
						U32* hitIdxPtr4 = cornerPtr[(i + 1) * cmsize + j].hitIdx_.get();

						LUV::Vec3< T >* coordsPtr1 = cornerPtr[i * cmsize + j].refCoords_.get();
						LUV::Vec3< T >* coordsPtr2 = cornerPtr[i * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr3 = cornerPtr[(i + 1) * cmsize + j + 1].refCoords_.get();
						LUV::Vec3< T >* coordsPtr4 = cornerPtr[(i + 1) * cmsize + j].refCoords_.get();

						for (U32 n = 0; n < refCount1; n++)
						{
							if (((hitIdxPtr1[n] & hitIdxPtr2[n] & hitIdxPtr3[n] & hitIdxPtr4[n]) == hitIdxPtr1[n])
								&& ((hitIdxPtr1[n] | hitIdxPtr2[n] | hitIdxPtr3[n] | hitIdxPtr4[n]) == hitIdxPtr1[n])) {

								rayPtr[i * msize + j].refCount_ = refCount1;
							}
							else
							{
								rayPtr[i * msize + j].refCount_ = 0;
								T diagDiff = LUV::Length(coordsPtr1[n] - coordsPtr3[n]) + LUV::Length(coordsPtr2[n] - coordsPtr4[n]);
								LUV::Vec3 hitNormal1 = bvhPtr[hitIdxPtr1[n]].trig_.GetLoadNormal();
								LUV::Vec3 hitNormal2 = bvhPtr[hitIdxPtr2[n]].trig_.GetLoadNormal();
								LUV::Vec3 hitNormal3 = bvhPtr[hitIdxPtr3[n]].trig_.GetLoadNormal();
								LUV::Vec3 hitNormal4 = bvhPtr[hitIdxPtr4[n]].trig_.GetLoadNormal();
								T normalDiff = LUV::Dot(hitNormal1, hitNormal2) + LUV::Dot(hitNormal2, hitNormal3) + LUV::Dot(hitNormal3, hitNormal4) + LUV::Dot(hitNormal4, hitNormal1);
								normalDiff /= 4;
								if ((diagDiff < maxDiffD)&&(normalDiff> minDiffNormal))//*******************0.3*Lambda
								{
									rayPtr[i * msize + j].refCount_ = refCount1;
								}
								else
								{
									break;
								}
							}

						}
					}
				}
			}
		}

	}
	void ShootAndBounceRaysAccDieGUI(const ReducedBvhArray< T >& bvhArray, RayPoolDie< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite, const bool& enterInTr, const std::vector< Param< Float > >& param, const std::vector< ParamCoat< Float > >& coatParams)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* stdt = excite.DeriveSeriesArray_.get();
		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		const U32 rayTubeCountSqrt = rayPool.rayCountSqrt_;
		const T rayAreaInit = rayPool.rayArea_;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const T fre = obs.frequency_;
		const T k0 = 2 * pi * fre / c0;
		//const T coatedmaterialD = 0.1;
		const U32 sampleNums = excite.sampleNums_;
		const T deltaT = excite.deltaTime_;

		//std::complex< T > epsilonR = param.epsilon_;
		//std::complex< T > muR = param.mu_;
		int procsNums = omp_get_num_procs();
		//#pragma omp parallel for num_threads(procsNums*2-1)
		int max_threads = omp_get_max_threads();
		#pragma omp parallel for num_threads(procsNums*2-1)
		for (int idRayRow = 0; idRayRow < rayPool.rayCountSqrt_; idRayRow++)
		{
			for (int idRayCol = 0; idRayCol < rayPool.rayCountSqrt_; idRayCol++)
			{
				RayTubeDie< T >& ray = rayPtr[idRayRow * rayPool.rayCountSqrt_ + idRayCol];
				bool isHitAtAll = true;

				ray.dist_ = 0;

				U32 reflectionCount = ray.refCount_;
				ray.refCount_ = 0;
				while (isHitAtAll && ray.refCount_ < 5)
				{
					if (reflectionCount == 0) {
						break;
					}
					U32 meterialType = 1;
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							ray.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != ray.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							ray.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
								meterialType = node.trig_.mtype_;
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						/*if (hitNormal[0]< 0.64427 && hitNormal[0] > 0.64426)
						{
							int dy = 99;
						}*/
						//U32 idx1 = (idRay / rayTubeCountSqrt) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx4 = (idRay / rayTubeCountSqrt + 1) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx2 = idx1 + 1;
						//U32 idx3 = idx4 + 1;
						// __________________ATTENTION______________________
						const RayTubeCorDie< T >& corner1 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol];
						const RayTubeCorDie< T >& corner2 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner3 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner4 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol];

						const LUV::Vec3< T >* hitPointPtr1 = corner1.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr2 = corner2.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr3 = corner3.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr4 = corner4.refCoords_.get();

						LUV::Vec3< T > p1 = hitPointPtr1[ray.refCount_];
						LUV::Vec3< T > p2 = hitPointPtr2[ray.refCount_];
						LUV::Vec3< T > p3 = hitPointPtr3[ray.refCount_];
						LUV::Vec3< T > p4 = hitPointPtr4[ray.refCount_];

						T rayAreaRef = LUV::TriangleArea(p1, p3, p4) + LUV::TriangleArea(p1, p2, p3);
						T DF = std::sqrt(rayAreaInit / rayAreaRef); T val = LUV::Dot(hitNormal, (-ray.dir_));
						T thetaIn = LUV::_Acos(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
						if (rayAreaRef > 3.0 * (rayAreaInit / LUV::_Cos(thetaIn)))
						{
							break;
						}

						std::complex< T > refCoefU;
						std::complex< T > refCoefR;
						LUV::Vec3< std::complex< T > > refPol;

						LUV::Vec3< T > refDir = LUV::Unit(ray.dir_ - hitNormal * (2.0 * LUV::Dot(ray.dir_, hitNormal)));
						
						T judgeV = LUV::Dot(ray.dir_, -hitNormal) - 1;

						if (judgeV < -eps || judgeV > eps)
						{
							// N's direction
							//LUV::Vec3< T > dirCrossNormal = LUV::Cross(ray.dir_, hitNormal);
							LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, ray.dir_);

							LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
							LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(ray.dir_, polU));
							//direction N's Setting
							//LUV::Vec3< T > refPolU = -polU;
							LUV::Vec3< std::complex< T > > refPolU = polU;
							LUV::Vec3< std::complex< T > > refPolR = LUV::CrossX(refDir, refPolU);

							std::complex< T > polCompU = LUV::Dot(ray.pol_, polU);
							std::complex< T > polCompR = LUV::Dot(ray.pol_, polR);

							if (meterialType == 0) {
								//T refCoefU = -1; //R_vertical=-1
								//T refCoefR = 1;//R_horizontal=1
								refCoefU = -1;
								refCoefR = 1;
							}
							else if (meterialType >= 100 && meterialType < 200) {
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								T kx = k0 * LUV::_Sin(thetaIn);
								T k1z = std::sqrtf(powf(k0, 2) - powf(kx, 2));
								std::complex< T > k2z = std::sqrt(powf(k0, 2) * coatParams[meterialType-100].epsilon_ * coatParams[meterialType - 100].mu_ - powf(kx, 2));
								std::complex< T > r12v = (coatParams[meterialType - 100].mu_ * k1z - k2z) / (coatParams[meterialType - 100].mu_ * k1z + k2z);
								std::complex< T > r12p = (coatParams[meterialType - 100].epsilon_ * k1z - k2z) / (coatParams[meterialType - 100].epsilon_ * k1z + k2z);

								std::complex< T > j(0.0, 1.0f);
								T r23v = -1;
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_));
								T r23p = 1;
								refCoefR = (r12p + r23p * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_)) / (1.0f + r12p * r23p * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_));
							}
							else if (meterialType >= 200)
							{
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								std::complex< T > thetaT = LUV::_Asin(LUV::_Sin(thetaIn) / std::sqrt(param[meterialType - 200].epsilon_ * param[meterialType - 200].mu_));
								std::complex< T > eta2 = std::sqrt(param[meterialType - 200].mu_ / param[meterialType - 200].epsilon_);
								refCoefR = (LUV::_Cos(thetaIn) - eta2 * LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaT) + LUV::_Cos(thetaIn));  //---change---eta2 * LUV::_Cos(thetaT) - LUV::_Cos(thetaIn)-->cos(theta_i)-eta2*cos(theta_t)
								refCoefU = (eta2 * LUV::_Cos(thetaIn) - LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaIn) + LUV::_Cos(thetaT));
							}
							else
							{
								refCoefU = -1;
								refCoefR = 1;
							}
							//Before : 
							refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;

						}
						else
						{
							if (meterialType == 0)
							{
								refPol = -ray.pol_;
							}
							else if (meterialType >= 100 && meterialType < 200)
							{
								T k1z = k0;
								std::complex< T > k2z = k0 * std::sqrt(coatParams[meterialType - 100].epsilon_ * coatParams[meterialType - 100].mu_);
								std::complex< T > r12v = (coatParams[meterialType - 100].mu_ * k1z - k2z) / (coatParams[meterialType - 100].mu_ * k1z + k2z);

								T r23v = -1;
								std::complex< T > j(0.0, 1.0f);
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_));
								refPol = refCoefU * ray.pol_;
							}
							else if (meterialType >= 200)
							{
								std::complex< T > oneComplex(1.0, 0.0);
								std::complex< T > eta2 = std::sqrt(param[meterialType - 200].mu_ / param[meterialType - 200].epsilon_);
								refCoefU = (eta2 - oneComplex) / (eta2 + oneComplex);
								refPol = refCoefU * ray.pol_;
							}
						}
						//LUV::Vec3< T > refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						//delay
						if (ray.refCount_ == 0)
						{
							ray.delay_ += LUV::Dot(ray.dir_, hitPointMin) / c0;
							ray.es_.reset(new std::complex< T >[sampleNums], [](std::complex< T >* ptr) { delete[] ptr; });
							std::fill(ray.es_.get(), ray.es_.get() + sampleNums, (T)0.0);
						}
						else
						{
							ray.delay_ += hitDistMin / c0;
						}
						ray.pos_ = hitPointMin;
						ray.lastHitIdx_ = hitIdx;
						//Enter Dieletric or not
						if (meterialType >= 200 && enterInTr)
						{
							Param< T > paramInTr = param[meterialType - 200];
							TraceInDieleAndEs(ray, hitNormal, paramInTr, excite, bvhArray, rayPool, obs, txd, rayAreaRef);
						}
						// Es
						LUV::Vec3< std::complex< T > > hIn = (1 / z0) * LUV::CrossX(ray.dir_, ray.pol_);
						LUV::Vec3< std::complex< T > > hRef = (1 / z0) * LUV::CrossX(refDir, refPol);

						LUV::Vec3< std::complex< T > > eTotal = (ray.pol_ + refPol) * DF;
						LUV::Vec3< std::complex< T > > hTotal = (hIn + hRef) * DF;

						LUV::Vec3< std::complex< T > > EQ = LUV::CrossX(ki,
							LUV::CrossX(ki, LUV::CrossX(hitNormal, hTotal))
							+ LUV::CrossX(hitNormal, eTotal) / z0);
						std::complex< T > EQPol = LUV::Dot(EQ, esPol);
						EQPol *= rayAreaRef;

						ray.dir_ = refDir;
						//ray.pol_ = -polCompR * refPolR + polCompU * refPolU;
						ray.pol_ = refPol;
						ray.dist_ += hitDistMin;
						ray.refNormal_ = hitNormal;
						ray.refCount_ += 1;
						T delay2 = LUV::Dot(ki, hitPointMin);
						delay2 /= c0;
						//tx_d=-2*x_r_min 入射离散点平面的位置距离包围盒的最小距
						U32 nDelay = std::ceil((ray.delay_ + delay2 + txd / c0) / deltaT);
						//U32 nDelay = std::ceil((ray.delay_ + LUV::Dot(ki, hitPointMin) / c0) / deltaT);

						std::complex< T >* es = ray.es_.get();
						//std::vector< std::complex< T >> ecVector;
						for (int kk = 0; kk < sampleNums; kk++)
						{
							int kkn = kk - nDelay;
							if (kkn >= 0 && kkn < sampleNums)
							{
								es[kk] += stdt[kkn] * EQPol;
								//ecVector.push_back(es[kk]);
								/*if (std::isnan(es[kk].real()) || std::isnan(es[kk].imag()))
								{
									int j = kk;
								}*/
							}
						}


					}

				}
			}
		}
	}
	void ShootAndBounceRaysAccDieGUI2Pol(const ReducedBvhArray< T >& bvhArray, RayPoolDie< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite, const bool& enterInTr, const std::vector< Param< Float > >& param, const std::vector< ParamCoat< Float > >& coatParams)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* stdt = excite.DeriveSeriesArray_.get();
		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		const U32 rayTubeCountSqrt = rayPool.rayCountSqrt_;
		const T rayAreaInit = rayPool.rayArea_;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const LUV::Vec3< T > esPolX = obs.polarizationX_;
		const T fre = obs.frequency_;
		const T k0 = 2 * pi * fre / c0;
		//const T coatedmaterialD = 0.1;
		const U32 sampleNums = excite.sampleNums_;
		const T deltaT = excite.deltaTime_;

		//std::complex< T > epsilonR = param.epsilon_;
		//std::complex< T > muR = param.mu_;
		int procsNums = omp_get_num_procs();
		#pragma omp parallel for num_threads(procsNums*2-1)
		for (int idRayRow = 0; idRayRow < rayPool.rayCountSqrt_; idRayRow++)
		{
			for (int idRayCol = 0; idRayCol < rayPool.rayCountSqrt_; idRayCol++)
			{
				RayTubeDie< T >& ray = rayPtr[idRayRow * rayPool.rayCountSqrt_ + idRayCol];
				bool isHitAtAll = true;

				ray.dist_ = 0;

				U32 reflectionCount = ray.refCount_;
				ray.refCount_ = 0;
				while (isHitAtAll && ray.refCount_ < 5)
				{
					if (reflectionCount == 0) {
						break;
					}
					U32 meterialType = 1;
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							ray.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != ray.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							ray.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
								meterialType = node.trig_.mtype_;
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						/*if (hitNormal[0]< 0.64427 && hitNormal[0] > 0.64426)
						{
							int dy = 99;
						}*/
						//U32 idx1 = (idRay / rayTubeCountSqrt) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx4 = (idRay / rayTubeCountSqrt + 1) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx2 = idx1 + 1;
						//U32 idx3 = idx4 + 1;
						// __________________ATTENTION______________________
						const RayTubeCorDie< T >& corner1 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol];
						const RayTubeCorDie< T >& corner2 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner3 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner4 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol];

						const LUV::Vec3< T >* hitPointPtr1 = corner1.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr2 = corner2.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr3 = corner3.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr4 = corner4.refCoords_.get();

						LUV::Vec3< T > p1 = hitPointPtr1[ray.refCount_];
						LUV::Vec3< T > p2 = hitPointPtr2[ray.refCount_];
						LUV::Vec3< T > p3 = hitPointPtr3[ray.refCount_];
						LUV::Vec3< T > p4 = hitPointPtr4[ray.refCount_];

						T rayAreaRef = LUV::TriangleArea(p1, p3, p4) + LUV::TriangleArea(p1, p2, p3);
						T DF = std::sqrt(rayAreaInit / rayAreaRef); T val = LUV::Dot(hitNormal, (-ray.dir_));
						T thetaIn = LUV::_Acos(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
						if (rayAreaRef > 3.0 * (rayAreaInit / LUV::_Cos(thetaIn)))
						{
							break;
						}

						std::complex< T > refCoefU;
						std::complex< T > refCoefR;
						LUV::Vec3< std::complex< T > > refPol;

						LUV::Vec3< T > refDir = LUV::Unit(ray.dir_ - hitNormal * (2.0 * LUV::Dot(ray.dir_, hitNormal)));
						
						T judgeV = LUV::Dot(ray.dir_, -hitNormal) - 1;

						if (judgeV < -eps || judgeV > eps)
						{
							// N's direction
							//LUV::Vec3< T > dirCrossNormal = LUV::Cross(ray.dir_, hitNormal);
							LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, ray.dir_);

							LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
							LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(ray.dir_, polU));
							//direction N's Setting
							//LUV::Vec3< T > refPolU = -polU;
							LUV::Vec3< std::complex< T > > refPolU = polU;
							LUV::Vec3< std::complex< T > > refPolR = LUV::CrossX(refDir, refPolU);

							std::complex< T > polCompU = LUV::Dot(ray.pol_, polU);
							std::complex< T > polCompR = LUV::Dot(ray.pol_, polR);

							if (meterialType == 0) {
								//T refCoefU = -1; //R_vertical=-1
								//T refCoefR = 1;//R_horizontal=1
								refCoefU = -1;
								refCoefR = 1;
							}
							else if (meterialType >= 100 && meterialType < 200) {
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								T kx = k0 * LUV::_Sin(thetaIn);
								T k1z = std::sqrtf(powf(k0, 2) - powf(kx, 2));
								std::complex< T > k2z = std::sqrt(powf(k0, 2) * coatParams[meterialType - 100].epsilon_ * coatParams[meterialType - 100].mu_ - powf(kx, 2));
								std::complex< T > r12v = (coatParams[meterialType - 100].mu_ * k1z - k2z) / (coatParams[meterialType - 100].mu_ * k1z + k2z);
								std::complex< T > r12p = (coatParams[meterialType - 100].epsilon_ * k1z - k2z) / (coatParams[meterialType - 100].epsilon_ * k1z + k2z);//change

								std::complex< T > j(0.0, 1.0f);
								T r23v = -1;
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_));
								T r23p = 1;
								refCoefR = (r12p + r23p * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_)) / (1.0f + r12p * r23p * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_));
							}
							else if (meterialType >= 200)
							{
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								std::complex< T > thetaT = LUV::_Asin(LUV::_Sin(thetaIn) / std::sqrt(param[meterialType - 200].epsilon_ * param[meterialType - 200].mu_));
								std::complex< T > eta2 = std::sqrt(param[meterialType - 200].mu_ / param[meterialType - 200].epsilon_);
								refCoefR = (LUV::_Cos(thetaIn) - eta2 * LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaT) + LUV::_Cos(thetaIn));  //---change---eta2 * LUV::_Cos(thetaT) - LUV::_Cos(thetaIn)-->cos(theta_i)-eta2*cos(theta_t)
								refCoefU = (eta2 * LUV::_Cos(thetaIn) - LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaIn) + LUV::_Cos(thetaT));
							}
							else
							{
								refCoefU = -1;
								refCoefR = 1;
							}
							//Before : 
							refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;

						}
						else
						{
							if (meterialType == 0)
							{
								refPol = -ray.pol_;
							}
							else if (meterialType >= 100 && meterialType < 200)
							{
								T k1z = k0;
								std::complex< T > k2z = k0 * std::sqrt(coatParams[meterialType - 100].epsilon_ * coatParams[meterialType - 100].mu_);
								std::complex< T > r12v = (coatParams[meterialType - 100].mu_ * k1z - k2z) / (coatParams[meterialType - 100].mu_ * k1z + k2z);

								T r23v = -1;
								std::complex< T > j(0.0, 1.0f);
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatParams[meterialType - 100].coatD_));
								refPol = refCoefU * ray.pol_;
							}
							else if (meterialType >= 200)
							{
								std::complex< T > oneComplex(1.0, 0.0);
								std::complex< T > eta2 = std::sqrt(param[meterialType - 200].mu_ / param[meterialType - 200].epsilon_);
								refCoefU = (eta2 - oneComplex) / (eta2 + oneComplex);
								refPol = refCoefU * ray.pol_;
							}
						}
						//LUV::Vec3< T > refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						//delay
						if (ray.refCount_ == 0)
						{
							ray.delay_ += LUV::Dot(ray.dir_, hitPointMin) / c0;
							ray.es_.reset(new std::complex< T >[sampleNums * 2], [](std::complex< T >* ptr) { delete[] ptr; });
							std::fill(ray.es_.get(), ray.es_.get() + sampleNums * 2, (T)0.0);
						}
						else
						{
							ray.delay_ += hitDistMin / c0;
						}
						ray.pos_ = hitPointMin;
						ray.lastHitIdx_ = hitIdx;
						//Enter Dieletric or not
						if (meterialType >= 200 && enterInTr)
						{
							Param< T > paramInTr = param[meterialType - 200];
							TraceInDieleAndEs(ray, hitNormal, paramInTr, excite, bvhArray, rayPool, obs, txd, rayAreaRef);
						}
						// Es
						LUV::Vec3< std::complex< T > > hIn = (1 / z0) * LUV::CrossX(ray.dir_, ray.pol_);
						LUV::Vec3< std::complex< T > > hRef = (1 / z0) * LUV::CrossX(refDir, refPol);

						LUV::Vec3< std::complex< T > > eTotal = (ray.pol_ + refPol) * DF;
						LUV::Vec3< std::complex< T > > hTotal = (hIn + hRef) * DF;

						LUV::Vec3< std::complex< T > > EQ = LUV::CrossX(ki,
							LUV::CrossX(ki, LUV::CrossX(hitNormal, hTotal))
							+ LUV::CrossX(hitNormal, eTotal) / z0);
						std::complex< T > EQPol = LUV::Dot(EQ, esPol);
						std::complex< T > EQPolX = LUV::Dot(EQ, esPolX);
						EQPol *= rayAreaRef;
						EQPolX *= rayAreaRef;

						ray.dir_ = refDir;
						//ray.pol_ = -polCompR * refPolR + polCompU * refPolU;
						ray.pol_ = refPol;
						ray.dist_ += hitDistMin;
						ray.refNormal_ = hitNormal;
						ray.refCount_ += 1;
						T delay2 = LUV::Dot(ki, hitPointMin);
						delay2 /= c0;
						//tx_d=-2*x_r_min 入射离散点平面的位置距离包围盒的最小距
						U32 nDelay = std::ceil((ray.delay_ + delay2 + txd / c0) / deltaT);
						//U32 nDelay = std::ceil((ray.delay_ + LUV::Dot(ki, hitPointMin) / c0) / deltaT);

						std::complex< T >* es = ray.es_.get();
						//std::vector< std::complex< T >> ecVector;
						for (int kk = 0; kk < sampleNums; kk++)
						{
							int kkn = kk - nDelay;
							if (kkn >= 0 && kkn < sampleNums)
							{
								es[kk] += stdt[kkn] * EQPol;
								es[kk + sampleNums] += stdt[kkn] * EQPolX;
								//ecVector.push_back(es[kk]);
								/*if (std::isnan(es[kk].real()) || std::isnan(es[kk].imag()))
								{
									int j = kk;
								}*/
							}
						}


					}

				}
			}
		}
	}

	void ShootAndBounceRaysAccDieMemoryLess(const ReducedBvhArray< T >& bvhArray, RayPoolDie< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite, const bool& enterInTr, const Param< T >& param)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* stdt = excite.DeriveSeriesArray_.get();
		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		const U32 rayTubeCountSqrt = rayPool.rayCountSqrt_;
		const T rayAreaInit = rayPool.rayArea_;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const T fre = obs.frequency_;
		const T k0 = 2 * pi * fre / c0;
		const T coatedmaterialD = 0.001;
		const U32 sampleNums = excite.sampleNums_;
		const T deltaT = excite.deltaTime_;

		std::complex< T > epsilonR = param.epsilon_;
		std::complex< T > muR = param.mu_;

		int max_threads = omp_get_max_threads();
		int procsNums = omp_get_num_procs();
		int m_num_thread = max_threads;// procsNums * 2 - 1;
		#pragma omp parallel for num_threads(m_num_thread)
		for (int idRayRow = 0; idRayRow < rayPool.rayCountSqrt_; idRayRow++)
		{
			for (int idRayCol = 0; idRayCol < rayPool.rayCountSqrt_; idRayCol++)
			{
				RayTubeDie< T >& ray = rayPtr[idRayRow * rayPool.rayCountSqrt_ + idRayCol];
				bool isHitAtAll = true;

				ray.dist_ = 0;

				U32 reflectionCount = ray.refCount_;
				ray.refCount_ = 0;
				while (isHitAtAll && ray.refCount_ < 5)
				{
					if (reflectionCount == 0) {
						break;
					}
					U32 meterialType = 1;
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							ray.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != ray.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							ray.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
								meterialType = node.trig_.mtype_;
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						/*if (hitNormal[0]< 0.64427 && hitNormal[0] > 0.64426)
						{
							int dy = 99;
						}*/
						//U32 idx1 = (idRay / rayTubeCountSqrt) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx4 = (idRay / rayTubeCountSqrt + 1) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx2 = idx1 + 1;
						//U32 idx3 = idx4 + 1;
						// __________________ATTENTION______________________
						const RayTubeCorDie< T >& corner1 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol];
						const RayTubeCorDie< T >& corner2 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner3 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner4 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol];

						/*const LUV::Vec3< T >* hitPointPtr1 = corner1.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr2 = corner2.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr3 = corner3.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr4 = corner4.refCoords_.get();*/

						LUV::Vec3< T > p1 = corner1.refCoords_[ray.refCount_];
						LUV::Vec3< T > p2 = corner2.refCoords_[ray.refCount_];
						LUV::Vec3< T > p3 = corner3.refCoords_[ray.refCount_];
						LUV::Vec3< T > p4 = corner4.refCoords_[ray.refCount_];

						T rayAreaRef = LUV::TriangleArea(p1, p3, p4) + LUV::TriangleArea(p1, p2, p3);
						T DF = std::sqrt(rayAreaInit / rayAreaRef);
						T val = LUV::Dot(hitNormal, (-ray.dir_));
						T thetaIn = LUV::_Acos(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
						if (rayAreaRef > 3.0 * (rayAreaInit / LUV::_Cos(thetaIn)))
						{
							break;
						}
						std::complex< T > refCoefU;
						std::complex< T > refCoefR;
						LUV::Vec3< std::complex< T > > refPol;

						LUV::Vec3< T > refDir = LUV::Unit(ray.dir_ - hitNormal * (2.0 * LUV::Dot(ray.dir_, hitNormal)));

						T judgeV = LUV::Dot(ray.dir_, -hitNormal) - 1;

						if (judgeV < -eps || judgeV > eps)
						{
							// N's direction
							//LUV::Vec3< T > dirCrossNormal = LUV::Cross(ray.dir_, hitNormal);
							LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, ray.dir_);

							LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
							LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(ray.dir_, polU));
							//direction N's Setting
							//LUV::Vec3< T > refPolU = -polU;
							LUV::Vec3< std::complex< T > > refPolU = polU;
							LUV::Vec3< std::complex< T > > refPolR = LUV::CrossX(refDir, refPolU);

							std::complex< T > polCompU = LUV::Dot(ray.pol_, polU);
							std::complex< T > polCompR = LUV::Dot(ray.pol_, polR);

							if (meterialType == 1) {
								//T refCoefU = -1; //R_vertical=-1
								//T refCoefR = 1;//R_horizontal=1
								refCoefU = -1;
								refCoefR = 1;
							}
							else if (meterialType == 2) {
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								T kx = k0 * LUV::_Sin(thetaIn);
								T k1z = std::sqrtf(powf(k0, 2) - powf(kx, 2));
								std::complex< T > k2z = std::sqrt(powf(k0, 2) * epsilonR * muR - powf(kx, 2));
								std::complex< T > r12v = (muR * k1z - k2z) / (muR * k1z + k2z);
								std::complex< T > r12p = (epsilonR * k1z - k2z) / (epsilonR * k1z + k2z);//---change--- R12_p=(epsilon_r*k1z-k2z)/(k2z+epsilon_r*k1z);

								std::complex< T > j(0.0, 1.0f);
								T r23v = -1;
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatedmaterialD)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatedmaterialD));//---change-- j->-j
								T r23p = 1;
								refCoefR = (r12p + r23p * exp(-j * 2.0f * k2z * coatedmaterialD)) / (1.0f + r12p * r23p * exp(-j * 2.0f * k2z * coatedmaterialD));//---change-- j->-j
							}
							else if (meterialType == 3)
							{
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								std::complex< T > thetaT = LUV::_Asin(LUV::_Sin(thetaIn) / std::sqrt(epsilonR * muR));
								std::complex< T > eta2 = std::sqrt(muR / epsilonR);
								refCoefR = (LUV::_Cos(thetaIn) - eta2 * LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaT) + LUV::_Cos(thetaIn));  //---change---eta2 * LUV::_Cos(thetaT) - LUV::_Cos(thetaIn)-->cos(theta_i)-eta2*cos(theta_t)
								refCoefU = (eta2 * LUV::_Cos(thetaIn) - LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaIn) + LUV::_Cos(thetaT));
							}
							else
							{
								refCoefU = -1;
								refCoefR = 1;
							}
							//Before : 
							refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;

						}
						else
						{
							if (meterialType == 1)
							{
								refPol = -ray.pol_;
							}
							else if (meterialType == 2)
							{
								T k1z = k0;
								std::complex< T > k2z = k0 * std::sqrt(epsilonR * muR);
								std::complex< T > r12v = (muR * k1z - k2z) / (muR * k1z + k2z);

								T r23v = -1;
								std::complex< T > j(0.0, 1.0f);
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatedmaterialD)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatedmaterialD));//---change--- j->-j
								refPol = refCoefU * ray.pol_;
							}
							else if (meterialType == 3)
							{
								std::complex< T > oneComplex(1.0, 0.0);
								std::complex< T > eta2 = std::sqrt(muR / epsilonR);
								refCoefU = (eta2 - oneComplex) / (eta2 + oneComplex);
								refPol = refCoefU * ray.pol_;
							}
						}
						//LUV::Vec3< T > refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						//delay
						if (ray.refCount_ == 0)
						{
							ray.delay_ += LUV::Dot(ray.dir_, hitPointMin) / c0;
							/*ray.es_.reset(new std::complex< T >[sampleNums], [](std::complex< T >* ptr) { delete[] ptr; });
							std::fill(ray.es_.get(), ray.es_.get() + sampleNums, (T)0.0);*/
						}
						else
						{
							ray.delay_ += hitDistMin / c0;
						}
						ray.pos_ = hitPointMin;
						ray.lastHitIdx_ = hitIdx;
						//Enter Dieletric or not
						if (meterialType == 3 && enterInTr)
						{
							TraceInDieleAndEs(ray, hitNormal, param, excite, bvhArray, rayPool, obs, txd, rayAreaRef);
						}
						// Es
						LUV::Vec3< std::complex< T > > hIn = (1 / z0) * LUV::CrossX(ray.dir_, ray.pol_);
						LUV::Vec3< std::complex< T > > hRef = (1 / z0) * LUV::CrossX(refDir, refPol);

						LUV::Vec3< std::complex< T > > eTotal = (ray.pol_ + refPol) * DF;
						LUV::Vec3< std::complex< T > > hTotal = (hIn + hRef) * DF;

						LUV::Vec3< std::complex< T > > EQ = LUV::CrossX(ki,
							LUV::CrossX(ki, LUV::CrossX(hitNormal, hTotal))
							+ LUV::CrossX(hitNormal, eTotal) / z0);
						std::complex< T > EQPol = LUV::Dot(EQ, esPol);
						EQPol *= rayAreaRef;

						ray.dir_ = refDir;
						//ray.pol_ = -polCompR * refPolR + polCompU * refPolU;
						ray.pol_ = refPol;
						ray.dist_ += hitDistMin;
						ray.refNormal_ = hitNormal;
						ray.refCount_ += 1;
						T delay2 = LUV::Dot(ki, hitPointMin);
						delay2 /= c0;

						U32 nDelay = std::ceil((ray.delay_ + delay2 + txd / c0) / deltaT);

						ray.nDelay_[ray.refCount_ - 1] = nDelay;
						ray.eqPol_[ray.refCount_ - 1] = EQPol;
					}

				}
			}
		}
		std::cout << "### Shoot And Bounce Return###\n";
	}


	void ShootAndBounceRaysAccDie(const ReducedBvhArray< T >& bvhArray, RayPoolDie< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite, const bool& enterInTr, const Param< T >& param)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorDie< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* stdt = excite.DeriveSeriesArray_.get();
		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		const U32 rayTubeCountSqrt = rayPool.rayCountSqrt_;
		const T rayAreaInit = rayPool.rayArea_;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const T fre = obs.frequency_;
		const T k0 = 2 * pi * fre / c0;
		const T coatedmaterialD = 0.001;
		const U32 sampleNums = excite.sampleNums_;
		const T deltaT = excite.deltaTime_;

		std::complex< T > epsilonR = param.epsilon_;
		std::complex< T > muR = param.mu_;

		int max_threads = omp_get_max_threads();
		int procsNums = omp_get_num_procs();
		int m_num_thread = max_threads;// procsNums * 2 - 1;
		#pragma omp parallel for num_threads(m_num_thread)
		for (int idRayRow = 0; idRayRow < rayPool.rayCountSqrt_; idRayRow++)
		{
			for (int idRayCol = 0; idRayCol < rayPool.rayCountSqrt_; idRayCol++)
			{
				RayTubeDie< T >& ray = rayPtr[idRayRow * rayPool.rayCountSqrt_ + idRayCol];
				bool isHitAtAll = true;

				ray.dist_ = 0;

				U32 reflectionCount = ray.refCount_;
				ray.refCount_ = 0;
				while (isHitAtAll && ray.refCount_ < 5)
				{
					if (reflectionCount == 0) {
						break;
					}
					U32 meterialType = 1;
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							ray.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != ray.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							ray.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
								meterialType = node.trig_.mtype_;
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						/*if (hitNormal[0]< 0.64427 && hitNormal[0] > 0.64426)
						{
							int dy = 99;
						}*/
						//U32 idx1 = (idRay / rayTubeCountSqrt) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx4 = (idRay / rayTubeCountSqrt + 1) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx2 = idx1 + 1;
						//U32 idx3 = idx4 + 1;
						// __________________ATTENTION______________________
						const RayTubeCorDie< T >& corner1 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol];
						const RayTubeCorDie< T >& corner2 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner3 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorDie< T >& corner4 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol];

						/*const LUV::Vec3< T >* hitPointPtr1 = corner1.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr2 = corner2.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr3 = corner3.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr4 = corner4.refCoords_.get();*/

						LUV::Vec3< T > p1 = corner1.refCoords_[ray.refCount_];
						LUV::Vec3< T > p2 = corner2.refCoords_[ray.refCount_];
						LUV::Vec3< T > p3 = corner3.refCoords_[ray.refCount_];
						LUV::Vec3< T > p4 = corner4.refCoords_[ray.refCount_];

						T rayAreaRef = LUV::TriangleArea(p1, p3, p4) + LUV::TriangleArea(p1, p2, p3);
						T DF = std::sqrt(rayAreaInit / rayAreaRef);
						T val = LUV::Dot(hitNormal, (-ray.dir_));
						T thetaIn = LUV::_Acos(val < -1.0 ? -1.0 : (val > 1.0 ? 1.0 : val));
						if (rayAreaRef > 3.0 * (rayAreaInit / LUV::_Cos(thetaIn)))
						{
							break;
						}
						std::complex< T > refCoefU;
						std::complex< T > refCoefR;
						LUV::Vec3< std::complex< T > > refPol;

						LUV::Vec3< T > refDir = LUV::Unit(ray.dir_ - hitNormal * (2.0 * LUV::Dot(ray.dir_, hitNormal)));
						
						T judgeV = LUV::Dot(ray.dir_, -hitNormal) - 1;

						if (judgeV < -eps || judgeV > eps)
						{
							// N's direction
							//LUV::Vec3< T > dirCrossNormal = LUV::Cross(ray.dir_, hitNormal);
							LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, ray.dir_);

							LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
							LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(ray.dir_, polU));
							//direction N's Setting
							//LUV::Vec3< T > refPolU = -polU;
							LUV::Vec3< std::complex< T > > refPolU = polU;
							LUV::Vec3< std::complex< T > > refPolR = LUV::CrossX(refDir, refPolU);

							std::complex< T > polCompU = LUV::Dot(ray.pol_, polU);
							std::complex< T > polCompR = LUV::Dot(ray.pol_, polR);

							if (meterialType == 1) {
								//T refCoefU = -1; //R_vertical=-1
								//T refCoefR = 1;//R_horizontal=1
								refCoefU = -1;
								refCoefR = 1;
							}
							else if (meterialType == 2) {
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								T kx = k0 * LUV::_Sin(thetaIn);
								T k1z = std::sqrtf(powf(k0, 2) - powf(kx, 2));
								std::complex< T > k2z = std::sqrt(powf(k0, 2) * epsilonR * muR - powf(kx, 2));
								std::complex< T > r12v = (muR * k1z - k2z) / (muR * k1z + k2z);
								std::complex< T > r12p = (epsilonR * k1z - k2z) / (epsilonR * k1z + k2z);//---change--- R12_p=(epsilon_r*k1z-k2z)/(k2z+epsilon_r*k1z);

								std::complex< T > j(0.0, 1.0f);
								T r23v = -1;
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatedmaterialD)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatedmaterialD));//---change-- j->-j
								T r23p = 1;
								refCoefR = (r12p + r23p * exp(-j * 2.0f * k2z * coatedmaterialD)) / (1.0f + r12p * r23p * exp(-j * 2.0f * k2z * coatedmaterialD));//---change-- j->-j
							}
							else if (meterialType == 3)
							{
								//T thetaIn = LUV::_Acos(LUV::Dot(hitNormal, -ray.dir_));
								std::complex< T > thetaT = LUV::_Asin(LUV::_Sin(thetaIn) / std::sqrt(epsilonR * muR));
								std::complex< T > eta2 = std::sqrt(muR / epsilonR);
								refCoefR = (LUV::_Cos(thetaIn) - eta2 * LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaT) + LUV::_Cos(thetaIn));  //---change---eta2 * LUV::_Cos(thetaT) - LUV::_Cos(thetaIn)-->cos(theta_i)-eta2*cos(theta_t)
								refCoefU = (eta2 * LUV::_Cos(thetaIn) - LUV::_Cos(thetaT)) / (eta2 * LUV::_Cos(thetaIn) + LUV::_Cos(thetaT));
							}
							else
							{
								refCoefU = -1;
								refCoefR = 1;
							}
							//Before : 
							refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;

						}
						else
						{
							if (meterialType == 1)
							{
								refPol = -ray.pol_;
							}
							else if (meterialType == 2)
							{
								T k1z = k0;
								std::complex< T > k2z = k0 * std::sqrt(epsilonR * muR);
								std::complex< T > r12v = (muR * k1z - k2z) / (muR * k1z + k2z);

								T r23v = -1;
								std::complex< T > j(0.0, 1.0f);
								refCoefU = (r12v + r23v * exp(-j * 2.0f * k2z * coatedmaterialD)) / (1.0f + r12v * r23v * exp(-j * 2.0f * k2z * coatedmaterialD));//---change--- j->-j
								refPol = refCoefU * ray.pol_;
							}
							else if (meterialType == 3)
							{
								std::complex< T > oneComplex(1.0, 0.0);
								std::complex< T > eta2 = std::sqrt(muR / epsilonR);
								refCoefU = (eta2 - oneComplex) / (eta2 + oneComplex);
								refPol = refCoefU * ray.pol_;
							}
						}
						//LUV::Vec3< T > refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						//delay
						if (ray.refCount_ == 0)
						{
							ray.delay_ += LUV::Dot(ray.dir_, hitPointMin) / c0;
							ray.es_.reset(new std::complex< T >[sampleNums], [](std::complex< T >* ptr) { delete[] ptr; });
							std::fill(ray.es_.get(), ray.es_.get() + sampleNums, (T)0.0);
						}
						else
						{
							ray.delay_ += hitDistMin / c0;
						}
						ray.pos_ = hitPointMin;
						ray.lastHitIdx_ = hitIdx;
						//Enter Dieletric or not
						if (meterialType == 3 && enterInTr)
						{
							TraceInDieleAndEs(ray, hitNormal, param, excite, bvhArray, rayPool, obs, txd, rayAreaRef);
						}
						// Es
						LUV::Vec3< std::complex< T > > hIn = (1 / z0) * LUV::CrossX(ray.dir_, ray.pol_);
						LUV::Vec3< std::complex< T > > hRef = (1 / z0) * LUV::CrossX(refDir, refPol);

						LUV::Vec3< std::complex< T > > eTotal = (ray.pol_ + refPol) * DF;
						LUV::Vec3< std::complex< T > > hTotal = (hIn + hRef) * DF;

						LUV::Vec3< std::complex< T > > EQ = LUV::CrossX(ki,
							LUV::CrossX(ki, LUV::CrossX(hitNormal, hTotal))
							+ LUV::CrossX(hitNormal, eTotal) / z0);
						std::complex< T > EQPol = LUV::Dot(EQ, esPol);
						EQPol *= rayAreaRef;

						ray.dir_ = refDir;
						//ray.pol_ = -polCompR * refPolR + polCompU * refPolU;
						ray.pol_ = refPol;
						ray.dist_ += hitDistMin;
						ray.refNormal_ = hitNormal;
						ray.refCount_ += 1;
						T delay2 = LUV::Dot(ki, hitPointMin);
						delay2 /= c0;
						//tx_d=-2*x_r_min 入射离散点平面的位置距离包围盒的最小距
						U32 nDelay = std::ceil((ray.delay_ + delay2 + txd / c0) / deltaT);
						//U32 nDelay = std::ceil((ray.delay_ + LUV::Dot(ki, hitPointMin) / c0) / deltaT);

						std::complex< T >* es = ray.es_.get();
						//std::vector< std::complex< T >> ecVector;
						for (int kk = 0; kk < sampleNums; kk++)
						{
							int kkn = kk - nDelay;
							if (kkn >= 0 && kkn < sampleNums)
							{
								es[kk] += stdt[kkn] * EQPol;
								//ecVector.push_back(es[kk]);
								if (std::isnan(es[kk].real()) || std::isnan(es[kk].imag()))
								{
									int j = kk;
								}
							}
						}


					}

				}
			}
		}
	}
	
	//inline LUV::Vec3< std::complex< T > > CrossWithComplex(const LUV::Vec3< T >& lhs, const LUV::Vec3< std::complex< T > >& rhs)
	//{
	//	return LUV::Vec3< std::complex< T > >(
	//		lhs[1] * rhs[2] - lhs[2] * rhs[1],
	//		lhs[2] * rhs[0] - lhs[0] * rhs[2],
	//		lhs[0] * rhs[1] - lhs[1] * rhs[0]
	//	);
	//}
	void ShootAndBounceRaysAcc(const ReducedBvhArray< T >& bvhArray, RayPool< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* stdt = excite.DeriveSeriesArray_.get();
		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		const U32 rayTubeCountSqrt = rayPool.rayCountSqrt_;
		const T rayAreaInit = rayPool.rayArea_;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const U32 sampleNums = excite.sampleNums_;
		const T deltaT = excite.deltaTime_;

		for (U32 idRayRow = 0; idRayRow < rayPool.rayCountSqrt_; idRayRow++)
		{
			for (U32 idRayCol = 0; idRayCol < rayPool.rayCountSqrt_; idRayCol++)
			{
				RayTube< T >& ray = rayPtr[idRayRow* rayPool.rayCountSqrt_+ idRayCol];
				bool isHitAtAll = true;

				ray.dist_ = 0;

				U32 reflectionCount = ray.refCount_;
				ray.refCount_ = 0;
				while (isHitAtAll && ray.refCount_ < 5)
				{
					if (reflectionCount == 0) {
						break;
					}
					U32 meterialType = 1;
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhPtr[stackArray[stackIdx]];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							ray.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != ray.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							ray.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
								meterialType = node.trig_.mtype_;
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhPtr[hitIdx];
						LUV::Vec3< T > hitNormal = node.trig_.GetLoadNormal();
						// N's direction
						//LUV::Vec3< T > dirCrossNormal = LUV::Cross(ray.dir_, hitNormal);
						LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, ray.dir_);

						LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
						LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(ray.dir_, polU));

						LUV::Vec3< T > refDir = LUV::Unit(ray.dir_ - hitNormal * (2.0 * LUV::Dot(ray.dir_, hitNormal)));
						//direction N's Setting
						//LUV::Vec3< T > refPolU = -polU;
						LUV::Vec3< T > refPolU = polU;
						LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

						T polCompU = LUV::Dot(ray.pol_, polU);
						T polCompR = LUV::Dot(ray.pol_, polR);

						//U32 rH = 1;
						//U32 rV = -1;

						//U32 idx1 = (idRay / rayTubeCountSqrt) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx4 = (idRay / rayTubeCountSqrt + 1) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						//U32 idx2 = idx1 + 1;
						//U32 idx3 = idx4 + 1;
						// __________________ATTENTION______________________
						const RayTubeCorner< T >& corner1 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol];
						const RayTubeCorner< T >& corner2 = rayCornerPtr[idRayRow * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorner< T >& corner3 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol + 1];
						const RayTubeCorner< T >& corner4 = rayCornerPtr[(idRayRow + 1) * (rayPool.rayCountSqrt_ + 1) + idRayCol];

						const LUV::Vec3< T >* hitPointPtr1 = corner1.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr2 = corner2.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr3 = corner3.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr4 = corner4.refCoords_.get();

						LUV::Vec3< T > p1 = hitPointPtr1[ray.refCount_];
						LUV::Vec3< T > p2 = hitPointPtr2[ray.refCount_];
						LUV::Vec3< T > p3 = hitPointPtr3[ray.refCount_];
						LUV::Vec3< T > p4 = hitPointPtr4[ray.refCount_];

						T rayAreaRef = LUV::TriangleArea(p1, p2, p3) + LUV::TriangleArea(p1, p3, p4);
						T DF = std::sqrt(rayAreaInit / rayAreaRef);


						/*if (meterialType == 1) {

						}*/
						T refCoefU = -1; //R_vertical=-1
						T refCoefR = 1;//R_horizontal=1
						LUV::Vec3< T > refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						//delay
						if (ray.refCount_ == 0)
						{
							ray.delay_ += LUV::Dot(ray.dir_, hitPointMin) / c0;
							ray.es_.reset(new T[sampleNums], [](T* ptr) { delete[] ptr; });
							std::fill(ray.es_.get(), ray.es_.get() + sampleNums, (T)0.0);
						}
						else
						{
							ray.delay_ += hitDistMin / c0;
						}
						// Es
						LUV::Vec3< T > hIn = (1 / z0) * LUV::Cross(ray.dir_, ray.pol_);
						LUV::Vec3< T > hRef = (1 / z0) * LUV::Cross(refDir, refPol);

						LUV::Vec3< T > eTotal = (ray.pol_ + refPol) * DF;
						LUV::Vec3< T > hTotal = (hIn + hRef) * DF;

						LUV::Vec3< T > EQ = LUV::Cross(ki,
							LUV::Cross(ki, LUV::Cross(hitNormal, hTotal))
							+ LUV::Cross(hitNormal, eTotal) / z0);
						T EQPol = LUV::Dot(EQ, esPol);
						EQPol *= rayAreaRef;

						ray.pos_ = hitPointMin;
						ray.dir_ = refDir;
						//ray.pol_ = -polCompR * refPolR + polCompU * refPolU;
						ray.pol_ = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						ray.dist_ += hitDistMin;
						ray.refNormal_ = hitNormal;
						ray.refCount_ += 1;
						ray.lastHitIdx_ = hitIdx;
						T delay2 = LUV::Dot(ki, hitPointMin);
						delay2 /= c0;
						//tx_d=-2*x_r_min 入射离散点平面的位置距离包围盒的最小距
						U32 nDelay = std::ceil((ray.delay_ + delay2 + txd / c0) / deltaT);
						//U32 nDelay = std::ceil((ray.delay_ + LUV::Dot(ki, hitPointMin) / c0) / deltaT);

						T* es = ray.es_.get();
						for (int kk = 0; kk < sampleNums; kk++)
						{
							int kkn = kk - nDelay;
							if (kkn >= 0 && kkn < sampleNums)
							{
								es[kk] += stdt[kkn] * EQPol;
							}
						}


					}

				}
			}
		}
	}

	void ShootAndBounceRaysGpuAcc(const ReducedBvhArray< T >& bvhArray, RayPool< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite)
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* stdt = excite.DeriveSeriesArray_.get();

		array_view< const ReducedBvhNode< T >, 1 > bvhGpu(bvhArray.nodeCount_, bvhPtr);
		array_view< RayTube< T >, 1 > rayGpu(rayPool.rayCount_, rayPtr);
		array_view< const RayTubeCorner< T >, 1> rayCornerGpu(rayPool.rayCornerCount_, rayCornerPtr);
		const U32 rayTubeCountSqrt = rayPool.rayCountSqrt_;
		const T rayAreaInit = rayPool.rayArea_;
		const T ki = obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const U32 sampleNums = excite.sampleNums_;
		const T deltaT = excite.deltaTime_;

		parallel_for_each(rayGpu.extent, [=](index< 1 > idRay) restrict(amp)
			{
				RayTube< T >& ray = rayGpu[idRay];
				bool isHitAtAll = true;

				ray.dist_ = 0;
				
				U32 reflectionCount = ray.refCount_;
				ray.refCount_ = 0;
				while (isHitAtAll && ray.refCount_ < 10)
				{
					if (reflectionCount == 0) {
						break;
					}
					U32 meterialType = 1;
					isHitAtAll = false;

					U32 stackArray[32];
					I32 stackIdx = 0;
					stackArray[stackIdx] = 0;

					T hitDistMin = 1E32;
					U32 hitIdx = -1;
					LUV::Vec3< T > hitPointMin;

					while (stackIdx >= 0)
					{
						bool isHit = false;
						T hitDist = 0;

						const ReducedBvhNode< T >& node = bvhGpu[index< 1 >(stackArray[stackIdx])];

						if (node.status_ == BRANCH || node.status_ == ROOT)
						{
							ray.CollisionWithBoundBox(node.data_.boundBox_, isHit, hitDist);
							if (isHit && hitDist < hitDistMin)
							{
								stackArray[stackIdx++] = node.data_.leftChildIdx_;
								stackArray[stackIdx++] = node.data_.rightChildIdx_;
							}
						}
						else if (stackArray[stackIdx] != ray.lastHitIdx_)
						{
							LUV::Vec3< T > hitPoint;
							ray.CollisionWithTriangleSbr(node.trig_, isHit, hitDist, hitPoint);
							if (isHit && hitDist < hitDistMin)
							{
								isHitAtAll = true;
								hitDistMin = hitDist;
								hitPointMin = hitPoint;
								hitIdx = stackArray[stackIdx];
								meterialType = node.trig_.mtype_;
							}
						}

						--stackIdx;
					}

					if (isHitAtAll)
					{
						const ReducedBvhNode< T >& node = bvhGpu[index< 1 >(hitIdx)];
						LUV::Vec3< T > hitNormal = node.trig_.GetNormal();
						// N's direction
						//LUV::Vec3< T > dirCrossNormal = LUV::Cross(ray.dir_, hitNormal);
						LUV::Vec3< T > dirCrossNormal = LUV::Cross(hitNormal, ray.dir_);

						LUV::Vec3< T > polU = LUV::Unit(dirCrossNormal);
						LUV::Vec3< T > polR = LUV::Unit(LUV::Cross(ray.dir_, polU));

						LUV::Vec3< T > refDir = LUV::Unit(ray.dir_ - hitNormal * (2.0 * LUV::Dot(ray.dir_, hitNormal)));
						//direction N's Setting
						//LUV::Vec3< T > refPolU = -polU;
						LUV::Vec3< T > refPolU = polU;
						LUV::Vec3< T > refPolR = LUV::Cross(refDir, refPolU);

						T polCompU = LUV::Dot(ray.pol_, polU);
						T polCompR = LUV::Dot(ray.pol_, polR);

						U32 rH = 1;
						U32 rV = -1;

						U32 idx1 = (idRay / rayTubeCountSqrt) * (rayTubeCountSqrt+1)+(idRay % rayTubeCountSqrt);
						U32 idx4 = (idRay / rayTubeCountSqrt + 1) * (rayTubeCountSqrt + 1) + (idRay % rayTubeCountSqrt);
						U32 idx2 = idx1 + 1;
						U32 idx3 = idx4 + 1;
						const RayTubeCorner< T >& corner1 = rayCornerGpu[idx1];
						const RayTubeCorner< T >& corner2 = rayCornerGpu[idx2];
						const RayTubeCorner< T >& corner3 = rayCornerGpu[idx3];
						const RayTubeCorner< T >& corner4 = rayCornerGpu[idx4];

						const LUV::Vec3< T >* hitPointPtr1 = corner1.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr2 = corner2.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr3 = corner3.refCoords_.get();
						const LUV::Vec3< T >* hitPointPtr4 = corner4.refCoords_.get();

						LUV::Vec3< T > p1 = hitPointPtr1[ray.refCount_];
						LUV::Vec3< T > p2 = hitPointPtr2[ray.refCount_];
						LUV::Vec3< T > p3 = hitPointPtr3[ray.refCount_];
						LUV::Vec3< T > p4 = hitPointPtr4[ray.refCount_];

						T rayAreaRef = LUV::TriangleArea(p1, p2, p3) + LUV::TriangleArea(p1, p3, p4);
						T DF = std::sqrt(rayAreaInit / rayAreaRef);


						/*if (meterialType == 1) {

						}*/
						T refCoefU = 1; //R_vertical=-1
						T refCoefR = -1;//R_horizontal=1
						LUV::Vec3< T > refPol = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						//delay
						if (ray.refCount_==0)
						{
							ray.delay_ += LUV::Dot(ray.dir_, hitPointMin) / c0;
						}
						else
						{
							ray.delay_ += hitDistMin / c0;
						}
						// Es
						LUV::Vec3< T > hIn = (1 / z0) * LUV::Cross(ray.dir_, ray.pol_);
						LUV::Vec3< T > hRef = (1 / z0) * LUV::Cross(refDir, refPol);

						LUV::Vec3< T > eTotal = (ray.pol_ + refPol) * DF;
						LUV::Vec3< T > hTotal = (hIn + hRef) * DF;

						T EQ = LUV::Cross(ki,
							LUV::Cross(ki, LUV::Cross(hitNormal, hTotal))
							+ LUV::Cross(hitNormal, eTotal) / z0);
						T EQPol = LUV::Dot(EQ, esPol);
						EQPol *= rayAreaRef;

						ray.pos_ = hitPointMin;
						ray.dir_ = refDir;
						//ray.pol_ = -polCompR * refPolR + polCompU * refPolU;
						ray.pol_ = refCoefR * polCompR * refPolR + refCoefU * polCompU * refPolU;
						ray.dist_ += hitDistMin;
						ray.refNormal_ = hitNormal;
						ray.refCount_ += 1;
						ray.lastHitIdx_ = hitIdx;
						U32 nDelay = std::ceil((ray.delay_ + LUV::Dot(ki, hitPointMin) / z0) / deltaT);
						T* es = ray.es_.get();
						for (int kk = 0; kk < sampleNums; kk++)
						{
							int kkn = kk - nDelay;
							if (kkn >= 0 && kkn < sampleNums)
							{
								es[kk] += stdt[kkn] * EQPol;
							}
						}


					}

				}

			});

		rayGpu.synchronize();
	}

	void TimeFarField(const RayPool< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite)
	{
		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		U32 rayCount = rayPool.rayCount_;
	}
	void ShootAndBounceRaysGpu( const ReducedBvhArray< T >& bvhArray, RayPool< T >& rayPool )
	{
		using namespace concurrency;

		ReducedBvhNode< T >* bvhPtr = bvhArray.bvhNodeArray_.get();
		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		
		array_view< const ReducedBvhNode< T >, 1 > bvhGpu( bvhArray.nodeCount_, bvhPtr );
		array_view< RayTube< T >, 1 > rayGpu( rayPool.rayCount_, rayPtr );

		parallel_for_each( rayGpu.extent, [=]( index< 1 > idRay ) restrict( amp )
		{
			RayTube< T >& ray = rayGpu[ idRay ];
			bool isHitAtAll = true;

			ray.dist_ = 0;

			while( isHitAtAll && ray.refCount_ < 10 )
			{
				isHitAtAll = false;

				U32 stackArray[ 32 ];
				I32 stackIdx = 0;
				stackArray[ stackIdx ] = 0;

				T hitDistMin = 1E32;
				U32 hitIdx = -1;
				LUV::Vec3< T > hitPointMin;

				while( stackIdx >= 0 )
				{
					bool isHit = false;
					T hitDist = 0;

					const ReducedBvhNode< T >& node = bvhGpu[ index< 1 >( stackArray[ stackIdx ] ) ];

					if( node.status_ == BRANCH || node.status_ == ROOT )
					{
						ray.CollisionWithBoundBox( node.data_.boundBox_, isHit, hitDist );
						if( isHit && hitDist < hitDistMin )
						{
							stackArray[ stackIdx++ ] = node.data_.leftChildIdx_;
							stackArray[ stackIdx++ ] = node.data_.rightChildIdx_;
						}
					}
					else if( stackArray[ stackIdx ] != ray.lastHitIdx_ )
					{
						LUV::Vec3< T > hitPoint;
						ray.CollisionWithTriangleSbr( node.trig_, isHit, hitDist, hitPoint );
						if( isHit && hitDist < hitDistMin )
						{
							isHitAtAll = true;
							hitDistMin = hitDist;
							hitPointMin = hitPoint;
							hitIdx = stackArray[ stackIdx ];
						}
					}

					--stackIdx;
				}

				if( isHitAtAll )
				{
					const ReducedBvhNode< T >& node = bvhGpu[ index< 1 >( hitIdx ) ];
					LUV::Vec3< T > hitNormal = node.trig_.GetNormal();
					LUV::Vec3< T > dirCrossNormal = LUV::Cross( ray.dir_, hitNormal );

					LUV::Vec3< T > polU = LUV::Unit( dirCrossNormal );
					LUV::Vec3< T > polR = LUV::Unit( LUV::Cross( ray.dir_, polU ) );

					LUV::Vec3< T > refDir = LUV::Unit( ray.dir_ - hitNormal * ( 2.0 * LUV::Dot( ray.dir_, hitNormal ) ) );

					LUV::Vec3< T > refPolU = -polU;
					LUV::Vec3< T > refPolR = LUV::Cross( refDir, refPolU );

					T polCompU = LUV::Dot( ray.pol_, polU );
					T polCompR = LUV::Dot( ray.pol_, polR );
					ray.pos_ = hitPointMin;
					ray.dir_ = refDir;
					ray.pol_ = - polCompR * refPolR + polCompU * refPolU;
					ray.dist_ += hitDistMin;
					ray.refNormal_ = hitNormal;
					ray.refCount_ += 1;
					ray.lastHitIdx_ = hitIdx;


					//const ReducedBvhNode< T >& node = bvhGpu[ index< 1 >( hitIdx ) ];
					//LUV::Vec3< T > hitNormal = node.trig_.GetNormal();
					//LUV::Vec3< T > dirCrossNormal = LUV::Cross( ray.dir_, hitNormal );
					//LUV::Vec3< T > polU = LUV::Unit( dirCrossNormal );
					//LUV::Vec3< T > polR = LUV::Cross( polU, ray.dir_ );
					//LUV::Vec3< T > refDir = LUV::Unit( ray.dir_ - hitNormal * ( 2.0 * LUV::Dot( ray.dir_, hitNormal ) ) );
					//LUV::Vec3< T > refPolU = polU;
					//LUV::Vec3< T > refPolR = LUV::Cross( refPolU, refDir );
					//T polCompU = LUV::Dot( ray.pol_, polU );
					//T polCompR = LUV::Dot( ray.pol_, polR );
					//ray.pos_ = hitPointMin;
					//ray.dir_ = refDir;
					//ray.pol_ = - polCompR * refPolR + polCompU * refPolU;
					//ray.dist_ += hitDistMin;
					//ray.refNormal_ = hitNormal;
					//ray.refCount_ += 1;
					//ray.lastHitIdx_ = hitIdx;
				}

			}

		});

		rayGpu.synchronize();

	}

	void PhysicalOpticsIntegral( const RayPool< T >& rayPool, const Observation< T >& obs, T& rcs )
	{
		using namespace std;
		using namespace std::complex_literals;

		T freq = obs.frequency_;
		T angFreq = 2 * pi * freq;
		T waveLen = c0 / freq;
		T waveNum = 2 * pi / waveLen;

		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		U32 rayCount = rayPool.rayCount_;
		T rayArea = rayPool.rayArea_;

		LUV::Vec3< T > obsDir = obs.direction_;
		LUV::Vec3< T > obsDirSph = LUV::CtsToSph( obsDir );

		T phi = obsDirSph[ 1 ];
		T the = obsDirSph[ 2 ];

		T cp = cos( phi );
		T sp = sin( phi );
		T ct = cos( the );
		T st = sin( the );

		LUV::Vec3< T > dirX( 1.0, 0.0, 0.0 );
		LUV::Vec3< T > dirY( 0.0, 1.0, 0.0 );
		LUV::Vec3< T > dirZ( 0.0, 0.0, 1.0 );
		LUV::Vec3< T > dirP( -sp, cp, 0.0 );
		LUV::Vec3< T > dirT( cp * ct, sp * ct, -st );

		LUV::Vec3< T > vecK = waveNum * ( ( dirX * cp + dirY * sp ) * st + dirZ * ct );
		
		complex< T > AU = 0;
		complex< T > AR = 0;

		complex< T > i( 0.0, 1.0 );

		for( U32 idRay = 0; idRay < rayCount; ++idRay )
		{
			RayTube< T >& ray = rayPtr[ idRay ];
			if( ray.refCount_ > 0 )
			{
				T kr = waveNum * ray.dist_;
				//T reflectionCoef = pow( -1.0, ray.refCount_ );
				T reflectionCoef = pow( 1.0, ray.refCount_ );

				LUV::Vec3< complex< T > > apE = exp( i * kr ) * ray.pol_ * reflectionCoef;
				LUV::Vec3< complex< T > > apH = -LUV::Cross( apE, ray.dir_ );

				complex< T > BU = LUV::Dot( -( LUV::Cross( apE, -dirP ) + LUV::Cross( apH, dirT ) ), ray.dir_ );
				complex< T > BR = LUV::Dot( -( LUV::Cross( apE, dirT ) + LUV::Cross( apH, dirP ) ), ray.dir_ );

				complex< T > factor = complex< T >( 0.0, ( ( waveNum * rayArea ) / ( 4.0 * pi ) ) ) * exp( -i * LUV::Dot( vecK, ray.pos_ ) );

				AU += BU * factor;
				AR += BR * factor;
			}

		}

		//std::cout << "AU: " << AU.real() << " + i" << AU.imag() << std::endl;
		//std::cout << "AR: " << AR.real() << " + i" << AR.imag() << std::endl;

		rcs = 4.0 * pi * ( pow( abs( AU ), 2 ) + pow( abs( AR ), 2 ) ); // * 4 * pi
		std::cout << obsDir[0] << " " << obsDir[1] << " " << obsDir[2] << " RCS:" << rcs << std::endl;
	}

	void PhysicalOpticsIntegralWithTD(const RayPool< T >& rayPool, const Observation< T >& obs, RcsArray< T >& rcsArray, const Excitation<T>& excite)
	{
		using namespace std;
		using namespace std::complex_literals;

		T freq = obs.frequency_;
		T angFreq = 2 * pi * freq;
		T waveLen = c0 / freq;
		T waveNum = 2 * pi / waveLen;

		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		U32 rayCount = rayPool.rayCount_;
		T rayArea = rayPool.rayArea_;

		T* st = excite.SeriesArray_.get();
		T* st_dt = excite.DeriveSeriesArray_.get();
		T deltaTime = excite.deltaTime;
		T sampleNums = excite.sampleNums;

		std::shared_ptr<T> Es;
		Es.reset(new T[sampleNums], [](T* ptr) { delete[] ptr; });
		T* EsPtr = Es.get();
		std::fill_n(EsPtr, sampleNums, T(0));

		LUV::Vec3< T > obsDir = obs.direction_;
		LUV::Vec3< T > obsDirSph = LUV::CtsToSph(obsDir);

		T phi = obsDirSph[1];
		T the = obsDirSph[2];

		T cp = cos(phi);
		T sp = sin(phi);
		T ct = cos(the);
		T st = sin(the);

		LUV::Vec3< T > dirX(1.0, 0.0, 0.0);
		LUV::Vec3< T > dirY(0.0, 1.0, 0.0);
		LUV::Vec3< T > dirZ(0.0, 0.0, 1.0);
		LUV::Vec3< T > dirP(-sp, cp, 0.0);
		LUV::Vec3< T > dirT(cp * ct, sp * ct, -st);

		LUV::Vec3< T > vecK = waveNum * ((dirX * cp + dirY * sp) * st + dirZ * ct);

		complex< T > AU = 0;
		complex< T > AR = 0;

		complex< T > i(0.0, 1.0);


		for (U32 idRay = 0; idRay < rayCount; ++idRay)
		{
			RayTube< T >& ray = rayPtr[idRay];
			if (ray.refCount_ > 0)
			{
				T kr = waveNum * ray.dist_;
				//T reflectionCoef = pow( -1.0, ray.refCount_ );
				T reflectionCoef = pow(1.0, ray.refCount_);

				LUV::Vec3< complex< T > > apE = exp(i * kr) * ray.pol_ * reflectionCoef;
				LUV::Vec3< complex< T > > apH = -LUV::Cross(apE, ray.dir_);

				complex< T > BU = LUV::Dot(-(LUV::Cross(apE, -dirP) + LUV::Cross(apH, dirT)), ray.dir_);
				complex< T > BR = LUV::Dot(-(LUV::Cross(apE, dirT) + LUV::Cross(apH, dirP)), ray.dir_);

				complex< T > factor = complex< T >(0.0, ((waveNum * rayArea) / (4.0 * pi))) * exp(-i * LUV::Dot(vecK, ray.pos_));

				// AU += BU * factor;
				// AR += BR * factor;
				//rcs = 4.0 * pi * (pow(abs(BU), 2) + pow(abs(BR), 2)); // * 4 * pi

				//tx_d=-2*x_r_min t_delay_total=t_delay1+t_delay2+tx_d;
				T delayTime = ray.dist_ / c0; //make up for [negative]
				U32 delayCnt = (U32)std::ceil(delayTime / deltaTime);

				for (U32 samplePoint_ = 0; samplePoint_ < sampleNums; sampleNums++)
				{
					U32 samplePointValid_ = samplePoint_ - delayCnt;
					if (samplePointValid_ >= 0 && samplePointValid_<sampleNums)
					{
						T esVecWithPol = LUV::Dot((BU * dirP + BR * dirT),dirP);
						EsPtr[samplePoint_] += esVecWithPol * st_dt[samplePointValid_];
					}
				}
			}

		}

		//std::cout << "AU: " << AU.real() << " + i" << AU.imag() << std::endl;
		//std::cout << "AR: " << AR.real() << " + i" << AR.imag() << std::endl;

		rcsArray.rcsArray_.reset(abs(FFT::fft1d(Es.get(), FFT::fft_dir::DIR_FWD)).data(), [](T* ptr) {});

		
		//std::cout << obsDir[0] << " " << obsDir[1] << " " << obsDir[2] << " RCS:" << rcs << std::endl;
	}

	void RcsTimeToFreqDomainDieMemoryLess(TriangleMesh< T >& trigMesh, const Observation< T >& obs, RayPoolDie< T >& rayPool, RcsArray< T >& rcsArray, Excitation< T >& excite)
	{

		T* stdt = excite.DeriveSeriesArray_.get();
		std::shared_ptr< std::complex< T >> mes;
		mes.reset(new std::complex< T >[rcsArray.rcsCount_], [](std::complex< T >* ptr) { delete[] ptr; });
		std::fill(mes.get(), mes.get() + rcsArray.rcsCount_, 0.0);

		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		//trigMesh.FindEdge(txd, excite, obs, mes.get());

		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		//RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();

		std::complex< T >* esPtr = mes.get();
		T* st = excite.SeriesArray_.get();

		int max_threads = omp_get_max_threads();
		int procsNums = omp_get_num_procs();
		int m_num_thread = max_threads;
		//int m_num_thread = procsNums * 2 - 1;

		std::vector<int>start;
		std::vector<int>end;
		std::vector<int>validFlag;
		std::vector<std::shared_ptr<std::complex< T >>> esPart(m_num_thread);
		int splitNums = (rayPool.rayCountSqrt_ + m_num_thread - 1) / m_num_thread;
		int k1 = 0; int k2 = splitNums;
		for (int i = 0; i < m_num_thread; i++)
		{
			start.push_back(k1);
			end.push_back(k2);
			k1 = k2;
			k2 += splitNums;
			if (k2 >= rayPool.rayCountSqrt_)
			{
				k2 = rayPool.rayCountSqrt_;
			}
			validFlag.push_back(0);
			esPart.at(i).reset(new std::complex< T >[rcsArray.rcsCount_], [](std::complex< T >* ptr) { delete[] ptr; });
			std::fill(esPart.at(i).get(), esPart.at(i).get() + rcsArray.rcsCount_, 0.0);
		}
		#pragma omp parallel num_threads(m_num_thread)
		{
			int thread_id = omp_get_thread_num();
			int start_row = start.at(thread_id);
			int end_row = end.at(thread_id);
			/*std::complex< T >* esPart= rayPtr[start_row * rayPool.rayCountSqrt_ + 0].es_.get();*/
			std::complex< T >* esPtr = esPart.at(thread_id).get();
			for (int row = start_row; row < end_row; row++)
			{
				for (int col = 0; col < rayPool.rayCountSqrt_; col++)
				{
					const RayTubeDie< T >& ray = rayPtr[row * rayPool.rayCountSqrt_ + col];
					if (ray.refCount_ == 0) {
						continue;
					}
					else
					{
						U32 sampleNums = rcsArray.rcsCount_;

						for (U32 refIdx = 0; refIdx < ray.refCount_; refIdx++)
						{
							for (U32 i = 0; i < sampleNums; i++)
							{
								int kkn = i - ray.nDelay_[refIdx];
								if (kkn >= 0 && kkn < sampleNums)
								{
									esPtr[i] = esPtr[i] + stdt[kkn] * ray.eqPol_[refIdx];
								}
							}
						}
					}
				}
			}
		}
		for (int k = 0; k < m_num_thread; k++)
		{
			std::complex< T >* esPartPtr = esPart.at(k).get();
			U32 sampleNums = rcsArray.rcsCount_;
			for (U32 i = 0; i < sampleNums; i++)
			{
				esPtr[i] = esPtr[i] + esPartPtr[i];
			}
		}

		U32 transNums = 1;
		while (transNums <= rcsArray.rcsCount_)
		{
			transNums = transNums * 2;
		}
		transNums = transNums / 2;

		std::vector<std::complex< T >> fft_arg;
		std::vector<std::complex< T >> fft_arg_st;
		for (U32 i = 0; i < transNums; i++)
		{
			//fft_arg.push_back(esPtr[i]);
			fft_arg.push_back(std::complex< T >(esPtr[i].real(), 0.0));
			fft_arg_st.push_back(std::complex< T >(st[i], 0.0));
		}
		auto fftEsData = FFT::fft1d(fft_arg, FFT::fft_dir::DIR_FWD);
		auto fftStData = FFT::fft1d(fft_arg_st, FFT::fft_dir::DIR_FWD);

		std::vector< T > vecEs;
		std::vector< T > vecEi;

		for (U32 i = 0; i < transNums; i++)
		{
			vecEs.push_back(std::abs(fftEsData[i]));
			vecEi.push_back(std::abs(fftStData[i]));
		}
		rcsArray.rcsCount_ = transNums;
		rcsArray.rcsArray_.reset(new T[transNums], [](T* ptr) { delete[] ptr; });
		T* rcsPtr = rcsArray.rcsArray_.get();
		std::vector< T > vecRcs;
		T rcsTmp;
		for (U32 i = 0; i < transNums; i++)
		{
			rcsTmp = (pow(z0, 2) / (4 * pi * pow(c0, 2))) * pow(vecEs[i], 2) / pow(vecEi[i], 2);
			rcsPtr[i] = 10 * std::log10f(rcsTmp);
			vecRcs.push_back(rcsTmp / (pi * pow(0.2, 2)));
		}
		//delta_f=fs/n_fft; fs=50*fH; 
		//N_fL = ceil(fL / delta_f);% 计算工作频段的最低频率fL对应的频谱图上横坐标序列号
		//N_fH = ceil(fH / delta_f);% 计算工作频段的最高频率fH对应的频谱图上横坐标序列号
		T deltaFreq = 50 * excite.freqHigh_ / excite.sampleNums_;
		U32 idxFreqL = std::ceil(excite.freqLow_ / deltaFreq);//change
		U32 idxFreqH = std::ceil(excite.freqHigh_ / deltaFreq);
		std::vector< T >vecValidRCS;
		for (U32 i = idxFreqL; i <= idxFreqH; i++)
		{
			vecValidRCS.push_back(vecRcs[i]);
		}

		std::fstream rcsFile("E:\\Files\\cube2.rcs", std::ios::trunc | std::ios::out | std::ios::binary);

		if (!rcsFile.good())
		{
			rcsFile.close();
		}
		int num = idxFreqH - idxFreqL + 1;
		rcsFile.write((char*)(&num), sizeof(U32));
		rcsFile.write((char*)(rcsPtr + idxFreqL), sizeof(T) * num);

		rcsFile.close();
		int dengy = 6;
	}


	void RcsTimeToFreqDomainDie(TriangleMesh< T >& trigMesh, const Observation< T >& obs, RayPoolDie< T >& rayPool, RcsArray< T >& rcsArray, Excitation< T >& excite)
	{

		std::shared_ptr< std::complex< T >> mes;
		mes.reset(new std::complex< T >[rcsArray.rcsCount_], [](std::complex< T >* ptr) { delete[] ptr; });
		std::fill(mes.get(), mes.get() + rcsArray.rcsCount_, 0.0);

		T txd = rayPool.kiDotMinDirN_ < 0 ? (-2.0 * rayPool.kiDotMinDirN_) : 0;
		//trigMesh.FindEdge(txd, excite, obs, mes.get());

		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		//RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		
		std::complex< T >* esPtr = mes.get();
		T* st = excite.SeriesArray_.get();

		int max_threads = omp_get_max_threads();
		int procsNums = omp_get_num_procs();
		int m_num_thread = procsNums * 2 - 1; 
		
		std::vector<int>start;
		std::vector<int>end;
		std::vector<int>validFlag;
		std::vector<std::shared_ptr<std::complex< T >>> esPart(m_num_thread);
		int splitNums = (rayPool.rayCountSqrt_ + m_num_thread - 1) / m_num_thread;
		int k1 = 0; int k2 = splitNums;
		for (int i = 0; i < m_num_thread; i++)
		{
			start.push_back(k1);
			end.push_back(k2);
			k1 = k2;
			k2 += splitNums;
			if (k2 >= rayPool.rayCountSqrt_)
			{
				k2 = rayPool.rayCountSqrt_;
			}
			validFlag.push_back(0);
			esPart.at(i).reset(new std::complex< T >[rcsArray.rcsCount_], [](std::complex< T >* ptr) { delete[] ptr; });
			std::fill(esPart.at(i).get(), esPart.at(i).get() + rcsArray.rcsCount_, 0.0);
		}
		#pragma omp parallel num_threads(m_num_thread)
		{
			int thread_id = omp_get_thread_num();
			int start_row = start.at(thread_id);
			int end_row = end.at(thread_id);
			/*std::complex< T >* esPart= rayPtr[start_row * rayPool.rayCountSqrt_ + 0].es_.get();*/
			std::complex< T >* esPtr= esPart.at(thread_id).get();
			for (int row = start_row ; row < end_row; row++)
			{
				for (int col = 0; col < rayPool.rayCountSqrt_; col++)
				{
					const RayTubeDie< T >& ray = rayPtr[row * rayPool.rayCountSqrt_ + col];
					if (ray.refCount_ == 0) {
						continue;
					}
					else
					{
						std::complex< T >* esRay = ray.es_.get();
						U32 sampleNums = rcsArray.rcsCount_;
						for (U32 i = 0; i < sampleNums; i++)
						{
							esPtr[i] = esPtr[i] + esRay[i];
							/*if (std::isnan(esRay[i].real()) || std::isnan(esRay[i].imag()))
							{
								int j = i;
							}*/
						}
						////debug
						//std::vector< std::complex< T >> ecVector(esRay, esRay + rcsArray.rcsCount_);
						////
					}
				}
			}
		}
		for (int k = 0; k < m_num_thread; k++)
		{
			std::complex< T >* esPartPtr = esPart.at(k).get();
			U32 sampleNums = rcsArray.rcsCount_;
			for (U32 i = 0; i < sampleNums; i++)
			{
				esPtr[i] = esPtr[i] + esPartPtr[i];
			}
		}

		U32 transNums = 1;
		while (transNums <= rcsArray.rcsCount_)
		{
			transNums = transNums * 2;
		}
		transNums = transNums / 2;

		std::vector<std::complex< T >> fft_arg;
		std::vector<std::complex< T >> fft_arg_st;
		for (U32 i = 0; i < transNums; i++)
		{
			//fft_arg.push_back(esPtr[i]);
			fft_arg.push_back(std::complex< T >(esPtr[i].real(), 0.0));
			fft_arg_st.push_back(std::complex< T >(st[i], 0.0));
		}
		auto fftEsData = FFT::fft1d(fft_arg, FFT::fft_dir::DIR_FWD);
		auto fftStData = FFT::fft1d(fft_arg_st, FFT::fft_dir::DIR_FWD);

		std::vector< T > vecEs;
		std::vector< T > vecEi;

		for (U32 i = 0; i < transNums; i++)
		{
			vecEs.push_back(std::abs(fftEsData[i]));
			vecEi.push_back(std::abs(fftStData[i]));
		}
		rcsArray.rcsCount_ = transNums;
		rcsArray.rcsArray_.reset(new T[transNums], [](T* ptr) { delete[] ptr; });
		T* rcsPtr = rcsArray.rcsArray_.get();
		std::vector< T > vecRcs;
		T rcsTmp;
		for (U32 i = 0; i < transNums; i++)
		{
			rcsTmp = (pow(z0, 2) / (4 * pi * pow(c0, 2))) * pow(vecEs[i], 2) / pow(vecEi[i], 2);
			rcsPtr[i] = 10 * std::log10f(rcsTmp);
			vecRcs.push_back(rcsTmp / (pi * pow(0.2, 2)));
		}
		//delta_f=fs/n_fft; fs=50*fH; 
		//N_fL = ceil(fL / delta_f);% 计算工作频段的最低频率fL对应的频谱图上横坐标序列号
		//N_fH = ceil(fH / delta_f);% 计算工作频段的最高频率fH对应的频谱图上横坐标序列号
		T deltaFreq = 50 * excite.freqHigh_ / excite.sampleNums_;
		U32 idxFreqL = std::ceil(excite.freqLow_ / deltaFreq);//change
		U32 idxFreqH = std::ceil(excite.freqHigh_ / deltaFreq);
		std::vector< T >vecValidRCS;
		for (U32 i = idxFreqL; i <= idxFreqH; i++)
		{
			vecValidRCS.push_back(vecRcs[i]);
		}

		std::fstream rcsFile("E:\\Files\\cube2.rcs", std::ios::trunc | std::ios::out | std::ios::binary);

		if (!rcsFile.good())
		{
			rcsFile.close();
		}
		int num = idxFreqH - idxFreqL + 1;
		rcsFile.write((char*)(&num), sizeof(U32));
		rcsFile.write((char*)(rcsPtr + idxFreqL), sizeof(T) * num);

		rcsFile.close();
		int dengy = 6;
	}

	void RcsTimeToFreqDomainDie(TriangleMesh< T >& trigMesh, const Observation< T >& obs, RayPoolDie< T >& rayPool, RcsArray< T >& rcsArray, Excitation< T >& excite,
								int polCnt)
	{
		RayTubeDie< T >* rayPtr = rayPool.rayTubeArray_.get();
		//RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		std::shared_ptr< std::complex< T >> mes;
		std::shared_ptr< std::complex< T >> mesX;
		mes.reset(new std::complex< T >[rcsArray.rcsCount_], [](std::complex< T >* ptr) { delete[] ptr; });
		mesX.reset(new std::complex< T >[rcsArray.rcsCount_], [](std::complex< T >* ptr) { delete[] ptr; });
		std::complex< T >* esPtr = mes.get();
		std::complex< T >* esPtrX = mesX.get();
		T* st = excite.SeriesArray_.get();
		std::fill(esPtr, esPtr + rcsArray.rcsCount_, 0.0);
		std::fill(esPtrX, esPtrX + rcsArray.rcsCount_, 0.0);
		for (U32 row = 0; row < rayPool.rayCountSqrt_; row++)
		{
			for (U32 col = 0; col < rayPool.rayCountSqrt_; col++)
			{
				const RayTubeDie< T >& ray = rayPtr[row * rayPool.rayCountSqrt_ + col];
				if (ray.refCount_ == 0) {
					continue;
				}
				else
				{
					std::complex< T >* esRay = ray.es_.get();
					U32 sampleNums = rcsArray.rcsCount_;
					for (U32 i = 0; i < sampleNums; i++)
					{
						esPtr[i] = esPtr[i] + esRay[i];
						esPtrX[i] = esPtrX[i] + esRay[i + sampleNums];
						/*if (std::isnan(esRay[i].real()) || std::isnan(esRay[i].imag()))
						{
							int j = i;
						}*/
					}
					////debug
					//std::vector< std::complex< T >> ecVector(esRay, esRay + rcsArray.rcsCount_);
					////
				}
			}
		}

		U32 transNums = 1;
		while (transNums <= rcsArray.rcsCount_)
		{
			transNums = transNums * 2;
		}
		transNums = transNums / 2;

		std::vector<std::complex< T >> fft_arg;
		std::vector<std::complex< T >> fft_argX;
		std::vector<std::complex< T >> fft_arg_st;
		for (U32 i = 0; i < transNums; i++)
		{
			//fft_arg.push_back(esPtr[i]);
			fft_arg.push_back(std::complex< T >(esPtr[i].real(), 0.0));
			fft_argX.push_back(std::complex< T >(esPtrX[i].real(), 0.0));
			fft_arg_st.push_back(std::complex< T >(st[i], 0.0));
		}
		auto fftEsData = FFT::fft1d(fft_arg, FFT::fft_dir::DIR_FWD);
		auto fftEsDataX = FFT::fft1d(fft_argX, FFT::fft_dir::DIR_FWD);
		auto fftStData = FFT::fft1d(fft_arg_st, FFT::fft_dir::DIR_FWD);

		std::vector< T > vecEs;
		std::vector< T > vecEsX;
		std::vector< T > vecEi;

		for (U32 i = 0; i < transNums; i++)
		{
			vecEs.push_back(std::abs(fftEsData[i]));
			vecEsX.push_back(std::abs(fftEsDataX[i]));
			vecEi.push_back(std::abs(fftStData[i]));
		}
		rcsArray.rcsCount_ = transNums * 2;
		rcsArray.rcsArray_.reset(new T[transNums * 2], [](T* ptr) { delete[] ptr; });
		T* rcsPtr = rcsArray.rcsArray_.get();
		std::vector< T > vecRcs;
		T rcsTmp;
		T rcsTmpX;
		for (U32 i = 0; i < transNums; i++)
		{
			rcsTmp = (pow(z0, 2) / (4 * pi * pow(c0, 2))) * pow(vecEs[i], 2) / pow(vecEi[i], 2);
			rcsTmpX = (pow(z0, 2) / (4 * pi * pow(c0, 2))) * pow(vecEsX[i], 2) / pow(vecEi[i], 2);
			rcsPtr[i] = 10 * std::log10f(rcsTmp);
			rcsPtr[i + transNums] = 10 * std::log10f(rcsTmpX);
			vecRcs.push_back(rcsTmp / (pi * pow(0.2, 2)));
		}
		//delta_f=fs/n_fft; fs=50*fH; 
		//N_fL = ceil(fL / delta_f);% 计算工作频段的最低频率fL对应的频谱图上横坐标序列号
		//N_fH = ceil(fH / delta_f);% 计算工作频段的最高频率fH对应的频谱图上横坐标序列号
		T deltaFreq = 50 * excite.freqHigh_ / excite.sampleNums_;
		U32 idxFreqL = std::ceil(excite.freqLow_ / deltaFreq);
		U32 idxFreqH = std::ceil(excite.freqHigh_ / deltaFreq);
		std::vector< T >vecValidRCS;
		for (U32 i = idxFreqL; i <= idxFreqH; i++)
		{
			vecValidRCS.push_back(vecRcs[i]);
		}

		std::fstream rcsFile("E:\\Files\\cube2.rcs", std::ios::trunc | std::ios::out | std::ios::binary);

		if (!rcsFile.good())
		{
			rcsFile.close();
		}
		int num = idxFreqH - idxFreqL + 1;
		rcsFile.write((char*)(&num), sizeof(U32));
		rcsFile.write((char*)(rcsPtr + idxFreqL), sizeof(T) * num);

		rcsFile.close();
		//-----------------------//
		std::fstream rcsFileX("E:\\Files\\cube2X.rcs", std::ios::trunc | std::ios::out | std::ios::binary);

		if (!rcsFileX.good())
		{
			rcsFileX.close();
		}
		//int num = idxFreqH - idxFreqL + 1;
		rcsFileX.write((char*)(&num), sizeof(U32));
		rcsFileX.write((char*)(rcsPtr + transNums + idxFreqL), sizeof(T) * num);

		rcsFileX.close();
	}

	void RcsTimeToFreqDomain(RayPool< T >& rayPool, RcsArray< T >& rcsArray, Excitation< T >& excite)
	{
		RayTube< T >* rayPtr = rayPool.rayTubeArray_.get();
		//RayTubeCorner< T >* rayCornerPtr = rayPool.rayTubeCornerArray_.get();
		T* esPtr = rcsArray.rcsArray_.get();
		T* st = excite.SeriesArray_.get();
		std::fill(esPtr, esPtr + rcsArray.rcsCount_, 0.0);
		for (U32 row = 0; row < rayPool.rayCountSqrt_; row++)
		{
			for (U32 col = 0; col < rayPool.rayCountSqrt_; col++) 
			{
				const RayTube< T >& ray = rayPtr[row * rayPool.rayCountSqrt_ + col];
				if (ray.refCount_ == 0) {
					continue;
				}
				else
				{
					T* esRay = ray.es_.get();
					U32 sampleNums = rcsArray.rcsCount_;
					for (U32 i = 0; i < sampleNums; i++)
					{
						esPtr[i] = esPtr[i] + esRay[i];
					}
				}
			}
		}
		U32 transNums = 1;
		while (transNums  <= rcsArray.rcsCount_)
		{
			transNums = transNums*2;
		}
		transNums = transNums / 2;

		std::vector<std::complex< T >> fft_arg;
		std::vector<std::complex< T >> fft_arg_st;
		for (U32 i = 0; i < transNums; i++)
		{
			fft_arg.push_back(std::complex< T >(esPtr[i], 0));
			fft_arg_st.push_back(std::complex< T >(st[i], 0));
		}
		auto fftEsData = FFT::fft1d(fft_arg, FFT::fft_dir::DIR_FWD);
		auto fftStData = FFT::fft1d(fft_arg_st, FFT::fft_dir::DIR_FWD);

		std::vector< T > vecEs;
		std::vector< T > vecEi;
		
		for (U32 i = 0; i < transNums; i++)
		{
			vecEs.push_back(std::abs(fftEsData[i]));
			vecEi.push_back(std::abs(fftStData[i]));
		}
		rcsArray.rcsCount_ = transNums;
		rcsArray.rcsArray_.reset(new T[transNums], [](T* ptr) { delete[] ptr; });
		T* rcsPtr = rcsArray.rcsArray_.get();
		std::vector< T > vecRcs;
		T rcsTmp;
		for (U32 i = 0; i < transNums; i++)
		{
			rcsTmp = (pow(z0, 2) / (4 * pi * pow(c0, 2))) * pow(vecEs[i], 2) / pow(vecEi[i], 2);
			rcsPtr[i] = 10 * std::log10f(rcsTmp);
			vecRcs.push_back(rcsTmp / (pi * pow(0.5, 2)));
		}
		//delta_f=fs/n_fft; fs=50*fH; 
		//N_fL = ceil(fL / delta_f);% 计算工作频段的最低频率fL对应的频谱图上横坐标序列号
		//N_fH = ceil(fH / delta_f);% 计算工作频段的最高频率fH对应的频谱图上横坐标序列号
		T deltaFreq = 50 * excite.freqHigh_ / excite.sampleNums_;
		U32 idxFreqL = std::ceil(excite.freqLow_ / deltaFreq);
		U32 idxFreqH = std::ceil(excite.freqHigh_ / deltaFreq);
		std::vector< T >vecValidRCS;
		for (U32 i = idxFreqL; i <= idxFreqH; i++)
		{
			vecValidRCS.push_back(rcsPtr[i]);
		}

		std::fstream rcsFile("E:\\Files\\cube2.rcs", std::ios::trunc | std::ios::out | std::ios::binary);

		if (!rcsFile.good())
		{
			rcsFile.close();
		}
		int num = idxFreqH - idxFreqL + 1;
		rcsFile.write((char*)(&num), sizeof(U32));
		rcsFile.write((char*)(rcsPtr+ idxFreqL), sizeof(T) * num);

		rcsFile.close();
		int dengy = 6;
	}

	
};


template< class T >
T SbrSolver< T >::c0 = 299792458.0;

template< class T >
T SbrSolver< T >::mu0 = 12.566370614E-7;

template< class T >
T SbrSolver< T >::eps0 = 8.854187817E-12;

template< class T >
T SbrSolver< T >::z0 = 376.730313451;

template< class T >
T SbrSolver< T >::pi = 3.14159265359;

template< class T >
T SbrSolver< T >::eps = 1E-7;

template< class T >
int SbrSolver< T >::N_Tracing_Max = 5;

template< class T >
int SbrSolver< T >::N_Tracing_Max_In = 10;

//template< class T >
//T SbrSolver< T >::epsilonR = 3.2;
//
//template< class T >
//T SbrSolver< T >::muR = 1.0;
//
//template< class T >
//std::complex< T > SbrSolver< T >::epsilonRC = std::complex< T >(6, -9);
//template< class T >
//std::complex< T > SbrSolver< T >::muRC = std::complex< T >(1, -2);




#endif