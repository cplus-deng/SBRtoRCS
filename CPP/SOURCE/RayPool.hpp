#ifndef RAY_POOL_INCLUDED
#define RAY_POOL_INCLUDED

#include "TypeDef.hpp"
#include "BoundBox.hpp"
#include "RayTube.hpp"

template< class T >
class RayPool
{
public:
	bool init_;
	U32 rayCountSqrt_;
	U32 rayCount_;
	U32 rayCornerCount_;
	T rayArea_;
	T kiDotMinDirN_;
	std::shared_ptr< RayTubeCorner< T >> rayTubeCornerArray_;
	std::shared_ptr< RayTube< T >> rayTubeArray_;
public:

	RayPool() :
		init_( false ),
		rayCountSqrt_( 0 ),
		rayCount_( 0 ),
		rayCornerCount_(0),
		rayArea_( 0 ),
		rayTubeArray_(),
		rayTubeCornerArray_()
	{

	}

	~RayPool()
	{

	}

	void Reset()
	{
		if( init_ )
		{
			init_ = false;
			rayCount_ = 0;
			rayCount_ = 0;
			rayArea_ = 0;
			rayTubeArray_.reset();
			rayTubeCornerArray_.reset();
		}
	}

	void Initialize( const U32& rayCountSqrt )
	{
		Reset();
		rayCountSqrt_ = rayCountSqrt;
		rayCount_ = rayCountSqrt * rayCountSqrt;
		rayCornerCount_ = (rayCountSqrt_ + 1) * (rayCountSqrt_ + 1);
		rayArea_ = 0;
		rayTubeArray_.reset( new RayTube< T >[ rayCount_ ], []( RayTube< T >* ptr ){ delete[] ptr; } );
		rayTubeCornerArray_.reset(new RayTubeCorner<T>[rayCornerCount_],
									[](RayTubeCorner<T>* ptr) { delete[] ptr; });
		init_ = true;
	}

	//void Initialize( const U32& rayCountSqrt, std::shared_ptr< RayTube< T > >& rayTubeArray )
	//{
	//	Reset();
	//	rayCountSqrt_ = rayCountSqrt;
	//	rayCount_ = rayCountSqrt * rayCountSqrt;
	//	rayTubeArray_ = rayTubeArray;
	//	init_ = true;
	//}
	void ReGenerateRaysAcc
	(
		const BoundBox< T >& boundBox,
		const LUV::Vec3< T >& incDir,
		const LUV::Vec3< T >& polDir
	)
	{
		//LUV::Vec3< std::complex< T >> polDir(polDirReal[0], polDirReal[1], polDirReal[2]);
		LUV::Vec3< T > outDirSph(LUV::CtsToSph(-incDir));
		LUV::Vec3< T > dirN; // ray direction
		LUV::Vec3< T > dirU;
		LUV::Vec3< T > dirR;
		LUV::OrthonormalSet(outDirSph[1], outDirSph[2], dirN, dirU, dirR);
		LUV::Orthonormalize(dirN, dirU, dirR);

		LUV::Vec3< T > boundBoxCenter = boundBox.GetCenter();
		T boundBoxRadius = boundBox.GetRadius();
		LUV::Vec3< T > rayPoolCenter = boundBoxCenter - dirN * 2.0 * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMin = rayPoolCenter - (dirR + dirU) * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMax = rayPoolCenter + (dirR + dirU) * boundBoxRadius;

		T rayTubeRadius = boundBoxRadius / (T)rayCountSqrt_;
		T rayTubeDiameter = rayTubeRadius * 2.0;
		LUV::Vec3< T > rayPosStepU = rayTubeDiameter * dirU;
		LUV::Vec3< T > rayPosStepR = rayTubeDiameter * dirR;
		LUV::Vec3< T > rayPosBegin = rayPoolRectMin + (rayPosStepU + rayPosStepR) / 2.0;
		LUV::Vec3< T > rayCornerPosBegin = rayPoolRectMin;

		rayArea_ = rayTubeDiameter * rayTubeDiameter;
		kiDotMinDirN_ = LUV::Dot(-incDir, boundBoxCenter - dirN * boundBoxRadius); //incDir = obsDir , ki = -incDir
		RayTube< T >* rayArray = rayTubeArray_.get();
		RayTubeCorner< T >* cornerArray = rayTubeCornerArray_.get();

		for (U32 idU = 0; idU < rayCountSqrt_; ++idU)
		{
			for (U32 idR = 0; idR < rayCountSqrt_; ++idR)
			{
				U32 idRay = idR + rayCountSqrt_ * idU;
				rayArray[idRay].pos_ = rayPosBegin + rayPosStepU * (T)idU + rayPosStepR * (T)idR;
				rayArray[idRay].dir_ = dirN;
				rayArray[idRay].pol_ = polDir;
				rayArray[idRay].dist_ = 1E36;
				rayArray[idRay].delay_ = 0.0;
				rayArray[idRay].refCount_ = 0;
				rayArray[idRay].lastHitIdx_ = -1;
			}
		}
		for (U32 idU = 0; idU <= rayCountSqrt_; ++idU)
		{
			for (U32 idR = 0; idR <= rayCountSqrt_; ++idR)
			{
				U32 idRay = idR + (rayCountSqrt_ + 1) * idU;
				cornerArray[idRay].pos_ = rayCornerPosBegin + rayPosStepU * (T)idU + rayPosStepR * (T)idR;
				cornerArray[idRay].dir_ = dirN;
				cornerArray[idRay].pol_ = polDir;
				cornerArray[idRay].dist_ = 1E36;
				cornerArray[idRay].refCount_ = 0;
				cornerArray[idRay].lastHitIdx_ = -1;
				cornerArray[idRay].refCoords_.reset(new LUV::Vec3< T >[5], [](LUV::Vec3< T >* ptr) { delete[] ptr; });
				cornerArray[idRay].hitIdx_.reset(new U32[5], [](U32* ptr) { delete[] ptr; });
			}
		}

	}
	void ReGenerateRays
	(
		const BoundBox< T >& boundBox,
		const LUV::Vec3< T >& incDir,
		const LUV::Vec3< T >& polDir
	)
	{
		LUV::Vec3< T > outDirSph( LUV::CtsToSph( -incDir ) );
		LUV::Vec3< T > dirN; // ray direction
		LUV::Vec3< T > dirU;
		LUV::Vec3< T > dirR;
		LUV::OrthonormalSet( outDirSph[1], outDirSph[2], dirN, dirU, dirR );
		LUV::Orthonormalize( dirN, dirU, dirR );

		LUV::Vec3< T > boundBoxCenter = boundBox.GetCenter();
		T boundBoxRadius = boundBox.GetRadius();
		LUV::Vec3< T > rayPoolCenter = boundBoxCenter - dirN * 2.0 * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMin = rayPoolCenter - ( dirR + dirU ) * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMax = rayPoolCenter + ( dirR + dirU ) * boundBoxRadius;

		T rayTubeRadius = boundBoxRadius / (T)rayCountSqrt_;
		T rayTubeDiameter = rayTubeRadius * 2.0;
		LUV::Vec3< T > rayPosStepU = rayTubeDiameter * dirU;
		LUV::Vec3< T > rayPosStepR = rayTubeDiameter * dirR;
		LUV::Vec3< T > rayPosBegin = rayPoolRectMin + ( rayPosStepU + rayPosStepR ) / 2.0;

		rayArea_ = rayTubeDiameter * rayTubeDiameter;

		RayTube< T >* rayArray = rayTubeArray_.get();

		for( U32 idU = 0; idU < rayCountSqrt_; ++idU )
		{
			for( U32 idR = 0; idR < rayCountSqrt_; ++idR )
			{
				U32 idRay = idR + rayCountSqrt_ * idU;
				rayArray[ idRay ].pos_ = rayPosBegin + rayPosStepU * (T)idU + rayPosStepR * (T)idR;
				rayArray[ idRay ].dir_ = dirN;
				rayArray[ idRay ].pol_ = polDir;
				rayArray[ idRay ].dist_ = 1E36;
				rayArray[ idRay ].refCount_ = 0;
				rayArray[ idRay ].lastHitIdx_ = -1;
			}
		}

	}

	void ReGenerateRaysRand
	(
		const BoundBox< T >& boundBox,
		const LUV::Vec3< T >& incDir,
		const LUV::Vec3< T >& polDir
	)
	{
		LUV::Vec3< T > outDirSph( LUV::CtsToSph( -incDir ) );
		LUV::Vec3< T > dirN; // ray direction
		LUV::Vec3< T > dirU;
		LUV::Vec3< T > dirR;
		LUV::OrthonormalSet( outDirSph[1], outDirSph[2], dirN, dirU, dirR );
		LUV::Orthonormalize( dirN, dirU, dirR );

		LUV::Vec3< T > boundBoxCenter = boundBox.GetCenter();
		T boundBoxRadius = boundBox.GetRadius();
		LUV::Vec3< T > rayPoolCenter = boundBoxCenter - dirN * 2.0 * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMin = rayPoolCenter - ( dirR + dirU ) * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMax = rayPoolCenter + ( dirR + dirU ) * boundBoxRadius;

		T rayTubeRadius = boundBoxRadius / (T)rayCountSqrt_;
		T rayTubeDiameter = rayTubeRadius * 2.0;
		LUV::Vec3< T > rayPosStepU = rayTubeDiameter * dirU;
		LUV::Vec3< T > rayPosStepR = rayTubeDiameter * dirR;
		LUV::Vec3< T > rayPosBegin = rayPoolRectMin + ( rayPosStepU + rayPosStepR ) / 2.0;

		rayArea_ = rayTubeDiameter * rayTubeDiameter;

		RayTube< T >* rayArray = rayTubeArray_.get();





		std::srand(std::time(0));


		std::vector< U32 > mixIdRay;
		mixIdRay.reserve( rayCount_ );
		for( U32 idx = 0; idx < rayCount_; idx++ )
		{
			mixIdRay.push_back( idx );
		}
		mixIdRay.shrink_to_fit();
		std::random_shuffle( mixIdRay.begin(), mixIdRay.end() );




		for( U32 idU = 0; idU < rayCountSqrt_; ++idU )
		{
			for( U32 idR = 0; idR < rayCountSqrt_; ++idR )
			{
				U32 idRay = idR + rayCountSqrt_ * idU;

				U32 idMix = mixIdRay[ idRay ];
				U32 idMixR = idMix % rayCountSqrt_;
				U32 idMixU = ( idMix - idMixR ) / rayCountSqrt_;

				//std::cout << idMix << std::endl;

				rayArray[ idRay ].pos_ = rayPosBegin + rayPosStepU * (T)idMixU + rayPosStepR * (T)idMixR;
				rayArray[ idRay ].dir_ = dirN;
				rayArray[ idRay ].pol_ = polDir;
				rayArray[ idRay ].dist_ = 1E36;
				rayArray[ idRay ].refCount_ = 0;
				rayArray[ idRay ].lastHitIdx_ = -1;
			}
		}


	}


};



template< class T >
class RayPoolDie
{
public:
	bool init_;
	U32 rayCountSqrt_;
	U32 rayCount_;
	U32 rayCornerCount_;
	T rayArea_;
	T kiDotMinDirN_;
	std::shared_ptr< RayTubeCorDie< T >> rayTubeCornerArray_;
	std::shared_ptr< RayTubeDie< T >> rayTubeArray_;
public:

	RayPoolDie() :
		init_(false),
		rayCountSqrt_(0),
		rayCount_(0),
		rayCornerCount_(0),
		rayArea_(0),
		rayTubeArray_(),
		rayTubeCornerArray_()
	{

	}

	~RayPoolDie()
	{

	}

	void Reset()
	{
		if (init_)
		{
			init_ = false;
			rayCount_ = 0;
			rayCount_ = 0;
			rayArea_ = 0;
			rayTubeArray_.reset();
			rayTubeCornerArray_.reset();
		}
	}

	void Initialize(const U32& rayCountSqrt)
	{
		Reset();
		rayCountSqrt_ = rayCountSqrt;
		rayCount_ = rayCountSqrt * rayCountSqrt;
		rayCornerCount_ = (rayCountSqrt_ + 1) * (rayCountSqrt_ + 1);
		rayArea_ = 0;
		rayTubeArray_.reset(new RayTubeDie< T >[rayCount_], [](RayTubeDie< T >* ptr) { delete[] ptr; });
		rayTubeCornerArray_.reset(new RayTubeCorDie<T>[rayCornerCount_],
			[](RayTubeCorDie<T>* ptr) { delete[] ptr; });
		init_ = true;
	}

	//void Initialize( const U32& rayCountSqrt, std::shared_ptr< RayTube< T > >& rayTubeArray )
	//{
	//	Reset();
	//	rayCountSqrt_ = rayCountSqrt;
	//	rayCount_ = rayCountSqrt * rayCountSqrt;
	//	rayTubeArray_ = rayTubeArray;
	//	init_ = true;
	//}
	void ReGenerateRaysAcc
	(
		const BoundBox< T >& boundBox,
		const LUV::Vec3< T >& incDir,
		const LUV::Vec3< T >& polDirReal
	)
	{
		LUV::Vec3< std::complex< T >> polDir(polDirReal[0], polDirReal[1], polDirReal[2]);
		LUV::Vec3< T > outDirSph(LUV::CtsToSph(-incDir));
		LUV::Vec3< T > dirN; // ray direction
		LUV::Vec3< T > dirU;
		LUV::Vec3< T > dirR;
		LUV::OrthonormalSet(outDirSph[1], outDirSph[2], dirN, dirU, dirR);
		LUV::Orthonormalize(dirN, dirU, dirR);

		LUV::Vec3< T > boundBoxCenter = boundBox.GetCenter();
		T boundBoxRadius = boundBox.GetRadius();
		LUV::Vec3< T > rayPoolCenter = boundBoxCenter - dirN * 2.0 * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMin = rayPoolCenter - (dirR + dirU) * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMax = rayPoolCenter + (dirR + dirU) * boundBoxRadius;

		T rayTubeRadius = boundBoxRadius / (T)rayCountSqrt_;
		T rayTubeDiameter = rayTubeRadius * 2.0;
		LUV::Vec3< T > rayPosStepU = rayTubeDiameter * dirU;
		LUV::Vec3< T > rayPosStepR = rayTubeDiameter * dirR;
		LUV::Vec3< T > rayPosBegin = rayPoolRectMin + (rayPosStepU + rayPosStepR) / 2.0;
		LUV::Vec3< T > rayCornerPosBegin = rayPoolRectMin;

		rayArea_ = rayTubeDiameter * rayTubeDiameter;
		kiDotMinDirN_ = LUV::Dot(-incDir, boundBoxCenter - dirN * boundBoxRadius); //incDir = obsDir , ki = -incDir
		RayTubeDie< T >* rayArray = rayTubeArray_.get();
		RayTubeCorDie< T >* cornerArray = rayTubeCornerArray_.get();

		for (U32 idU = 0; idU < rayCountSqrt_; ++idU)
		{
			for (U32 idR = 0; idR < rayCountSqrt_; ++idR)
			{
				U32 idRay = idR + rayCountSqrt_ * idU;
				rayArray[idRay].pos_ = rayPosBegin + rayPosStepU * (T)idU + rayPosStepR * (T)idR;
				rayArray[idRay].dir_ = dirN;
				rayArray[idRay].pol_ = polDir;
				rayArray[idRay].dist_ = 1E36;
				rayArray[idRay].delay_ = 0.0;
				rayArray[idRay].refCount_ = 0;
				rayArray[idRay].lastHitIdx_ = -1;
			}
		}
		for (U32 idU = 0; idU <= rayCountSqrt_; ++idU)
		{
			for (U32 idR = 0; idR <= rayCountSqrt_; ++idR)
			{
				U32 idRay = idR + (rayCountSqrt_ + 1) * idU;
				cornerArray[idRay].pos_ = rayCornerPosBegin + rayPosStepU * (T)idU + rayPosStepR * (T)idR;
				cornerArray[idRay].dir_ = dirN;
				cornerArray[idRay].pol_ = polDirReal;
				cornerArray[idRay].dist_ = 1E36;
				cornerArray[idRay].refCount_ = 0;
				cornerArray[idRay].lastHitIdx_ = -1;
				/*cornerArray[idRay].refCoords_.reset(new LUV::Vec3< T >[5], [](LUV::Vec3< T >* ptr) { delete[] ptr; });
				cornerArray[idRay].hitIdx_.reset(new U32[5], [](U32* ptr) { delete[] ptr; });*/
			}
		}

	}
	void ReGenerateRays
	(
		const BoundBox< T >& boundBox,
		const LUV::Vec3< T >& incDir,
		const LUV::Vec3< T >& polDir
	)
	{
		LUV::Vec3< T > outDirSph(LUV::CtsToSph(-incDir));
		LUV::Vec3< T > dirN; // ray direction
		LUV::Vec3< T > dirU;
		LUV::Vec3< T > dirR;
		LUV::OrthonormalSet(outDirSph[1], outDirSph[2], dirN, dirU, dirR);
		LUV::Orthonormalize(dirN, dirU, dirR);

		LUV::Vec3< T > boundBoxCenter = boundBox.GetCenter();
		T boundBoxRadius = boundBox.GetRadius();
		LUV::Vec3< T > rayPoolCenter = boundBoxCenter - dirN * 2.0 * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMin = rayPoolCenter - (dirR + dirU) * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMax = rayPoolCenter + (dirR + dirU) * boundBoxRadius;

		T rayTubeRadius = boundBoxRadius / (T)rayCountSqrt_;
		T rayTubeDiameter = rayTubeRadius * 2.0;
		LUV::Vec3< T > rayPosStepU = rayTubeDiameter * dirU;
		LUV::Vec3< T > rayPosStepR = rayTubeDiameter * dirR;
		LUV::Vec3< T > rayPosBegin = rayPoolRectMin + (rayPosStepU + rayPosStepR) / 2.0;

		rayArea_ = rayTubeDiameter * rayTubeDiameter;

		RayTubeDie< T >* rayArray = rayTubeArray_.get();

		for (U32 idU = 0; idU < rayCountSqrt_; ++idU)
		{
			for (U32 idR = 0; idR < rayCountSqrt_; ++idR)
			{
				U32 idRay = idR + rayCountSqrt_ * idU;
				rayArray[idRay].pos_ = rayPosBegin + rayPosStepU * (T)idU + rayPosStepR * (T)idR;
				rayArray[idRay].dir_ = dirN;
				rayArray[idRay].pol_ = polDir;
				rayArray[idRay].dist_ = 1E36;
				rayArray[idRay].refCount_ = 0;
				rayArray[idRay].lastHitIdx_ = -1;
			}
		}

	}

	void ReGenerateRaysRand
	(
		const BoundBox< T >& boundBox,
		const LUV::Vec3< T >& incDir,
		const LUV::Vec3< T >& polDir
	)
	{
		LUV::Vec3< T > outDirSph(LUV::CtsToSph(-incDir));
		LUV::Vec3< T > dirN; // ray direction
		LUV::Vec3< T > dirU;
		LUV::Vec3< T > dirR;
		LUV::OrthonormalSet(outDirSph[1], outDirSph[2], dirN, dirU, dirR);
		LUV::Orthonormalize(dirN, dirU, dirR);

		LUV::Vec3< T > boundBoxCenter = boundBox.GetCenter();
		T boundBoxRadius = boundBox.GetRadius();
		LUV::Vec3< T > rayPoolCenter = boundBoxCenter - dirN * 2.0 * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMin = rayPoolCenter - (dirR + dirU) * boundBoxRadius;
		LUV::Vec3< T > rayPoolRectMax = rayPoolCenter + (dirR + dirU) * boundBoxRadius;

		T rayTubeRadius = boundBoxRadius / (T)rayCountSqrt_;
		T rayTubeDiameter = rayTubeRadius * 2.0;
		LUV::Vec3< T > rayPosStepU = rayTubeDiameter * dirU;
		LUV::Vec3< T > rayPosStepR = rayTubeDiameter * dirR;
		LUV::Vec3< T > rayPosBegin = rayPoolRectMin + (rayPosStepU + rayPosStepR) / 2.0;

		rayArea_ = rayTubeDiameter * rayTubeDiameter;

		RayTubeDie< T >* rayArray = rayTubeArray_.get();





		std::srand(std::time(0));


		std::vector< U32 > mixIdRay;
		mixIdRay.reserve(rayCount_);
		for (U32 idx = 0; idx < rayCount_; idx++)
		{
			mixIdRay.push_back(idx);
		}
		mixIdRay.shrink_to_fit();
		std::random_shuffle(mixIdRay.begin(), mixIdRay.end());




		for (U32 idU = 0; idU < rayCountSqrt_; ++idU)
		{
			for (U32 idR = 0; idR < rayCountSqrt_; ++idR)
			{
				U32 idRay = idR + rayCountSqrt_ * idU;

				U32 idMix = mixIdRay[idRay];
				U32 idMixR = idMix % rayCountSqrt_;
				U32 idMixU = (idMix - idMixR) / rayCountSqrt_;

				//std::cout << idMix << std::endl;

				rayArray[idRay].pos_ = rayPosBegin + rayPosStepU * (T)idMixU + rayPosStepR * (T)idMixR;
				rayArray[idRay].dir_ = dirN;
				rayArray[idRay].pol_ = polDir;
				rayArray[idRay].dist_ = 1E36;
				rayArray[idRay].refCount_ = 0;
				rayArray[idRay].lastHitIdx_ = -1;
			}
		}


	}


};

#endif