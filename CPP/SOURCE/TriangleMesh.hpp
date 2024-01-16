#ifndef TRIANGLE_MESH_INCLUDED
#define TRIANGLE_MESH_INCLUDED

#include <vector>
#include <algorithm>
#include <fstream>

#include "TypeDef.hpp"
#include "BoundBox.hpp"
#include "BoundSphere.hpp"
#include "Triangle.hpp"
#include "UnvTrigMeshFile.hpp"
#include "StlTrigMeshFile.h"
#include "ObservationArray.hpp"
#include "Excitation.h"

template< class T >
class TriangleMesh
{
public:
	U32 trigCount_;
	std::vector< Triangle< T > > trigArray_;
	BoundSphere< T > boundSphere_;
	BoundBox< T > boundBox_;
public:
	static T pi;
	static T z0;
	static T c0;
public:
	TriangleMesh()
	{

	}

	TriangleMesh( const U32& reservedTrigCount ) :
		trigCount_( 0 ),
		trigArray_(),
		boundSphere_(),
		boundBox_()
	{
		trigArray_.reserve( reservedTrigCount );
	}

	~TriangleMesh()
	{

	}

	void Reset( const U32& reservedTrigCount )
	{
		trigCount_ = 0;
		trigArray_.clear();
		trigArray_.reserve( reservedTrigCount );
		boundSphere_ = BoundSphere< T >();
		boundBox_ = BoundBox< T >();
	}

	void CalculateBounds()
	{
		boundBox_ = trigArray_[0].GetBoundBox();
		for( const Triangle< T >& trig : trigArray_ )
		{
			boundBox_ = boundBox_.UnionWith( trig.GetBoundBox() );
		}

		boundSphere_.center_ = ( boundBox_.min_ + boundBox_.max_ ) / 2;
		boundSphere_.radius_ = LUV::Length( boundBox_.max_ - boundSphere_.center_ );
	}

	void InsertTrig( const Triangle< T >& trig )
	{
		trigArray_.push_back( trig );
		trigCount_ = trigArray_.size();
	}
	void ImportFromStlTrigMeshFile( const StlTrigMeshFile< T >& stlTrigMeshFile)
	{
		Reset(stlTrigMeshFile.trigCount_);
		T* vertexDataPtr = stlTrigMeshFile.vertexData_.get();
		T* normalDataPtr = stlTrigMeshFile.normalData_.get();
		U32* trigMeterialIndexPtr = stlTrigMeshFile.trigMeterialIndex_.get();

		for (U32 trigIndex = 0; trigIndex < stlTrigMeshFile.trigCount_; trigIndex++)
		{
			LUV::Vec3< T > v1(vertexDataPtr[trigIndex * 9], vertexDataPtr[trigIndex * 9 + 1], vertexDataPtr[trigIndex * 9 + 2]);
			LUV::Vec3< T > v2(vertexDataPtr[trigIndex * 9 + 3], vertexDataPtr[trigIndex * 9 + 4], vertexDataPtr[trigIndex * 9 + 5]);
			LUV::Vec3< T > v3(vertexDataPtr[trigIndex * 9 + 6], vertexDataPtr[trigIndex * 9 + 7], vertexDataPtr[trigIndex * 9 + 8]);
			LUV::Vec3< T > normal(normalDataPtr[trigIndex * 3], normalDataPtr[trigIndex * 3 + 1], normalDataPtr[trigIndex * 3 + 2]);
			U32 mtype = trigMeterialIndexPtr[trigIndex];
			trigArray_.push_back(Triangle< T >(v1, v2, v3, normal, mtype));
		}

		trigCount_ = trigArray_.size();
		CalculateBounds();
	}

	void ImportFromStlTrigMeshFileVec(const std::vector<StlTrigMeshFile< T >>& stlTrigMeshFileVec)
	{
		U32 nMesh = stlTrigMeshFileVec.size();
		U32 ntrigCount = 0;
		for (U32 i = 0; i < nMesh; i++)
		{
			ntrigCount += stlTrigMeshFileVec[i].trigCount_;
		}
		Reset(ntrigCount);

		for (U32 i = 0; i < nMesh; i++)
		{
			const StlTrigMeshFile< T > stlTrigMeshFile = stlTrigMeshFileVec[i];
			T* vertexDataPtr = stlTrigMeshFile.vertexData_.get();
			T* normalDataPtr = stlTrigMeshFile.normalData_.get();
			U32* trigMeterialIndexPtr = stlTrigMeshFile.trigMeterialIndex_.get();

			for (U32 trigIndex = 0; trigIndex < stlTrigMeshFile.trigCount_; trigIndex++)
			{
				LUV::Vec3< T > v1(vertexDataPtr[trigIndex * 9], vertexDataPtr[trigIndex * 9 + 1], vertexDataPtr[trigIndex * 9 + 2]);
				LUV::Vec3< T > v2(vertexDataPtr[trigIndex * 9 + 3], vertexDataPtr[trigIndex * 9 + 4], vertexDataPtr[trigIndex * 9 + 5]);
				LUV::Vec3< T > v3(vertexDataPtr[trigIndex * 9 + 6], vertexDataPtr[trigIndex * 9 + 7], vertexDataPtr[trigIndex * 9 + 8]);
				LUV::Vec3< T > normal(normalDataPtr[trigIndex * 3], normalDataPtr[trigIndex * 3 + 1], normalDataPtr[trigIndex * 3 + 2]);
				U32 mtype = trigMeterialIndexPtr[trigIndex];
				trigArray_.push_back(Triangle< T >(v1, v2, v3, normal, mtype));
			}
		}

		trigCount_ = trigArray_.size();
		CalculateBounds();
	}
	void ImportFromUnvTrigMeshFile( const UnvTrigMeshFile< T >& unvTrigMeshFile )
	{
		Reset( unvTrigMeshFile.trigCount_ );

		T* vertexPtr = unvTrigMeshFile.vertexData_.get();
		U32* indexPtr = unvTrigMeshFile.trigVertexIndex_.get();

		for( U32 trigIndex = 0; trigIndex < unvTrigMeshFile.trigCount_; ++trigIndex )
		{
			U32 indexV1 = trigIndex * 3;
			U32 indexV2 = indexV1 + 1;
			U32 indexV3 = indexV2 + 1;

			U32 indexV1x = indexPtr[ indexV1 ] * 3;
			U32 indexV1y = indexV1x + 1;
			U32 indexV1z = indexV1y + 1;

			U32 indexV2x = indexPtr[ indexV2 ] * 3;
			U32 indexV2y = indexV2x + 1;
			U32 indexV2z = indexV2y + 1;

			U32 indexV3x = indexPtr[ indexV3 ] * 3;
			U32 indexV3y = indexV3x + 1;
			U32 indexV3z = indexV3y + 1;

			LUV::Vec3< T > v1( vertexPtr[ indexV1x ], vertexPtr[ indexV1y ], vertexPtr[ indexV1z ] );
			LUV::Vec3< T > v2( vertexPtr[ indexV2x ], vertexPtr[ indexV2y ], vertexPtr[ indexV2z ] );
			LUV::Vec3< T > v3( vertexPtr[ indexV3x ], vertexPtr[ indexV3y ], vertexPtr[ indexV3z ] );

			trigArray_.push_back( Triangle< T >( v1, v2, v3 ) );
		}
		
		trigCount_ = trigArray_.size();
		CalculateBounds();
	}	

	void FindEdge(const T& txd, const Excitation< T >& excite, const Observation< T >& obs, std::complex< T >* es)
	{
		int edgeCnt = 0;
		std::vector< LUV::Vec3< T > > edge;
		std::vector< T > edgeAngleNPi;
		std::vector< int > edgeTriInc;

		const T fre = obs.frequency_;
		T k0 = 2 * pi * fre / c0;
		const LUV::Vec3< T > ki = -obs.direction_;
		const LUV::Vec3< T > esPol = obs.polarization_;
		const T deltaT = excite.deltaTime_;
		T* stdt = excite.DeriveSeriesArray_.get();
		const LUV::Vec3< T > e0 = obs.rayPol_;

		for (int i = 0; i < trigCount_; i++)
		{
			int edgeCountI = 0;
			for (int j = i + 1; j < trigCount_; j++)
			{
				int commonVertexCnt = 0;
				std::vector< LUV::Vec3< T > > commonVertexVec;
				findCommonVertex(trigArray_.at(i), trigArray_.at(j), commonVertexVec, commonVertexCnt);
				if (commonVertexCnt == 2)
				{
					edgeCountI += 1;
					if (LUV::Dot(trigArray_.at(i).normal_, trigArray_.at(j).normal_) <= 0.866f)
					{
						if (LUV::Dot(trigArray_.at(i).normal_, -ki) > 0.000001f || LUV::Dot(trigArray_.at(j).normal_, -ki) > 0.000001f)
						{
							edgeCnt += 1;
							edge.push_back(commonVertexVec.at(0));
							edge.push_back(commonVertexVec.at(1));
							T angleEdge = pi - LUV::_Acos(LUV::Dot(trigArray_.at(i).normal_, trigArray_.at(j).normal_));
							edgeAngleNPi.push_back(2 * pi - angleEdge);
							if (LUV::Dot(trigArray_.at(i).normal_, -ki) > LUV::Dot(trigArray_.at(j).normal_, -ki))
							{
								edgeTriInc.push_back(i);
							}
							else
							{
								edgeTriInc.push_back(j);
							}
						}
					}
				}
				if (edgeCountI == 3)
				{
					break;
				}
			}
		}
		for (int i = 0; i < edgeCnt; i++)
		{
			LUV::Vec3< T > node1 = edge.at(i * 2);
			LUV::Vec3< T > node2 = edge.at(i * 2 + 1);
			LUV::Vec3< T > zAxis = LUV::Unit(node1 - node2);
			LUV::Vec3< T > normalTri = trigArray_.at(edgeTriInc.at(i)).normal_;
			LUV::Vec3< T > ks = -ki;

			T betaI = LUV::_Acos(LUV::Dot(zAxis, ki));
			T betaS = LUV::_Acos(LUV::Dot(zAxis, ks));

			LUV::Vec3< T > sz = LUV::Unit(ks - zAxis * LUV::Dot(ks, zAxis));
			T phiS = LUV::_Acos(LUV::Dot(sz, normalTri)) + pi / 2;

			LUV::Vec3< T > iz = LUV::Unit(ki - zAxis * LUV::Dot(ki, zAxis));
			T phiI = LUV::_Acos(LUV::Dot(iz, normalTri)) + pi / 2;

			T nPi = edgeAngleNPi.at(i) / pi;

			T et = LUV::Dot(zAxis, e0);
			LUV::Vec3< T > h0 = 1.0f / z0 * LUV::Cross(ki, e0);
			T ht = LUV::Dot(zAxis, h0);

			T u1;
			if (pi-phiI < 0.0f)
			{
				u1 = 0.0f;
			}
			else
			{
				u1 = 1.0f;
			}
			T cosGama = LUV::_Sin(betaI) * LUV::_Sin(betaS) * LUV::_Cos(phiS) + LUV::_Cos(betaI) * LUV::_Cos(betaS);
			T mu1 = (cosGama - LUV::_Cos(betaS) * LUV::_Cos(betaS)) / (LUV::_Sin(betaI) * LUV::_Sin(betaI));
			std::complex< T > alpha1;
			std::complex< T > j(0.0f, 1.0f);
			if (mu1 > -1.0f && mu1 < 1.0f)
			{
				alpha1 = LUV::_Acos(mu1);
			}
			else
			{
				alpha1 = -j * std::log(mu1 - std::sqrtf(mu1 * mu1 - 1));
			}

			std::complex< T > i1poEt = 2.0f * j * u1 * LUV::_Sin(phiI) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI) + mu1) * z0 * LUV::_Sin(betaI));
			std::complex< T > i1poHt = -2.0f * j * u1 * ((1.0f / LUV::_Cos(betaI)) * LUV::_Cos(phiI) + (1.0f / LUV::_Cos(betaS)) * LUV::_Cos(phiS)) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI) + mu1));

			std::complex< T > m1poHt = -2.0f * j * z0 * LUV::_Sin(phiS) * u1 / (k0 * LUV::_Sin(betaI) * LUV::_Sin(betaS) * (LUV::_Cos(phiI) + mu1));

			std::complex< T > i1Et = 2.0f * j / nPi * LUV::_Sin(phiI / nPi) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI / nPi) - LUV::_Cos((pi - alpha1) / nPi)) * z0 * LUV::_Sin(betaI));
			std::complex< T > i1Ht = 2.0f * j / nPi * LUV::_Sin((pi - alpha1) / nPi) * (mu1 * (1.0f / LUV::_Tan(betaI)) - (1.0f / LUV::_Tan(betaS)) * LUV::_Cos(phiS)) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI / nPi) - LUV::_Cos((pi - alpha1) / nPi)) * LUV::_Sin(alpha1));

			std::complex< T > m1Ht = 2.0f * j * z0 * LUV::_Sin(phiS) / nPi * LUV::_Sin((pi - alpha1) / nPi) * (1.0f / LUV::_Sin(alpha1)) / (k0 * LUV::_Sin(betaI) * LUV::_Sin(betaS) * (LUV::_Cos((pi - alpha1) / nPi) - LUV::_Cos(phiI / nPi)));

			std::complex< T > i1fEt = i1Et - i1poEt;
			std::complex< T > i1fHt = i1Ht - i1poHt;
			std::complex< T > m1fHt = m1Ht - m1poHt;

			betaI = pi - betaI;
			betaS = pi - betaS;
			phiI = nPi * pi - phiI;
			phiS = nPi * pi - phiS;

			if (pi - phiI < 0.0f)
			{
				u1 = 0.0f;
			}
			else
			{
				u1 = 1.0f;
			}
			cosGama = LUV::_Sin(betaI) * LUV::_Sin(betaS) * LUV::_Cos(phiS) + LUV::_Cos(betaI) * LUV::_Cos(betaS);
			mu1 = (cosGama - LUV::_Cos(betaS) * LUV::_Cos(betaS)) / (LUV::_Sin(betaI) * LUV::_Sin(betaI));

			if (mu1 > -1.0f && mu1 < 1.0f)
			{
				alpha1 = LUV::_Acos(mu1);
			}
			else
			{
				alpha1 = -j * std::logf(mu1 - std::sqrtf(mu1 * mu1 - 1));
			}

			i1poEt = 2.0f * j * u1 * LUV::_Sin(phiI) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI) + mu1) * z0 * LUV::_Sin(betaI));
			i1poHt = -2.0f * j * u1 * ((1.0f / LUV::_Cos(betaI)) * LUV::_Cos(phiI) + (1.0f / LUV::_Cos(betaS)) * LUV::_Cos(phiS)) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI) + mu1));

			m1poHt = -2.0f * j * z0 * LUV::_Sin(phiS) * u1 / (k0 * LUV::_Sin(betaI) * LUV::_Sin(betaS) * (LUV::_Cos(phiI) + mu1));

			i1Et = 2.0f * j / nPi * LUV::_Sin(phiI / nPi) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI / nPi) - LUV::_Cos((pi - alpha1) / nPi)) * z0 * LUV::_Sin(betaI));
			i1Ht = 2.0f * j / nPi * LUV::_Sin((pi - alpha1) / nPi) * (mu1 * (1.0f / LUV::_Tan(betaI)) - (1.0f / LUV::_Tan(betaS)) * LUV::_Cos(phiS)) / (k0 * LUV::_Sin(betaI) * (LUV::_Cos(phiI / nPi) - LUV::_Cos((pi - alpha1) / nPi)) * LUV::_Sin(alpha1));

			m1Ht = 2.0f * j * z0 * LUV::_Sin(phiS) * LUV::_Sin((pi - alpha1) / nPi) * (1.0f / LUV::_Sin(alpha1)) / nPi / (k0 * LUV::_Sin(betaI) * LUV::_Sin(betaS) * (LUV::_Cos((pi - alpha1) / nPi) - LUV::_Cos(phiI / nPi)));

			std::complex< T > i2poEt = -i1poEt;
			std::complex< T > i2poHt = -i1poHt;
			std::complex< T > m2poHt = -m1poHt;
			std::complex< T > i2Et = -i1Et;
			std::complex< T > i2Ht = -i1Ht;
			std::complex< T > m2Ht = -m1Ht;

			std::complex< T > i2fEt = i2Et - i2poEt;
			std::complex< T > i2fHt = i2Ht - i2poHt;
			std::complex< T > m2fHt = m2Ht - m2poHt;

			std::complex< T > iIf = (i1fEt - i2fEt) * et + (i1fHt - i2fHt) * ht;
			std::complex< T > mMf = (m1fHt - m2fHt) * ht;

			T lengthEdge = LUV::Length(node1 - node2);
			LUV::Vec3< T > rc = (node1 + node2) / 2.0f;
			if (!std::isnan(iIf.real()) && !std::isnan(mMf.real()) && !std::isinf(iIf.real()) && !std::isinf(mMf.real()))
			{
				LUV::Vec3< std::complex< T > > jIf = LUV::Vec3< std::complex< T > >(zAxis) * iIf;
				LUV::Vec3< std::complex< T > > jMf = LUV::Vec3< std::complex< T > >(zAxis) * mMf;

				LUV::Vec3< std::complex< T > > tmp1 = LUV::CrossX(ki, jIf) - jMf / z0;
				LUV::Vec3< std::complex< T > > eq = LUV::CrossX(ki, tmp1);
				eq = eq * lengthEdge;

				std::complex< T > eqPol = LUV::Dot(eq, esPol);

				T tDelay1 = 2.0f * LUV::Dot(ki, rc) / c0;
				T tDelayTotal = tDelay1 + txd;
				U32 nDelay = std::ceil(tDelayTotal / deltaT);

				for (int kk = 0; kk < excite.sampleNums_; kk++)
				{
					int kkn = kk - nDelay;
					if (kkn >= 0 && kkn < excite.sampleNums_)
					{
						es[kk] += stdt[kkn] * eqPol;
					}
				}
			}
		}
	}

	inline bool sameVertex(const LUV::Vec3< T >& lhs, const LUV::Vec3< T >& rhs)
	{
		return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
	}

	inline void findCommonVertex(const Triangle< T >& t1, const Triangle< T >& t2, std::vector< LUV::Vec3< T > >& EdgeI, int& edgeCountI)
	{
		if (sameVertex(t1.v1_,t2.v1_) || sameVertex(t1.v1_, t2.v2_) || sameVertex(t1.v1_, t2.v3_))
		{
			EdgeI.push_back(t1.v1_); 
			edgeCountI += 1;
		}
		if (sameVertex(t1.v2_, t2.v1_) || sameVertex(t1.v2_, t2.v2_) || sameVertex(t1.v2_, t2.v3_))
		{
			EdgeI.push_back(t1.v2_);
			edgeCountI += 1;
		}
		if (sameVertex(t1.v3_, t2.v1_) || sameVertex(t1.v3_, t2.v2_) || sameVertex(t1.v3_, t2.v3_))
		{
			EdgeI.push_back(t1.v3_);
			edgeCountI += 1;
		}
	}
};
template< class T >
T TriangleMesh< T >::z0 = 376.730313451;

template< class T >
T TriangleMesh< T >::pi = 3.14159265359;

template< class T >
T TriangleMesh< T >::c0 = 299792458.0;

#endif
