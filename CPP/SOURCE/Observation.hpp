#ifndef OBSERVATION_INCLUDED
#define OBSERVATION_INCLUDED

#include "LuVector.hpp"
#include "TypeDef.hpp"

template< class T >
class Observation
{
public:
	LUV::Vec3< T > direction_;
	LUV::Vec3< T > polarization_;
	LUV::Vec3< T > polarizationX_;
	LUV::Vec3< T > rayPol_;
	T frequency_;
	T frequencyHigh_;
	U32 rayPerLam_;
	U32 polCnt_;

public:
	Observation() restrict( cpu, amp )
	{

	}

	Observation( const LUV::Vec3< T >& direction, const LUV::Vec3< T >& polarization, const LUV::Vec3< T >& rayPol, const T& frequency, const T& frequencyHigh, const U32& rayPerLam ) restrict( cpu, amp ) :
		direction_( direction ),
		polarization_( polarization ),
		rayPol_(rayPol),
		frequency_( frequency ),
		frequencyHigh_(frequencyHigh),
		rayPerLam_( rayPerLam ),
		polCnt_(1)
	{

	}
	Observation(const LUV::Vec3< T >& direction, const LUV::Vec3< T >& polarization, const LUV::Vec3< T >& polarizationX, const LUV::Vec3< T >& rayPol, const T& frequency, const T& frequencyHigh, const U32& rayPerLam) restrict(cpu, amp) :
		direction_(direction),
		polarization_(polarization),
		polarizationX_(polarizationX),
		rayPol_(rayPol),
		frequency_(frequency),
		frequencyHigh_(frequencyHigh),
		rayPerLam_(rayPerLam),
		polCnt_(2)
	{

	}

	~Observation()
	{

	}
};

template< class T >
class Param
{
public:
	std::complex< T > epsilon_;
	std::complex< T > mu_;
public:
	Param() restrict(cpu, amp)
	{

	}

	Param(const std::complex< T >& epsilon, const std::complex< T >& mu) restrict(cpu, amp) :
		epsilon_(epsilon),
		mu_(mu)
	{

	}

	~Param()
	{

	}
};

template< class T >
class ParamCoat
{
public:
	std::complex< T > epsilon_;
	std::complex< T > mu_;
	T coatD_;
public:
	ParamCoat() restrict(cpu, amp)
	{

	}

	ParamCoat(const std::complex< T >& epsilon, const std::complex< T >& mu, const T& coatD) restrict(cpu, amp) :
		epsilon_(epsilon),
		mu_(mu),
		coatD_(coatD)
	{

	}

	~ParamCoat()
	{

	}
};
#endif