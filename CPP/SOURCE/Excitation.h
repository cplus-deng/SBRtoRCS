#ifndef EXCITATION_INCLUDE
#define EXCITATION_INCLUDE
#include "TypeDef.hpp"
#include"BoundBox.hpp"
#include "ReducedBvhArray.hpp"

template<class T>
class Excitation 
{
public:
	bool init_;
	T freqHigh_;
	T freqLow_;
	std::shared_ptr<T> SeriesArray_;
	std::shared_ptr<T> DeriveSeriesArray_;
	U32 sampleNums_;
	T deltaTime_;

public:
	Excitation()
	{

	}
	Excitation(const T& freqHigh, const T& freqLow):
		freqHigh_(freqHigh),
		freqLow_(freqLow)
	{

	}
	~Excitation()
	{

	}

	void Reset()
	{
		if (init_)
		{
			init_ = false;
			freqHigh_ = 4E9;
			freqLow_ = 2E9;
			SeriesArray_.reset();
			DeriveSeriesArray_.reset();
		}
	}

	void Initialize(const ReducedBvhArray< T >& bvhArray)
	{
		Reset();

		T c0 = 299792458.0;
		T pi = 3.14159265359;
		T bandWidth = freqHigh_ - freqLow_;
		T tau = 1.6 / bandWidth;
		T t0 = 0.8 * tau;
		T w0 = pi * (freqHigh_ + freqLow_);

		BoundBox< T > boundBox = bvhArray.bvhNodeArray_.get()[0].data_.boundBox_;
		T objWidthMeter = boundBox.GetRadius() * 2;

		T traceTimeMax = 10 * objWidthMeter / c0; //10 : times limitation of reflection
		T sampleRate = 50 * freqHigh_;
		deltaTime_ = 1 / sampleRate;
		sampleNums_ = (U32)std::ceil(traceTimeMax / deltaTime_);

		DeriveSeriesArray_.reset(new T[sampleNums_], [](T* ptr) { delete[] ptr; });
		SeriesArray_.reset(new T[sampleNums_], [](T* ptr) { delete[] ptr; });

		T* st_dt = DeriveSeriesArray_.get();
		T* st = SeriesArray_.get();
		//time delay
		//in ray's PO
		for (U32 t = 0; t < sampleNums_; t++)
		{
			T pointSample = t * deltaTime_;
			T tmp = exp(-(4 * pi * pow(pointSample - t0, 2)) / pow(tau, 2));
			st[t] = -cos(w0 * pointSample) * tmp;
			st_dt[t] = tmp * (w0 * sin(w0 * pointSample) + cos(w0 * pointSample) * (pointSample - t0) * 8 * pi / pow(tau, 2));
			/*	
			temp1 = exp(-(4 * pi * (tt - t0) ^ 2) / (tao ^ 2));
			st(ii) = -cos(w0 * tt) * temp1;
			st_dt(ii) = temp1 * (w0 * sin(w0 * tt) + cos(w0 * tt) * (tt - t0) * 8 * pi / (tao ^ 2));*/
		}
		//array_view< T, 1 > seriesGpu(sampleNums, SeriesArray_);

		//parallel_for_each(seriesGpu.extent, [=](index< 1 > idSeries) restrict(amp)
		//	{
		//		T& st = seriesGpu[idSeries]; //Derivation
		//		T pointSample = idSeries * deltaTime;
		//		//temp1*(w0*sin(w0*tt)+cos(w0*tt)*(tt-t0)*8*pi/(tao^2))
		//		st = exp(-pow((4 * pi * (pointSample - t0)), 2) / pow(tau, 2))
		//			* (w0 * sin(w0 * pointSample) + cos(w0 * pointSample) * (pointSample - t0) * 8 * pi / pow(tau, 2));
		//	});
	}
	//init_ = true;

};
#endif // !EXCITATION_INCLUDE
