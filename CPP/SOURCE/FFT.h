#ifndef FFT_INCLUDE
#define FFT_INCLUDE

#include <complex> // std::complex
#include <vector>  // std::vector

namespace FFT 
{
	template<class T>
	using fft_arg = std::vector<std::complex<T>>;

	enum class fft_dir { DIR_FWD = +1, DIR_BWD = -1 };

    constexpr auto Pi = 3.141592653589793238462643383279502884;

    inline int findMSB(int x)
    {
        int p = 0;

        while (x > 1) {
            x >>= 1;
            ++p;
        }

        return p;
    }

    inline int bitr(uint32_t x, int nb)
    {
        x = (x << 16) | (x >> 16);
        x = ((x & 0x00FF00FF) << 8) | ((x & 0xFF00FF00) >> 8);
        x = ((x & 0x0F0F0F0F) << 4) | ((x & 0xF0F0F0F0) >> 4);
        x = ((x & 0x33333333) << 2) | ((x & 0xCCCCCCCC) >> 2);
        x = ((x & 0x55555555) << 1) | ((x & 0xAAAAAAAA) >> 1);

        return ((x >> (32 - nb)) & (0xFFFFFFFF >> (32 - nb)));
    }

    template <typename T> fft_arg<T> fft1d(const fft_arg<T>& xi, const fft_dir& dir)
    {
        int cnt = (int)xi.size();
        int msb = findMSB(cnt);
        T nrm = T(1) / std::sqrt(T(cnt));
        fft_arg<T> xo(cnt);

        for (int j = 0; j < cnt; ++j)
            xo[j] = nrm * xi[bitr(j, msb)];

        for (int i = 0; i < msb; ++i) {
            int bm = 1 << i; 
            int bw = 2 << i; 
            T ang = T(dir) * Pi / T(bm); 

            for (int j = 0; j < (cnt / 2); ++j) {
                int i1 = ((j >> i) << (i + 1)) + j % bm; 
                int i2 = i1 ^ bm;  
                std::complex<T> z1 = std::polar(T(1), ang * T(i1 ^ bw));
                std::complex<T> z2 = std::polar(T(1), ang * T(i2 ^ bw)); 
                std::complex<T> tmp = xo[i1];

                xo[i1] += z1 * xo[i2];
                xo[i2] = tmp + z2 * xo[i2];
            }
        }

        return xo;
    }
}
#endif // !FFT_INCLUDE

