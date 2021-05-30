#pragma once

#include <memory>
#include <math.h>
#include "guiconstants.h"

constexpr auto CH_LEFT = 0;
constexpr auto CH_RIGHT = 1;

const double kSmallestPositiveFloatValue = 1.175494351e-38;         /* min positive value */
const double kSmallestNegativeFloatValue = -1.175494351e-38;         /* min negative value */
const double kEuler = exp(1.0);
const double kEulerPlus1 = kEuler + 1.0;
const double kEulerMinus1 = kEuler - 1.0;
const double kPiHalf = kPi / 2.0;

inline bool checkFloatUnderflow(double& value)
{
	bool retValue = false;
	if (value > 0.0 && value < kSmallestPositiveFloatValue)
	{
		value = 0;
		retValue = true;
	}
	else if (value < 0.0 && value > kSmallestNegativeFloatValue)
	{
		value = 0;
		retValue = true;
	}
	return retValue;
}

inline double clip(double x)
{
	if (x > 0.0)
		return 1.0;
	if (x < 0.0)
		return -1.0;
	return 0.0;
}

inline double fastSigmoidSine(double x)
{
	double x_mod = -4.0 * fmod(0.25 * x, 1.0) + 2.0;
	double approx_1 = x_mod * (2.0 - fabs(x_mod));
	return 0.225 * (approx_1 * fabs(approx_1) - approx_1) + approx_1;
}

inline double ntsf(double x, double k, double m)
{
	double num = x - x * k;
	return num / (k - fabs(m * x) * 2.0 * k + 1.0);
}

const double ntsfn_m = -1.0 / 100.0;
inline double ntsfn(double x, double k_pc, double m_pc)
{
	if (k_pc == 0.0)
		return x;
	if (k_pc == 100.0)
		return clip(x);

	double k = ntsfn_m * k_pc;
	double m = fmin(m_pc / 100.0, 0.999);
	return ntsf(x, k, 1.0 / (1.0 - m));
}

const double ntsfp_m = 1.0 / 100.0;
inline double ntsfp(double x, double k_pc, double m_pc)
{
	if (k_pc == 0.0)
		return x;
	if (k_pc == 100.0)
		return 0.0;

	double k = ntsfp_m * k_pc;
	double m = fmin(m_pc / 100.0, 0.999);
	return ntsf(x, k, 1.0 - m);
}

inline double arry(double x)
{
	double fac_1 = (3.0 * x) / 2.0;
	double fac_2 = 1.0 - ((x * x) / 3.0);
	return fac_1 * fac_2;
}

inline double sig(double x, double k_pc)
{
	if (k_pc == 100.0)
		return clip(x);

	double k = 1.0 / (1.0 - k_pc / 100.0);
	return 2 * (1 / (1 + exp(-1.0 * k * x))) - 1.0;
}

inline double sig2(double x)
{
	double ex = exp(x);
	double num = (ex - 1.0) * kEulerPlus1;
	double denom = (ex + 1.0) * kEulerMinus1;
	return num / denom;
}

inline double tanh(double x, double k_pc)
{
	if (k_pc < 0.01)
		return x;

	return tanh(k_pc * x) / tanh(k_pc);
}

inline double atan(double x, double k_pc)
{
	if (k_pc < 0.01)
		return x;

	double k = 10.0 * k_pc;
	return atan(k * x) / atan(k);
}

inline double fexp1(double x, double k_pc)
{
	if (k_pc < 0.01)
		return x;

	double sign = clip(x);
	double num = 1 - exp(-1.0 * fabs(k_pc * x));
	double denom = 1 - exp(-1.0 * k_pc);
	return sign * (num / denom);
}

const double kTwoOverPi = 2.0 / kPi;
inline double sine(double x, double m_pc)
{
	double phaseShift = 0.0;
	if (m_pc > 0.0)
	{
		double fm = 4.0 * (m_pc / 100.0); // +2 octaves
		phaseShift = kTwoOverPi * fastSigmoidSine(fm * x);
	}
	return fastSigmoidSine(x + phaseShift);
}

inline double fexp2(double x)
{
	double sign = clip(-1.0 * x);
	double num = 1.0 - exp(fabs(x));
	double denom = kEuler - 1.0;
	return sign * (num / denom);
}

inline double expo2(double x)
{
	double num = kEuler - exp(1.0 - x);
	return num / kEulerMinus1;
}

inline double atsr(double x)
{
	double x_arg = 0.9 * x;
	double sum = atan(x_arg) + sqrt(1.0 - (x_arg * x_arg)) - 1.0;
	return 2.5 * sum;
}

inline double hclip(double x, double k)
{
	double cutoff = 1.0 - (k / 100.0);
	if (fabs(x) > cutoff)
		return cutoff * clip(x);

	return x;
}

inline double hwr(double x)
{
	return 0.5 * (x + fabs(x));
}

inline double fwr(double x)
{
	return fabs(x);
}

inline double sqs(double x)
{
	return x * x * clip(x);
}

inline double cube(double x)
{
	return x * x * x;
}

inline double sqr(double x)
{
	return x * x;
}

inline double asqrt(double x)
{
	return sqrt(fabs(x));
}


class DCFilter
{
public:
	DCFilter() {};
	~DCFilter() {};

	double processAudioSample(double x)
	{
		double y = x - xn1 + dcf_coeff * yn1;
		checkFloatUnderflow(y);
		xn1 = x;
		yn1 = y;
		return y;
	}

	void reset()
	{
		xn1 = 0.0;
		yn1 = 0.0;
	}

private:
	const double dcf_coeff = 0.995;

	double xn1 = 0.0;
	double yn1 = 0.0;
};

enum class waveshaperFunc { kNONE, kNTSFN, kARRY, kSIG, kSIG2, kTANH, kATAN, kFEXP1, kSIN, kNTSFP, kFEXP2, kEXP2, kATSR, kSQS, kCUBE, kHCLIP, kHWR, kFWR, kSQR, kASQRT };
enum class dcfStatus { kOFF, kON };

struct WaveshaperParameters
{
	WaveshaperParameters() {}

	WaveshaperParameters& operator=(const WaveshaperParameters& params)
	{
		if (this == &params)
			return *this;
		dcf_status = params.dcf_status;
		func = params.func;
		saturation_pct = params.saturation_pct;
		modifier_pct = params.modifier_pct;
		return *this;
	}

	dcfStatus dcf_status = dcfStatus::kOFF;
	waveshaperFunc func = waveshaperFunc::kNONE;
	double saturation_pct = 0.0;
	double modifier_pct = 0.0;
};

class WaveshaperBlock
{
public:
	WaveshaperBlock() {};
	~WaveshaperBlock() {};


	WaveshaperParameters getParameters() { return params; };

	double processAudioSample(double x, const int channel);

	void reset()
	{
		dcf_l.reset();
		dcf_r.reset();
	}

	void setParameters(const WaveshaperParameters& _params) { params = _params; };

private:
	inline double applyShaperFunc(double x);

	DCFilter dcf_l;
	DCFilter dcf_r;
	WaveshaperParameters params;
};

enum class wsBlockRouting { kSER, kPAR, kPAR_Y };

struct MultiWaveshaperParameters
{
	MultiWaveshaperParameters() {};

	MultiWaveshaperParameters& operator=(const MultiWaveshaperParameters& params)
	{
		if (this == &params)
			return *this;
		input_gain_dB = params.input_gain_dB;
		output_gain_dB = params.output_gain_dB;
		parallel_mix = params.parallel_mix;
		routing = params.routing;
		wsp_1 = params.wsp_1;
		wsp_2 = params.wsp_2;
		wsp_3 = params.wsp_3;
		return *this;
	}

	double input_gain_dB = 0.0;
	double output_gain_dB = 0.0;
	double parallel_mix = 50.0;
	wsBlockRouting routing = wsBlockRouting::kSER;

	WaveshaperParameters wsp_1;
	WaveshaperParameters wsp_2;
	WaveshaperParameters wsp_3;
};

class MultiWaveShaper
{
public:
	MultiWaveShaper() {};
	~MultiWaveShaper() {};

	MultiWaveshaperParameters getParameters() { return params; };

	double processAudioSample(double x, const int channel);

	bool processAudioFrame(const float* inputFrame,
		float* outputFrame,
		uint32_t inputChannels,
		uint32_t outputChannels);

	void reset()
	{
		wsb_1.reset();
		wsb_2.reset();
		wsb_3.reset();
	}

	void setParameters(const MultiWaveshaperParameters& _params) {
		if (params.input_gain_dB != _params.input_gain_dB)
		{
			input_gain_lin = pow(10.0, _params.input_gain_dB / 20.0);
		}
		if (params.output_gain_dB != _params.output_gain_dB)
		{
			output_gain_lin = pow(10.0, _params.output_gain_dB / 20.0);
		}
		if (params.parallel_mix != _params.parallel_mix)
		{
			double pm_normalized = _params.parallel_mix / 100.0;
			par_ratio_1 = (1.0 - pm_normalized);
			par_ratio_2 = pm_normalized;
		}
		params = _params;
		wsb_1.setParameters(params.wsp_1);
		wsb_2.setParameters(params.wsp_2);
		wsb_3.setParameters(params.wsp_3);
	};

private:
	double input_gain_lin = 1.0;
	double output_gain_lin = 1.0;
	double par_ratio_1 = 0.5;
	double par_ratio_2 = 0.5;
	MultiWaveshaperParameters params;

	WaveshaperBlock wsb_1;
	WaveshaperBlock wsb_2;
	WaveshaperBlock wsb_3;
};