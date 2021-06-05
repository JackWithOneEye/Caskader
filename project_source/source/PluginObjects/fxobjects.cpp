#include <memory>
#include <math.h>
#include "fxobjects.h"

inline double WaveshaperBlock::applyShaperFunc(double x)
{
	if (x == 0.0 || params.func == waveshaperFunc::kNONE)
		return x;

	switch (params.func)
	{
	case waveshaperFunc::kNTSFN:
		return ntsfn(x, params.saturation_pct, params.modifier_pct);
	case waveshaperFunc::kARRY:
		return arry(x);
	case waveshaperFunc::kSIG:
		return sig(x, params.saturation_pct);
	case waveshaperFunc::kSIG2:
		return sig2(x);
	case waveshaperFunc::kTANH:
		return tanh(x, params.saturation_pct);
	case waveshaperFunc::kATAN:
		return atan(x, params.saturation_pct);
	case waveshaperFunc::kFEXP1:
		return fexp1(x, params.saturation_pct);
	case waveshaperFunc::kTRI2SIN:
		return tri2sin(x, params.saturation_pct, params.modifier_pct);
	case waveshaperFunc::kNTSFP:
		return ntsfp(x, params.saturation_pct, params.modifier_pct);
	case waveshaperFunc::kFEXP2:
		return fexp2(x);
	case waveshaperFunc::kEXP2:
		return expo2(x);
	case waveshaperFunc::kATSR:
		return atsr(x);
	case waveshaperFunc::kSQS:
		return sqs(x);
	case waveshaperFunc::kCUBE:
		return cube(x);
	case waveshaperFunc::kHCLIP:
		return hclip(x, params.saturation_pct);
	case waveshaperFunc::kHWR:
		return hwr(x); // rectifier
	case waveshaperFunc::kFWR:
		return fwr(x); // rectifier
	case waveshaperFunc::kSQR:
		return sqr(x); // rectifier
	case waveshaperFunc::kASQRT:
		return asqrt(x); // rectifier
	case waveshaperFunc::kXROOT:
		return xroot(x, params.saturation_pct, params.modifier_pct);
	}
	return x;
}

double WaveshaperBlock::processAudioSample(double x, const int channel)
{
	double shaper_out = applyShaperFunc(x);

	if (params.dcf_status == dcfStatus::kON)
	{
		if (channel == CH_RIGHT)
			return dcf_r.processAudioSample(shaper_out);
		
		return dcf_l.processAudioSample(shaper_out);
	}

	return shaper_out;
}

double MultiWaveShaper::processAudioSample(double x, const int channel)
{
	double x_gain = x * input_gain_lin;
	double wsb_1_out = wsb_1.processAudioSample(x_gain, channel);
	wsBlockRouting routing = params.routing;
	double y = 0.0;
	if (routing == wsBlockRouting::kSER)
	{
		double wsb_2_out = wsb_2.processAudioSample(wsb_1_out, channel);
		y = wsb_3.processAudioSample(wsb_2_out, channel);
	}
	else if (routing == wsBlockRouting::kPAR)
	{
		double wsb_2_out = wsb_2.processAudioSample(x_gain, channel);
		double wsb_3_out = wsb_3.processAudioSample(wsb_2_out, channel);
		y = par_ratio_1 * wsb_1_out + par_ratio_2 * wsb_3_out;
	}
	else if (routing == wsBlockRouting::kPAR_Y)
	{
		double wsb_2_out = wsb_2.processAudioSample(wsb_1_out, channel);
		double wsb_3_out = wsb_3.processAudioSample(wsb_1_out, channel);
		y = par_ratio_1 * wsb_2_out + par_ratio_2 * wsb_3_out;
	}
	return y * output_gain_lin;
}

bool MultiWaveShaper::processAudioFrame(const float* inputFrame,
	float* outputFrame,
	uint32_t inputChannels,
	uint32_t outputChannels)
{
	if (inputChannels == 0 || outputChannels == 0)
		return false;

	if (params.routing != wsBlockRouting::kSER &&
		params.routing != wsBlockRouting::kPAR &&
		params.routing != wsBlockRouting::kPAR_Y)
		return false;

	// --- process left channel
	double ynL = processAudioSample(inputFrame[CH_LEFT], CH_LEFT);
	outputFrame[CH_LEFT] = ynL;
	if (outputChannels == 1)
		return true;
	
	if (inputChannels == 1)
	{
		outputFrame[CH_RIGHT] = ynL;
		return true;
	}

	// --- process right channel
	outputFrame[CH_RIGHT] = processAudioSample(inputFrame[CH_RIGHT], CH_RIGHT);

	return true;
}