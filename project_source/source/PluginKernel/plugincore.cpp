// -----------------------------------------------------------------------------
//    ASPiK Plugin Kernel File:  plugincore.cpp
//
/**
    \file   plugincore.cpp
    \author Will Pirkle
    \date   17-September-2018
    \brief  Implementation file for PluginCore object
    		- http://www.aspikplugins.com
    		- http://www.willpirkle.com
*/
// -----------------------------------------------------------------------------
#include "plugincore.h"
#include "plugindescription.h"

/**
\brief PluginCore constructor is launching pad for object initialization

Operations:
- initialize the plugin description (strings, codes, numbers, see initPluginDescriptors())
- setup the plugin's audio I/O channel support
- create the PluginParameter objects that represent the plugin parameters (see FX book if needed)
- create the presets
*/
PluginCore::PluginCore()
{
    // --- describe the plugin; call the helper to init the static parts you setup in plugindescription.h
    initPluginDescriptors();

    // --- default I/O combinations
	// --- for FX plugins
	if (getPluginType() == kFXPlugin)
	{
		addSupportedIOCombination({ kCFMono, kCFMono });
		addSupportedIOCombination({ kCFMono, kCFStereo });
		addSupportedIOCombination({ kCFStereo, kCFStereo });
	}
	else // --- synth plugins have no input, only output
	{
		addSupportedIOCombination({ kCFNone, kCFMono });
		addSupportedIOCombination({ kCFNone, kCFStereo });
	}

	// --- for sidechaining, we support mono and stereo inputs; auxOutputs reserved for future use
	addSupportedAuxIOCombination({ kCFMono, kCFNone });
	addSupportedAuxIOCombination({ kCFStereo, kCFNone });

	// --- create the parameters
    initPluginParameters();

    // --- create the presets
    initPluginPresets();
}

/**
\brief create all of your plugin parameters here

\return true if parameters were created, false if they already existed
*/
bool PluginCore::initPluginParameters()
{
	if (pluginParameterMap.size() > 0)
		return false;

    // --- Add your plugin parameter instantiation code bewtween these hex codes
	// **--0xDEA7--**

	PluginParameter* piParam = nullptr;

	piParam = new PluginParameter(controlID::input_gain, "Input Gain", "db", controlVariableType::kDouble, -60.000000, 20.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&input_gain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::output_gain, "Output Gain", "db", controlVariableType::kDouble, -60.000000, 20.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&output_gain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::shaper_1, "Shaper 1", "NONE,NTSFN,ARRY,SIG,SIG2,TANH,ATAN,FEXP1,NTSFP,FEXP2,EXP2,ATSR,SQS,CUBE,HCLIP,HWR,FWR,SQR,ASQRT", "NONE");
	piParam->setBoundVariable(&shaper_1, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::sat_1, "Saturation 1", "%", controlVariableType::kDouble, 0.000000, 100.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&sat_1, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::mod_1, "Modifier1", "%", controlVariableType::kDouble, 0.000000, 100.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&modifier_1, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::shaper_2, "Shaper 2", "NONE,NTSFN,ARRY,SIG,SIG2,TANH,ATAN,FEXP1,NTSFP,FEXP2,EXP2,ATSR,SQS,CUBE,HCLIP,HWR,FWR,SQR,ASQRT", "NONE");
	piParam->setBoundVariable(&shaper_2, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::sat_2, "Saturation 2", "%", controlVariableType::kDouble, 0.000000, 100.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&sat_2, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::mod_2, "Modifier2", "%", controlVariableType::kDouble, 0.000000, 100.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&modifier_2, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::shaper_3, "Shaper 3", "NONE,NTSFN,ARRY,SIG,SIG2,TANH,ATAN,FEXP1,NTSFP,FEXP2,EXP2,ATSR,SQS,CUBE,HCLIP,HWR,FWR,SQR,ASQRT", "NONE");
	piParam->setBoundVariable(&shaper_3, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::sat_3, "Saturation 3", "%", controlVariableType::kDouble, 0.000000, 100.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&sat_3, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::mod_3, "Modifier3", "%", controlVariableType::kDouble, 0.000000, 100.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&modifier_3, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::routing, "Shaper Routing", "SER,PAR,PAR_Y", "SER");
	piParam->setBoundVariable(&routing, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::parallel_ratio, "Parallel Branches Ratio", "%", controlVariableType::kDouble, 0.000000, 100.000000, 50.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&parallel_ratio, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::dc_filter, "DC Filter On/Off", "OFF, ON", "OFF");
	piParam->setBoundVariable(&dc_filter, boundVariableType::kInt);
	addPluginParameter(piParam);
    
	// **--0xEDA5--**
   
    // --- BONUS Parameter
    // --- SCALE_GUI_SIZE
    PluginParameter* piParamBonus = new PluginParameter(SCALE_GUI_SIZE, "Scale GUI", "tiny,small,medium,normal,large,giant", "normal");
    addPluginParameter(piParamBonus);

	// --- create the super fast access array
	initPluginParameterArray();

    return true;
}

inline bool checkFloatUnderflow(double& value)
{
	bool retValue = false;
	if (value > 0.0 && value < 1.175494351e-38)
	{
		value = 0;
		retValue = true;
	}
	else if (value < 0.0 && value > -1.175494351e-38)
	{
		value = 0;
		retValue = true;
	}
	return retValue;
}

const double kEuler = std::exp(1.0);
const double kEulerPlus1 = kEuler + 1.0;
const double kEulerMinus1 = kEuler - 1.0;

inline double clip(double x)
{
	if (x > 0.0)
		return 1.0;
	if (x < 0.0)
		return -1.0;
	return 0.0;
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
	return 2 * (1 / (1 + std::exp(-1.0 * k * x))) - 1.0;
}

inline double sig2(double x)
{
	double ex = std::exp(x);
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
	double num = 1 - std::exp(-1.0 * fabs(k_pc * x));
	double denom = 1 - std::exp(-1.0 * k_pc);
	return sign * (num / denom);
}

inline double fexp2(double x)
{
	double sign = clip(-1.0 * x);
	double num = 1.0 - std::exp(fabs(x));
	double denom = kEuler - 1.0;
	return sign * (num / denom);
}

inline double expo2(double x)
{
	double num = kEuler - std::exp(1.0 - x);
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
	double cutoff = k / 100.0;
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

inline double PluginCore::applyShaper(double x, int shaper, double saturation, double modifier)
{
	if (x == 0.0)
		return 0.0;

	if (compareEnumToInt(shaper_Enum::NONE, shaper))
		return x;

	if (compareEnumToInt(shaper_Enum::NTSFN, shaper))
		return ntsfn(x, saturation, modifier);

	if (compareEnumToInt(shaper_Enum::ARRY, shaper))
		return arry(x);

	if (compareEnumToInt(shaper_Enum::SIG, shaper))
		return sig(x, saturation);

	if (compareEnumToInt(shaper_Enum::SIG2, shaper))
		return sig2(x);

	if (compareEnumToInt(shaper_Enum::TANH, shaper))
		return tanh(x, saturation);

	if (compareEnumToInt(shaper_Enum::ATAN, shaper))
		return atan(x, saturation);

	if (compareEnumToInt(shaper_Enum::FEXP1, shaper))
		return fexp1(x, saturation);

	if (compareEnumToInt(shaper_Enum::NTSFP, shaper))
		return ntsfp(x, saturation, modifier);

	if (compareEnumToInt(shaper_Enum::FEXP2, shaper))
		return fexp2(x);

	if (compareEnumToInt(shaper_Enum::EXP2, shaper))
		return expo2(x);

	if (compareEnumToInt(shaper_Enum::ATSR, shaper))
		return atsr(x);

	if (compareEnumToInt(shaper_Enum::SQS, shaper))
		return sqs(x);

	if (compareEnumToInt(shaper_Enum::CUBE, shaper))
		return cube(x);

	if (compareEnumToInt(shaper_Enum::HCLIP, shaper))
		return hclip(x, saturation);

	if (compareEnumToInt(shaper_Enum::HWR, shaper))
		return hwr(x); // rectifier

	if (compareEnumToInt(shaper_Enum::FWR, shaper))
		return fwr(x); // rectifier

	if (compareEnumToInt(shaper_Enum::SQR, shaper))
		return sqr(x); // rectifier

	if (compareEnumToInt(shaper_Enum::ASQRT, shaper))
		return asqrt(x); // rectifier

	return x;
}

inline double calcParallelMix(double b1, double b2, double ratio)
{
	double r_normalized = ratio / 100.0;
	return (1.0 - r_normalized) * b1 + r_normalized * b2;
}

inline double PluginCore::applyDCFilter(double x, double& x_1, double& y_1)
{
	double y_n = x - x_1 + dc_coeff * y_1;
	checkFloatUnderflow(y_n);
	x_1 = x;
	y_1 = y_n;
	return y_n;
}

inline double PluginCore::processInput(double x)
{
	double x_gain = x * pow(10.0, input_gain / 20.0);
	double y = applyShaper(x_gain, shaper_1, sat_1, modifier_1);
	if (compareEnumToInt(routing_Enum::SER, routing))
	{
		y = applyShaper(y, shaper_2, sat_2, modifier_2);
		y = applyShaper(y, shaper_3, sat_3, modifier_3);
	}
	else if (compareEnumToInt(routing_Enum::PAR, routing))
	{
		double yn_L23 = applyShaper(x_gain, shaper_2, sat_2, modifier_2);
		yn_L23 = applyShaper(yn_L23, shaper_3, sat_3, modifier_3);
		y = calcParallelMix(y, yn_L23, parallel_ratio);
	}
	else if (compareEnumToInt(routing_Enum::PAR_Y, routing))
	{
		double yn_L12 = applyShaper(y, shaper_2, sat_2, modifier_2);
		double yn_L13 = applyShaper(y, shaper_3, sat_3, modifier_3);
		y = calcParallelMix(yn_L12, yn_L13, parallel_ratio);
	}
	double dc_filtered_y = applyDCFilter(y, dc_x_1.first, dc_y_1.first);
	if (compareEnumToInt(dc_filterEnum::ON, dc_filter))
	{
		y = dc_filtered_y;
	}
	return y * pow(10.0, output_gain / 20.0);
}

/**
\brief initialize object for a new run of audio; called just before audio streams

Operation:
- store sample rate and bit depth on audioProcDescriptor - this information is globally available to all core functions
- reset your member objects here

\param resetInfo structure of information about current audio format

\return true if operation succeeds, false otherwise
*/
bool PluginCore::reset(ResetInfo& resetInfo)
{
    // --- save for audio processing
    audioProcDescriptor.sampleRate = resetInfo.sampleRate;
    audioProcDescriptor.bitDepth = resetInfo.bitDepth;

	dc_x_1 = std::make_pair(0.0, 0.0);
	dc_y_1 = std::make_pair(0.0, 0.0);

    // --- other reset inits
    return PluginBase::reset(resetInfo);
}

/**
\brief one-time initialize function called after object creation and before the first reset( ) call

Operation:
- saves structure for the plugin to use; you can also load WAV files or state information here
*/
bool PluginCore::initialize(PluginInfo& pluginInfo)
{
	// --- add one-time init stuff here

	return true;
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- syncInBoundVariables when preProcessAudioBuffers is called, it is *guaranteed* that all GUI control change information
  has been applied to plugin parameters; this binds parameter changes to your underlying variables
- NOTE: postUpdatePluginParameter( ) will be called for all bound variables that are acutally updated; if you need to process
  them individually, do so in that function
- use this function to bulk-transfer the bound variable data into your plugin's member object variables

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::preProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
    // --- sync internal variables to GUI parameters; you can also do this manually if you don't
    //     want to use the auto-variable-binding
    syncInBoundVariables();

    return true;
}

/**
\brief frame-processing method

Operation:
- decode the plugin type - for synth plugins, fill in the rendering code; for FX plugins, delete the if(synth) portion and add your processing code
- note that MIDI events are fired for each sample interval so that MIDI is tightly sunk with audio
- doSampleAccurateParameterUpdates will perform per-sample interval smoothing

\param processFrameInfo structure of information about *frame* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processAudioFrame(ProcessFrameInfo& processFrameInfo)
{
	processFrameInfo.midiEventQueue->fireMidiEvents(processFrameInfo.currentFrame);
	doSampleAccurateParameterUpdates();
	if (getPluginType() == kSynthPlugin ||
		processFrameInfo.numAudioInChannels == 0 ||
		processFrameInfo.numAudioOutChannels == 0)
		return false;

	double ynL = processInput(processFrameInfo.audioInputFrame[0]);

    // --- FX Plugin:
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFMono)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = ynL;

        return true; /// processed
    }

    // --- Mono-In/Stereo-Out
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = ynL;
        processFrameInfo.audioOutputFrame[1] = ynL;

        return true; /// processed
    }

	double ynR = processInput(processFrameInfo.audioInputFrame[1]);

    // --- Stereo-In/Stereo-Out
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFStereo &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = ynL;
        processFrameInfo.audioOutputFrame[1] = ynR;

        return true; /// processed
    }

    return false; /// NOT processed
}


/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- updateOutBoundVariables sends metering data to the GUI meters

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
	// --- update outbound variables; currently this is meter data only, but could be extended
	//     in the future
	updateOutBoundVariables();

    return true;
}

/**
\brief update the PluginParameter's value based on GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- use base class helper
    setPIParamValue(controlID, controlValue);

    // --- do any post-processing
    postUpdatePluginParameter(controlID, controlValue, paramInfo);

    return true; /// handled
}

/**
\brief update the PluginParameter's value based on *normlaized* GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param normalizedValue the new control value in normalized form
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameterNormalized(int32_t controlID, double normalizedValue, ParameterUpdateInfo& paramInfo)
{
	// --- use base class helper, returns actual value
	double controlValue = setPIParamValueNormalized(controlID, normalizedValue, paramInfo.applyTaper);

	// --- do any post-processing
	postUpdatePluginParameter(controlID, controlValue, paramInfo);

	return true; /// handled
}

/**
\brief perform any operations after the plugin parameter has been updated; this is one paradigm for
	   transferring control information into vital plugin variables or member objects. If you use this
	   method you can decode the control ID and then do any cooking that is needed. NOTE: do not
	   overwrite bound variables here - this is ONLY for any extra cooking that is required to convert
	   the GUI data to meaninful coefficients or other specific modifiers.

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postUpdatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- now do any post update cooking; be careful with VST Sample Accurate automation
    //     If enabled, then make sure the cooking functions are short and efficient otherwise disable it
    //     for the Parameter involved
    /*switch(controlID)
    {
        case 0:
        {
            return true;    /// handled
        }

        default:
            return false;   /// not handled
    }*/

    return false;
}

/**
\brief has nothing to do with actual variable or updated variable (binding)

CAUTION:
- DO NOT update underlying variables here - this is only for sending GUI updates or letting you
  know that a parameter was changed; it should not change the state of your plugin.

WARNING:
- THIS IS NOT THE PREFERRED WAY TO LINK OR COMBINE CONTROLS TOGETHER. THE PROPER METHOD IS
  TO USE A CUSTOM SUB-CONTROLLER THAT IS PART OF THE GUI OBJECT AND CODE.
  SEE http://www.willpirkle.com for more information

\param controlID the control ID value of the parameter being updated
\param actualValue the new control value

\return true if operation succeeds, false otherwise
*/
bool PluginCore::guiParameterChanged(int32_t controlID, double actualValue)
{
	/*
	switch (controlID)
	{
		case controlID::<your control here>
		{

			return true; // handled
		}

		default:
			break;
	}*/

	return false; /// not handled
}

/**
\brief For Custom View and Custom Sub-Controller Operations

NOTES:
- this is for advanced users only to implement custom view and custom sub-controllers
- see the SDK for examples of use

\param messageInfo a structure containing information about the incoming message

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMessage(MessageInfo& messageInfo)
{
	// --- decode message
	switch (messageInfo.message)
	{
		// --- add customization appearance here
	case PLUGINGUI_DIDOPEN:
	{
		return false;
	}

	// --- NULL pointers so that we don't accidentally use them
	case PLUGINGUI_WILLCLOSE:
	{
		return false;
	}

	// --- update view; this will only be called if the GUI is actually open
	case PLUGINGUI_TIMERPING:
	{
		return false;
	}

	// --- register the custom view, grab the ICustomView interface
	case PLUGINGUI_REGISTER_CUSTOMVIEW:
	{

		return false;
	}

	case PLUGINGUI_REGISTER_SUBCONTROLLER:
	case PLUGINGUI_QUERY_HASUSERCUSTOM:
	case PLUGINGUI_USER_CUSTOMOPEN:
	case PLUGINGUI_USER_CUSTOMCLOSE:
	case PLUGINGUI_EXTERNAL_SET_NORMVALUE:
	case PLUGINGUI_EXTERNAL_SET_ACTUALVALUE:
	{

		return false;
	}

	default:
		break;
	}

	return false; /// not handled
}


/**
\brief process a MIDI event

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param event a structure containing the MIDI event data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMIDIEvent(midiEvent& event)
{
	return true;
}

/**
\brief (for future use)

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param vectorJoysickData a structure containing joystick data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::setVectorJoystickParameters(const VectorJoystickData& vectorJoysickData)
{
	return true;
}

/**
\brief use this method to add new presets to the list

NOTES:
- see the SDK for examples of use
- for non RackAFX users that have large paramter counts, there is a secret GUI control you
  can enable to write C++ code into text files, one per preset. See the SDK or http://www.willpirkle.com for details

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginPresets()
{
	// **--0xFF7A--**

	// **--0xA7FF--**

    return true;
}

/**
\brief setup the plugin description strings, flags and codes; this is ordinarily done through the ASPiKreator or CMake

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginDescriptors()
{
    pluginDescriptor.pluginName = PluginCore::getPluginName();
    pluginDescriptor.shortPluginName = PluginCore::getShortPluginName();
    pluginDescriptor.vendorName = PluginCore::getVendorName();
    pluginDescriptor.pluginTypeCode = PluginCore::getPluginType();

	// --- describe the plugin attributes; set according to your needs
	pluginDescriptor.hasSidechain = kWantSidechain;
	pluginDescriptor.latencyInSamples = kLatencyInSamples;
	pluginDescriptor.tailTimeInMSec = kTailTimeMsec;
	pluginDescriptor.infiniteTailVST3 = kVSTInfiniteTail;

    // --- AAX
    apiSpecificInfo.aaxManufacturerID = kManufacturerID;
    apiSpecificInfo.aaxProductID = kAAXProductID;
    apiSpecificInfo.aaxBundleID = kAAXBundleID;  /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.aaxEffectID = "aaxDeveloper.";
    apiSpecificInfo.aaxEffectID.append(PluginCore::getPluginName());
    apiSpecificInfo.aaxPluginCategoryCode = kAAXCategory;

    // --- AU
    apiSpecificInfo.auBundleID = kAUBundleID;   /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.auBundleName = kAUBundleName;

    // --- VST3
    apiSpecificInfo.vst3FUID = PluginCore::getVSTFUID(); // OLE string format
    apiSpecificInfo.vst3BundleID = kVST3BundleID;/* MacOS only: this MUST match the bundle identifier in your info.plist file */
	apiSpecificInfo.enableVST3SampleAccurateAutomation = kVSTSAA;
	apiSpecificInfo.vst3SampleAccurateGranularity = kVST3SAAGranularity;

    // --- AU and AAX
    apiSpecificInfo.fourCharCode = PluginCore::getFourCharCode();

    return true;
}

// --- static functions required for VST3/AU only --------------------------------------------- //
const char* PluginCore::getPluginBundleName() { return kAUBundleName; }
const char* PluginCore::getPluginName(){ return kPluginName; }
const char* PluginCore::getShortPluginName(){ return kShortPluginName; }
const char* PluginCore::getVendorName(){ return kVendorName; }
const char* PluginCore::getVendorURL(){ return kVendorURL; }
const char* PluginCore::getVendorEmail(){ return kVendorEmail; }
const char* PluginCore::getAUCocoaViewFactoryName(){ return AU_COCOA_VIEWFACTORY_STRING; }
pluginType PluginCore::getPluginType(){ return kPluginType; }
const char* PluginCore::getVSTFUID(){ return kVSTFUID; }
int32_t PluginCore::getFourCharCode(){ return kFourCharCode; }
