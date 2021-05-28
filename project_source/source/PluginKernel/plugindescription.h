// --- CMAKE generated variables for your plugin

#include "pluginstructures.h"

#ifndef _plugindescription_h
#define _plugindescription_h

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#define AU_COCOA_VIEWFACTORY_STRING STR(AU_COCOA_VIEWFACTORY_NAME)
#define AU_COCOA_VIEW_STRING STR(AU_COCOA_VIEW_NAME)

// --- AU Plugin Cocoa View Names (flat namespace) 
#define AU_COCOA_VIEWFACTORY_NAME AUCocoaViewFactory_2D9361B922CC3A5D9F3D2EC2EB02BD1C
#define AU_COCOA_VIEW_NAME AUCocoaView_2D9361B922CC3A5D9F3D2EC2EB02BD1C

// --- BUNDLE IDs (MacOS Only) 
const char* kAAXBundleID = "developer.aax.caskader.bundleID";
const char* kAUBundleID = "developer.au.caskader.bundleID";
const char* kVST3BundleID = "developer.vst3.caskader.bundleID";

// --- Plugin Names 
const char* kPluginName = "Caskader";
const char* kShortPluginName = "Caskader";
const char* kAUBundleName = "Caskader_AU";

// --- Plugin Type 
const pluginType kPluginType = pluginType::kFXPlugin;

// --- VST3 UUID 
const char* kVSTFUID = "{2d9361b9-22cc-3a5d-9f3d-2ec2eb02bd1c}";

// --- 4-char codes 
const int32_t kFourCharCode = 'CASK';
const int32_t kAAXProductID = 'CASK';
const int32_t kManufacturerID = 'ASPK';

// --- Vendor information 
const char* kVendorName = "My Company";
const char* kVendorURL = "www.myplugins.com";
const char* kVendorEmail = "support@myplugins.com";

// --- Plugin Options 
const bool kWantSidechain = false;
const uint32_t kLatencyInSamples = 0;
const double kTailTimeMsec = 0.000000;
const bool kVSTInfiniteTail = false;
const bool kVSTSAA = false;
const uint32_t kVST3SAAGranularity = 1;
const uint32_t kAAXCategory = 0;

#endif
