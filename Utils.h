#pragma once

#ifndef __Utils_h__
#define __Utils_h__

#include <UT/UT_Map.h>
#include <UT/UT_String.h>

class GA_ElementGroup;
class GU_RayIntersect;

namespace AKA
{

template<typename T>
struct MapPrimGroup
{
	UT_Map<T, GA_PrimitiveGroup*> Map;
};

template<typename T>
struct MapRay
{
	UT_Map<T, GU_RayIntersect*> Map;
};

struct Gdps
{
	GU_Detail *Gdp = nullptr;
	const GU_Detail *BaseGdp = nullptr;
	const GU_Detail *RestGdp = nullptr;
	const GU_Detail *DeformedGdp = nullptr;
};

struct CaptureAttributes
{
	GA_Attribute *RestP = nullptr;
	GA_Attribute *Prims = nullptr;
	GA_Attribute *UVWs = nullptr;
	GA_Attribute *Weights = nullptr;
	GA_Attribute *Xform = nullptr;
};

struct CaptureAttributes_Info
{
	bool CaptureMultiSamples = false;
	fpreal32 CaptureMinDistThresh = 0.001f;
	GA_RWHandleV3 RestP_H;
	GA_RWHandleT<UT_ValArray<int32>> CapturePrims_H;
	GA_RWHandleT<UT_ValArray<fpreal16>> CaptureUVWs_H;
	GA_RWHandleT<UT_ValArray<fpreal16>> CaptureWeights_H;
	bool XformRequired = false;
	GA_RWHandleM3 Xform_H;
};

struct DriveAttrib_Info
{
	bool Drive = false;
	GA_ROHandleV3 RestNormal_H;
	GA_ROHandleV3 RestUp_H;
	GA_ROHandleV3 DeformedNormal_H;
	GA_ROHandleV3 DeformedUp_H;
};

} // end AKA

#endif