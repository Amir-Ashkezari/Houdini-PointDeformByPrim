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

struct AttribsToInterpolate
{
	const GA_Attribute *OldPAttrib;
	GA_Attribute *PAttrib;
	UT_Array<const GA_Attribute*> BasePtAttribs;
	UT_Array<GA_Attribute*> PtAttribs;
};

struct CaptureAttributes
{
	bool MultipleSamples = false;
	fpreal32 MinDistThresh = 0.001f;
	GA_Attribute *RestP;
	GA_Attribute *Prims;
	GA_Attribute *UVWs;
	GA_Attribute *Weights;
};

struct DriveAttribHandles
{
	bool Drive = false;
	GA_ROHandleV3 RestNormal_H;
	GA_ROHandleV3 RestUp_H;
	GA_ROHandleV3 DeformedNormal_H;
	GA_ROHandleV3 DeformedUp_H;
};

} // end AKA

#endif