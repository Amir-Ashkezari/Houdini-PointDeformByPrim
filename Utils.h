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
	const GA_Attribute* BasePAttrib;
	GA_Attribute* PAttrib;
	UT_Array<const GA_Attribute*> BasePtAttribs;
	UT_Array<GA_Attribute*> PtAttribs;
	UT_Array<const GA_Attribute*> BaseVtxAttribs;
	UT_Array<GA_Attribute*> VtxAttribs;
};

struct HitAttributes
{
	GA_Attribute* Xform;
	GA_Attribute* RestP;
	GA_Attribute* Prim;
	GA_Attribute* UV;
};

struct DriveAttribs
{
	bool Drive;
	GA_Attribute* Normal;
	GA_Attribute* Up;
};

} // end AKA

#endif