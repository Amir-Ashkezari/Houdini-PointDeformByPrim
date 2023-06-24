#pragma once

#ifndef __ThreadedPointDeform_h__
#define __ThreadedPointDeform_h__

#include <GA/GA_SplittableRange.h>
#include <GA/GA_PageIterator.h>
#include <GA/GA_PageHandle.h>
#include <GU/GU_RayIntersect.h>
#include "Utils.h"

class GU_Detail;
class GU_RayIntersect;

namespace AKA
{

class ThreadedPointDeform
{
public:
	ThreadedPointDeform(GU_Detail *gdp,
						const GU_Detail *baseGdp,
						const GU_Detail *rest_gdp,
						const GU_Detail *deformedGdp,
						GA_SplittableRange &&ptrange,
						DriveAttribHandles &&drive_attrib_hs,
						CaptureAttributes &capture_attribs, 
						AttribsToInterpolate &attribs_to_interpolate);

	struct TransformInfo
	{
		TransformInfo()
			: Pos(0.f)
			, Up(0.f)
			, PrimNormal(0.f)
			, PrimPosition(0.f, 0.f, 0.f)
			, Rot(1.f)
		{}

		UT_ValArray<int32> CapturePrims;
		UT_ValArray<fpreal16> CaptureUVWs;
		UT_ValArray<fpreal16> CaptureWeights;
		UT_Vector3F Pos;
		UT_Vector3F Up;
		UT_Vector3F PrimNormal;
		UT_Vector4F PrimPosition;
		UT_Matrix3F Rot;
		const GEO_Primitive *GeomPrim;
	};

	THREADED_METHOD1(ThreadedPointDeform, myPtRange.canMultiThread(), captureClosestPoint, GU_RayIntersect&, ray_gdp);
	void captureClosestPointPartial(GU_RayIntersect &ray_rest, const UT_JobInfo &info);

	THREADED_METHOD2(ThreadedPointDeform, myPtRange.canMultiThread(), captureClosestPointByPieceAttrib, GA_ROHandleI, piece_attrib_h, MapRay<int32>, rest_prim_rays);
	void captureClosestPointByPieceAttribPartial(GA_ROHandleI piece_attrib_h, MapRay<int32> rest_prim_rays, const UT_JobInfo &info);

	THREADED_METHOD2(ThreadedPointDeform, myPtRange.canMultiThread(), captureClosestPointByPieceAttrib, GA_ROHandleS, piece_attrib_h, MapRay<UT_StringHolder>, rest_prim_rays);
	void captureClosestPointByPieceAttribPartial(GA_ROHandleS piece_attrib_h, MapRay<UT_StringHolder> rest_prim_rays, const UT_JobInfo &info);

	THREADED_METHOD(ThreadedPointDeform, myPtRange.canMultiThread(), computeDeformation);
	void computeDeformationPartial(const UT_JobInfo &info);

protected:
	void buildXformByPrimIntrinsic(TransformInfo &trn_info, int32 idx);
	void buildXformByAttribute(TransformInfo &trn_info,
							   const GU_Detail *gdp,
							   const GA_ROHandleV3 &normal_attrib_h,
							   const GA_ROHandleV3 &up_attrib_h,
							   int32 idx);

private:
	GU_Detail *myGdp = nullptr;
	const GU_Detail *myBaseGdp = nullptr;
	const GU_Detail *myRestGdp = nullptr;
	const GU_Detail *myDeformedGdp = nullptr;
	GA_SplittableRange &&myPtRange;
	DriveAttribHandles &&myDriveAttribHs;
	GA_ROHandleV3 myBasePh;
	GA_RWHandleV3 myPh;
	GA_RWHandleV3 myRestPh;
	const bool myCaptureMultiSamples;
	const fpreal32 myCaptureMinDistThresh;
	GA_RWHandleT<UT_ValArray<int32>> myCapturePrimsh;
	GA_RWHandleT<UT_ValArray<fpreal16>> myCaptureUVWsh;
	GA_RWHandleT<UT_ValArray<fpreal16>> myCaptureWeightsh;
	UT_Array<GA_ROHandleV3> myBasePtAttribsh;
	UT_Array<GA_RWHandleV3> myPtAttribsh;
	UT_Array<GA_ROHandleV3> myBaseVtxAttribsh;
	UT_Array<GA_RWHandleV3> myVtxAttribsh;

};
}


#endif