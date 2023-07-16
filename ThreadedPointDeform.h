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
	ThreadedPointDeform(const Gdps &gdps,
						GA_SplittableRange *ptrange,
						DriveAttrib_Info *drive_attrib_hs,
						CaptureAttributes_Info *captureattribs_info,
						const UT_Array<UT_StringHolder> &attribnames_to_interpolate);

	struct TransformInfo
	{
		TransformInfo()
			: Pos(0.f)
			, WeightedPos(0.f)
			, Up(0.f)
			, PrimNormal(0.f)
			, PrimPosition(0.f, 0.f, 0.f)
			, Rot(1.f)
		{}

		UT_ValArray<int32>CapturePrims;
		UT_ValArray<fpreal16> CaptureUVWs;
		UT_ValArray<fpreal16> CaptureWeights;
		UT_Vector3F Pos;
		UT_Vector3F WeightedPos;
		UT_Vector3F Up;
		UT_Vector3F PrimNormal;
		UT_Vector4F PrimPosition;
		UT_Matrix3F Rot;
	};

	THREADED_METHOD1(ThreadedPointDeform, myPtRange->canMultiThread(), capture, GU_RayIntersect*, ray_gdp);
	void capturePartial(GU_RayIntersect *ray_gdp, const UT_JobInfo &info);

	THREADED_METHOD2(ThreadedPointDeform, myPtRange->canMultiThread(), captureByPieceAttrib, 
					 GA_ROHandleI, pieceattrib_h, MapRay<int32>, rest_prim_rays);
	void captureByPieceAttribPartial(GA_ROHandleI pieceattrib_h, MapRay<int32> rest_prim_rays, const UT_JobInfo &info);

	THREADED_METHOD2(ThreadedPointDeform, myPtRange->canMultiThread(), captureByPieceAttrib, 
					 GA_ROHandleS, pieceattrib_h, MapRay<UT_StringHolder>, rest_prim_rays);
	void captureByPieceAttribPartial(GA_ROHandleS pieceattrib_h, MapRay<UT_StringHolder> rest_prim_rays, const UT_JobInfo &info);

	THREADED_METHOD(ThreadedPointDeform, myPtRange->canMultiThread(), deform);
	void deformPartial(const UT_JobInfo &info);

private:
	void pointCapture(GU_RayIntersect *ray_gdp, GA_Offset ptoff);
	void buildXform(TransformInfo &trn_info, 
					const GU_Detail *gdp, 
					const GA_ROHandleV3 &normal_attrib_h, 
					const GA_ROHandleV3 &up_attrib_h);

private:
	const Gdps &myGdps;
	GA_SplittableRange *myPtRange;
	DriveAttrib_Info *myDriveAttribHs;
	CaptureAttributes_Info *myCaptureAttributes_Info;
	GA_ROHandleV3 myBasePh;
	GA_RWHandleV3 myPh;
	UT_Array<GA_ROHandleV3> myBasePtAttribsh;
	UT_Array<GA_RWHandleV3> myPtAttribsh;

};
}


#endif