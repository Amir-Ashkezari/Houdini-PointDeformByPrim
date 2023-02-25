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
	ThreadedPointDeform(GU_Detail* gdp,
						const GU_Detail* baseGdp,
						const GU_Detail* rest_gdp,
						const GU_Detail* deformedGdp,
						const GA_SplittableRange& ptrange,
						HitAttributes& hit_attribs, 
						AttribsToInterpolate& attribs_to_interpolate);

	struct TransformInfo
	{
		TransformInfo()
			: hitprim(0)
			, hituv(0.f)
			, pos(0.f)
			, up(0.f)
			, primnml(0.f)
			, primpos(0.f, 0.f, 0.f)
			, rot(1.f)
		{}

		GA_Index hitprim;
		UT_Vector2H hituv;
		UT_Vector3H pos;
		UT_Vector3F up;
		UT_Vector3F primnml;
		UT_Vector4F primpos;
		UT_Matrix3F rot;
		const GEO_Primitive* geoprim;
	};

	THREADED_METHOD1(ThreadedPointDeform, myPtRange.canMultiThread(), captureClosestPoint, GU_RayIntersect&, ray_gdp);
	void captureClosestPointPartial(GU_RayIntersect& ray_rest, const UT_JobInfo& info);

	THREADED_METHOD2(ThreadedPointDeform, myPtRange.canMultiThread(), captureClosestPointByPieceAttrib, GA_ROHandleI, piece_attrib_h, MapRay<int32>, rest_prim_rays);
	void captureClosestPointByPieceAttribPartial(GA_ROHandleI piece_attrib_h, MapRay<int32> rest_prim_rays, const UT_JobInfo& info);

	THREADED_METHOD2(ThreadedPointDeform, myPtRange.canMultiThread(), captureClosestPointByPieceAttrib, GA_ROHandleS, piece_attrib_h, MapRay<UT_StringHolder>, rest_prim_rays);
	void captureClosestPointByPieceAttribPartial(GA_ROHandleS piece_attrib_h, MapRay<UT_StringHolder> rest_prim_rays, const UT_JobInfo& info);

	THREADED_METHOD1(ThreadedPointDeform, myPtRange.canMultiThread(), computeDeformation, const bool, rigid_projection);
	void computeDeformationPartial(const bool rigid_projection, const UT_JobInfo& info);

protected:
	void buildTransformationMatrix(TransformInfo& trn_info);

private:
	GU_Detail* myGdp = nullptr;
	const GU_Detail* myBaseGdp = nullptr;
	const GU_Detail* myRestGdp = nullptr;
	const GU_Detail* myDeformedGdp = nullptr;
	const GA_SplittableRange& myPtRange;
	GA_ROHandleV3 myBasePh;
	GA_RWHandleV3 myPh;
	GA_RWHandleM3 myRestXformh;
	GA_RWHandleT<UT_Vector3H> myRestPh;
	GA_RWHandleI myHitPrimh;
	GA_RWHandleT<UT_Vector2H> myHitUVh;
	UT_Array<GA_ROHandleV3> myBasePtAttribsh;
	UT_Array<GA_RWHandleV3> myPtAttribsh;
	UT_Array<GA_ROHandleV3> myBaseVtxAttribsh;
	UT_Array<GA_RWHandleV3> myVtxAttribsh;

};
}


#endif