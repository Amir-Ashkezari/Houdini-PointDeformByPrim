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
						const GU_Detail* restGdp,
						const GU_Detail* deformedGdp,
						const GA_SplittableRange& ptrange,
						HitAttributes& hitAttribs, 
						AttribsToInterpolate& attribsToInterpolate);

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

	THREADED_METHOD1(ThreadedPointDeform, m_ptrange.canMultiThread(), captureClosestPoint, GU_RayIntersect&, ray_gdp);
	void captureClosestPointPartial(GU_RayIntersect& ray_rest, const UT_JobInfo& info);

	THREADED_METHOD2(ThreadedPointDeform, m_ptrange.canMultiThread(), captureClosestPointByPieceAttrib, GA_ROHandleI, pieceAttrib_h, MapRay<int32>, restPrimRays);
	void captureClosestPointByPieceAttribPartial(GA_ROHandleI pieceAttrib_h, MapRay<int32> restPrimRays, const UT_JobInfo& info);

	THREADED_METHOD2(ThreadedPointDeform, m_ptrange.canMultiThread(), captureClosestPointByPieceAttrib, GA_ROHandleS, pieceAttrib_h, MapRay<UT_StringHolder>, restPrimRays);
	void captureClosestPointByPieceAttribPartial(GA_ROHandleS pieceAttrib_h, MapRay<UT_StringHolder> restPrimRays, const UT_JobInfo& info);

	THREADED_METHOD1(ThreadedPointDeform, m_ptrange.canMultiThread(), computeDeformation, const bool, rigidprojection);
	void computeDeformationPartial(const bool rigidprojection, const UT_JobInfo& info);

protected:
	void buildTransformationMatrix(TransformInfo&& trn_info);

private:
	GU_Detail* m_gdp = nullptr;
	const GU_Detail* m_baseGdp = nullptr;
	const GU_Detail* m_restGdp = nullptr;
	const GU_Detail* m_deformedGdp = nullptr;
	const GA_SplittableRange& m_ptrange;
	GA_ROHandleV3 m_baseP_h;
	GA_RWHandleV3 m_p_h;
	GA_RWHandleM3 m_restXform_h;
	GA_RWHandleT<UT_Vector3H> m_restP_h;
	GA_RWHandleI m_hitPrim_h;
	GA_RWHandleT<UT_Vector2H> m_hitUV_h;
	UT_Array<GA_ROHandleV3> m_basePtAttribs_h;
	UT_Array<GA_RWHandleV3> m_ptAttribs_h;
	UT_Array<GA_ROHandleV3> m_baseVtxAttribs_h;
	UT_Array<GA_RWHandleV3> m_vtxAttribs_h;

};
}


#endif