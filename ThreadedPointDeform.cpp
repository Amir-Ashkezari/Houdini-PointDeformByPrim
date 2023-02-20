#include <GU/GU_Detail.h>
#include <GU/GU_RayIntersect.h>
#include <UT/UT_Assert.h>

#include "ThreadedPointDeform.h"
#include <iostream>

using namespace AKA;

ThreadedPointDeform::ThreadedPointDeform(GU_Detail* gdp,
										 const GU_Detail* base_gdp,
										 const GU_Detail* rest_gdp,
										 const GU_Detail* deformed_gdp,
										 const GA_SplittableRange& ptrange,
										 HitAttributes& hit_attribs,
										 AttribsToInterpolate& attribs_to_interpolate)
	: myGdp(gdp)
	, myBaseGdp(base_gdp)
	, myRestGdp(rest_gdp)
	, myDeformedGdp(deformed_gdp)
	, myPtRange(ptrange)
	, myBasePh(attribs_to_interpolate.BasePAttrib)
	, myPh(attribs_to_interpolate.PAttrib)
	, myRestXformh(hit_attribs.Xform)
	, myRestPh(hit_attribs.RestP)
	, myHitPrimh(hit_attribs.Prim)
	, myHitUVh(hit_attribs.UV)
{
	for (const GA_Attribute* basePtAttrib : attribs_to_interpolate.BasePtAttribs)
		myBasePtAttribsh.emplace_back(basePtAttrib);

	for (GA_Attribute* ptAttrib : attribs_to_interpolate.PtAttribs)
		myPtAttribsh.emplace_back(ptAttrib);

	for (const GA_Attribute* baseVtxAttrib : attribs_to_interpolate.BaseVtxAttribs)
		myBaseVtxAttribsh.emplace_back(baseVtxAttrib);

	for (GA_Attribute* vtxAttrib : attribs_to_interpolate.VtxAttribs)
		myVtxAttribsh.emplace_back(vtxAttrib);
}

void
ThreadedPointDeform::captureClosestPointPartial(GU_RayIntersect& ray_gdp, const UT_JobInfo& info)
{
	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			GU_MinInfo min_info(1e+3F, 1e-2F, 1);

			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				ray_gdp.minimumPoint(myBasePh.get(ptoff), min_info);

				trn_info.pos = myBasePh.get(ptoff);
				trn_info.hituv = { min_info.u1, min_info.v1 };
				trn_info.geoprim = min_info.prim;

				buildTransformationMatrix(std::move(trn_info));
				trn_info.rot.invert();

				trn_info.pos -= UTverify_cast<UT_Vector3F>(trn_info.primpos);
				trn_info.pos.rowVecMult(trn_info.rot);

				myRestXformh.set(ptoff, trn_info.rot);
				myRestPh.set(ptoff, trn_info.pos);
				myHitPrimh.set(ptoff, min_info.prim->getMapIndex());
				myHitUVh.set(ptoff, trn_info.hituv);
			}
		}
	}
}

void
ThreadedPointDeform::captureClosestPointByPieceAttribPartial(GA_ROHandleI piece_attrib_h,
															 MapRay<int32> rest_prim_rays,
															 const UT_JobInfo& info)
{
	const GA_Attribute* pieceAttrib = piece_attrib_h.getAttribute();
	const GA_AttributeOwner& pieceAttribOwner = pieceAttrib->getOwner();

	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			GU_MinInfo min_info(1e+3F, 1e-2F, 1);
			int32 piece_attrib_val;
			GA_OffsetArray prims;

			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				if (pieceAttribOwner == GA_ATTRIB_PRIMITIVE)
				{
					myGdp->getPrimitivesReferencingPoint(prims, ptoff);
					piece_attrib_val = piece_attrib_h.get(prims[0]);
				}
				else
					piece_attrib_val = piece_attrib_h.get(ptoff);

				if (rest_prim_rays.Map.find(piece_attrib_val) == rest_prim_rays.Map.end())
					continue;

				rest_prim_rays.Map.at(piece_attrib_val)->minimumPoint(myPh.get(ptoff), min_info);

				trn_info.pos = myBasePh.get(ptoff);
				trn_info.hituv = { min_info.u1, min_info.v1 };
				trn_info.geoprim = min_info.prim;

				buildTransformationMatrix(std::move(trn_info));
				trn_info.rot.invert();
				
				trn_info.pos -= UTverify_cast<UT_Vector3F>(trn_info.primpos);
				trn_info.pos.rowVecMult(trn_info.rot);
				
				myRestXformh.set(ptoff, trn_info.rot);
				myRestPh.set(ptoff, trn_info.pos);
				myHitPrimh.set(ptoff, min_info.prim->getMapIndex());
				myHitUVh.set(ptoff, trn_info.hituv);
			}
		}
	}
}

void
ThreadedPointDeform::captureClosestPointByPieceAttribPartial(GA_ROHandleS piece_attrib_h,
															 MapRay<UT_StringHolder> rest_prim_rays,
															 const UT_JobInfo& info)
{
	const GA_Attribute* pieceAttrib = piece_attrib_h.getAttribute();
	const GA_AttributeOwner& pieceAttribOwner = pieceAttrib->getOwner();

	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			GU_MinInfo min_info(1e+3F, 1e-2F, 1);
			UT_StringHolder piece_attrib_val;
			GA_OffsetArray prims;

			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				if (pieceAttribOwner == GA_ATTRIB_PRIMITIVE)
				{
					myGdp->getPrimitivesReferencingPoint(prims, ptoff);
					piece_attrib_val = piece_attrib_h.get(prims[0]);
				}
				else
					piece_attrib_val = piece_attrib_h.get(ptoff);

				if (rest_prim_rays.Map.find(piece_attrib_val) == rest_prim_rays.Map.end())
					continue;

				rest_prim_rays.Map.at(piece_attrib_val)->minimumPoint(myPh.get(ptoff), min_info);

				trn_info.pos = myBasePh.get(ptoff);
				trn_info.hituv = { min_info.u1, min_info.v1 };
				trn_info.geoprim = min_info.prim;

				buildTransformationMatrix(std::move(trn_info));
				trn_info.rot.invert();
				
				trn_info.pos -= UTverify_cast<UT_Vector3F>(trn_info.primpos);
				trn_info.pos.rowVecMult(trn_info.rot);
				
				myRestXformh.set(ptoff, trn_info.rot);
				myRestPh.set(ptoff, trn_info.pos);
				myHitPrimh.set(ptoff, min_info.prim->getMapIndex());
				myHitUVh.set(ptoff, trn_info.hituv);
			}
		}
	}
}

void
ThreadedPointDeform::computeDeformationPartial(const bool rigid_projection, const UT_JobInfo& info)
{
	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				trn_info.pos = myRestPh.get(ptoff);
				trn_info.hitprim = myHitPrimh.get(ptoff);
				trn_info.hituv = myHitUVh.get(ptoff);

				const GA_IndexMap& prim_map = myDeformedGdp->getIndexMap(GA_ATTRIB_PRIMITIVE);
				trn_info.geoprim = myDeformedGdp->getGEOPrimitive(prim_map.offsetFromIndex(trn_info.hitprim));

				buildTransformationMatrix(std::move(trn_info));

				UT_Vector3 vecAttrib;
				for (size_t idx = 0; idx < myBasePtAttribsh.size(); ++idx)
				{
					vecAttrib = myBasePtAttribsh[idx].get(ptoff);
					vecAttrib.rowVecMult(myRestXformh.get(ptoff));
					vecAttrib.rowVecMult(trn_info.rot);

					myPtAttribsh[idx].set(ptoff, vecAttrib);
				}
				
				trn_info.pos.rowVecMult(trn_info.rot);
				trn_info.pos += UTverify_cast<UT_Vector3F>(trn_info.primpos);
				
				myPh.set(ptoff, trn_info.pos);
			}
		}
	}
}

void
ThreadedPointDeform::buildTransformationMatrix(TransformInfo&& trn_info)
{
	trn_info.geoprim->evaluateNormalVector(trn_info.primnml, trn_info.hituv.x(), trn_info.hituv.y());
	trn_info.geoprim->evaluateInteriorPoint(trn_info.primpos, trn_info.hituv.x(), trn_info.hituv.y());

	trn_info.up = trn_info.primnml;
	trn_info.up.cross({ 0.f, 1.f, 0.f });
	trn_info.rot.lookat({ 0.f, 0.f, 0.f }, trn_info.primnml, trn_info.up);
}

