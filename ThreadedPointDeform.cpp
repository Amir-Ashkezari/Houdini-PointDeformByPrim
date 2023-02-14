#include <GU/GU_Detail.h>
#include <GU/GU_RayIntersect.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Assert.h>

#include "ThreadedPointDeform.h"
#include <iostream>

using namespace AKA;

ThreadedPointDeform::ThreadedPointDeform(GU_Detail* gdp,
										 const GU_Detail* baseGdp,
										 const GU_Detail* restGdp,
										 const GU_Detail* deformedGdp,
										 const GA_SplittableRange& ptrange,
										 HitAttributes& hitAttribs,
										 AttribsToInterpolate& attribsToInterpolate)
	: m_gdp(gdp)
	, m_baseGdp(baseGdp)
	, m_restGdp(restGdp)
	, m_deformedGdp(deformedGdp)
	, m_ptrange(ptrange)
	, m_baseP_h(attribsToInterpolate.BasePAttrib)
	, m_p_h(attribsToInterpolate.PAttrib)
	, m_restXform_h(hitAttribs.Xform)
	, m_restP_h(hitAttribs.RestP)
	, m_hitPrim_h(hitAttribs.Prim)
	, m_hitUV_h(hitAttribs.UV)
{
	for (const GA_Attribute* basePtAttrib : attribsToInterpolate.BasePtAttribs)
		m_basePtAttribs_h.emplace_back(basePtAttrib);

	for (GA_Attribute* ptAttrib : attribsToInterpolate.PtAttribs)
		m_ptAttribs_h.emplace_back(ptAttrib);

	for (const GA_Attribute* baseVtxAttrib : attribsToInterpolate.BaseVtxAttribs)
		m_baseVtxAttribs_h.emplace_back(baseVtxAttrib);

	for (GA_Attribute* vtxAttrib : attribsToInterpolate.VtxAttribs)
		m_vtxAttribs_h.emplace_back(vtxAttrib);
}

void ThreadedPointDeform::captureClosestPointPartial(GU_RayIntersect& ray_gdp, const UT_JobInfo& info)
{
	for (GA_PageIterator pit = m_ptrange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			GU_MinInfo min_info(1e+3F, 1e-2F, 1);

			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				ray_gdp.minimumPoint(m_baseP_h.get(ptoff), min_info);

				trn_info.pos = m_baseP_h.get(ptoff);
				trn_info.hituv = { min_info.u1, min_info.v1 };
				trn_info.geoprim = min_info.prim;

				buildTransformationMatrix(std::move(trn_info));
				trn_info.rot.invert();

				trn_info.pos -= UTverify_cast<UT_Vector3F>(trn_info.primpos);
				//trn_info.pos.rowVecMult(UTverify_cast<UT_Matrix3F>(trn_info.rot));
				trn_info.pos.rowVecMult(trn_info.rot);

				m_restXform_h.set(ptoff, trn_info.rot);
				m_restP_h.set(ptoff, trn_info.pos);
				m_hitPrim_h.set(ptoff, min_info.prim->getMapIndex());
				m_hitUV_h.set(ptoff, trn_info.hituv);
			}
		}
	}
}

void ThreadedPointDeform::captureClosestPointByPieceAttribPartial(GA_ROHandleI pieceAttrib_h,
																  MapRay<int32> restPrimRays,
																  const UT_JobInfo& info)
{
	const GA_Attribute* pieceAttrib = pieceAttrib_h.getAttribute();
	const GA_AttributeOwner& pieceAttribOwner = pieceAttrib->getOwner();

	for (GA_PageIterator pit = m_ptrange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			GU_MinInfo min_info(1e+3F, 1e-2F, 1);
			int32 pieceAttribVal;
			GA_OffsetArray prims;

			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				if (pieceAttribOwner == GA_ATTRIB_PRIMITIVE)
				{
					m_gdp->getPrimitivesReferencingPoint(prims, ptoff);
					pieceAttribVal = pieceAttrib_h.get(prims[0]);
				}
				else
					pieceAttribVal = pieceAttrib_h.get(ptoff);

				if (restPrimRays.Map.find(pieceAttribVal) == restPrimRays.Map.end())
					continue;

				restPrimRays.Map.at(pieceAttribVal)->minimumPoint(m_p_h.get(ptoff), min_info);

				trn_info.pos = m_baseP_h.get(ptoff);
				trn_info.hituv = { min_info.u1, min_info.v1 };
				trn_info.geoprim = min_info.prim;

				buildTransformationMatrix(std::move(trn_info));
				trn_info.rot.invert();
				
				trn_info.pos -= UTverify_cast<UT_Vector3F>(trn_info.primpos);
				//trn_info.pos.rowVecMult(UTverify_cast<UT_Matrix3F>(trn_info.rot));
				trn_info.pos.rowVecMult(trn_info.rot);
				
				m_restXform_h.set(ptoff, trn_info.rot);
				m_restP_h.set(ptoff, trn_info.pos);
				m_hitPrim_h.set(ptoff, min_info.prim->getMapIndex());
				m_hitUV_h.set(ptoff, trn_info.hituv);
			}
		}
	}
}

void ThreadedPointDeform::captureClosestPointByPieceAttribPartial(GA_ROHandleS pieceAttrib_h,
																  MapRay<UT_StringHolder> restPrimRays,
																  const UT_JobInfo& info)
{
	const GA_Attribute* pieceAttrib = pieceAttrib_h.getAttribute();
	const GA_AttributeOwner& pieceAttribOwner = pieceAttrib->getOwner();

	for (GA_PageIterator pit = m_ptrange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			GU_MinInfo min_info(1e+3F, 1e-2F, 1);
			UT_StringHolder pieceAttribVal;
			GA_OffsetArray prims;

			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				if (pieceAttribOwner == GA_ATTRIB_PRIMITIVE)
				{
					m_gdp->getPrimitivesReferencingPoint(prims, ptoff);
					pieceAttribVal = pieceAttrib_h.get(prims[0]);
				}
				else
					pieceAttribVal = pieceAttrib_h.get(ptoff);

				if (restPrimRays.Map.find(pieceAttribVal) == restPrimRays.Map.end())
					continue;

				restPrimRays.Map.at(pieceAttribVal)->minimumPoint(m_p_h.get(ptoff), min_info);

				trn_info.pos = m_baseP_h.get(ptoff);
				trn_info.hituv = { min_info.u1, min_info.v1 };
				trn_info.geoprim = min_info.prim;

				buildTransformationMatrix(std::move(trn_info));
				trn_info.rot.invert();
				
				trn_info.pos -= UTverify_cast<UT_Vector3F>(trn_info.primpos);
				//trn_info.pos.rowVecMult(UTverify_cast<UT_Matrix3F>(trn_info.rot));
				trn_info.pos.rowVecMult(trn_info.rot);
				
				m_restXform_h.set(ptoff, trn_info.rot);
				m_restP_h.set(ptoff, trn_info.pos);
				m_hitPrim_h.set(ptoff, min_info.prim->getMapIndex());
				m_hitUV_h.set(ptoff, trn_info.hituv);
			}
		}
	}
}

void ThreadedPointDeform::computeDeformationPartial(const UT_JobInfo& info)
{
	for (GA_PageIterator pit = m_ptrange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			TransformInfo trn_info;
			
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				trn_info.pos = m_restP_h.get(ptoff);
				trn_info.hitprim = m_hitPrim_h.get(ptoff);
				trn_info.hituv = m_hitUV_h.get(ptoff);

				const GA_IndexMap& prim_map = m_deformedGdp->getIndexMap(GA_ATTRIB_PRIMITIVE);
				trn_info.geoprim = m_deformedGdp->getGEOPrimitive(prim_map.offsetFromIndex(trn_info.hitprim));

				buildTransformationMatrix(std::move(trn_info));
				
				UT_Vector3 vecAttrib;
				for (size_t idx = 0; idx < m_basePtAttribs_h.size(); ++idx)
				{
					vecAttrib = m_basePtAttribs_h[idx].get(ptoff);
					//vecAttrib.rowVecMult(UTverify_cast<UT_Matrix3F>(m_restXform_h.get(ptoff)));
					vecAttrib.rowVecMult(m_restXform_h.get(ptoff));
					//vecAttrib.rowVecMult(UTverify_cast<UT_Matrix3F>(trn_info.rot));
					vecAttrib.rowVecMult(trn_info.rot);
				
					m_ptAttribs_h[idx].set(ptoff, vecAttrib);
				}
				
				trn_info.pos.rowVecMult(trn_info.rot);
				trn_info.pos += UTverify_cast<UT_Vector3F>(trn_info.primpos);
				
				m_p_h.set(ptoff, trn_info.pos);
			}
		}
	}
}

void ThreadedPointDeform::buildTransformationMatrix(TransformInfo&& trn_info)
{
	trn_info.geoprim->evaluateNormalVector(trn_info.primnml, trn_info.hituv.x(), trn_info.hituv.y());
	trn_info.geoprim->evaluateInteriorPoint(trn_info.primpos, trn_info.hituv.x(), trn_info.hituv.y());

	trn_info.up = trn_info.primnml;
	trn_info.up.cross({ 0.f, 1.f, 0.f });
	trn_info.rot.lookat({ 0.f, 0.f, 0.f }, trn_info.primnml, trn_info.up);
}

