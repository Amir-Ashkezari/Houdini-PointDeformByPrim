#include <GU/GU_Detail.h>
#include <GU/GU_RayIntersect.h>
#include <UT/UT_Assert.h>

#include "ThreadedPointDeform.h"
#include <iostream>

using namespace AKA;

ThreadedPointDeform::ThreadedPointDeform(const Gdps &gdps,
										 GA_SplittableRange *ptrange,
										 DriveAttrib_Info *drive_attrib_hs,
										 CaptureAttributes_Info *captureattribs_info,
										 const UT_Array<UT_StringHolder> &attribnames_to_interpolate)
	: myGdps(gdps)
	, myPtRange(ptrange)
	, myDriveAttribHs(drive_attrib_hs)
	, myBasePh(gdps.BaseGdp->getP())
	, myPh(gdps.Gdp->getP())
	, myCaptureAttributes_Info(captureattribs_info)
{
	for (const UT_StringHolder &attribname : attribnames_to_interpolate)
	{
		myBasePtAttribsh.emplace_back(gdps.BaseGdp->findAttribute(GA_ATTRIB_POINT, attribname));
		myPtAttribsh.emplace_back(gdps.Gdp->findAttribute(GA_ATTRIB_POINT, attribname));
	}
}

void
ThreadedPointDeform::pointCapture(GU_RayIntersect *ray_gdp, GA_Offset ptoff)
{
	TransformInfo trn_info;
	trn_info.Pos = myBasePh.get(ptoff);

	GU_MinInfo min_info;
	ray_gdp->minimumPoint(trn_info.Pos, min_info);

	trn_info.CapturePrims.emplace_back(min_info.prim->getMapIndex());
	trn_info.CaptureUVWs.emplace_back(min_info.u1);
	trn_info.CaptureUVWs.emplace_back(min_info.v1);
	trn_info.CaptureWeights.emplace_back(1.f);

	min_info.prim->evaluateInteriorPoint(trn_info.PrimPosition, min_info.u1, min_info.v1);
	UT_Vector3F min_dir = trn_info.PrimPosition - trn_info.Pos;
	fpreal32 min_dist = min_dir.length();
	min_dir.normalize();

	if (min_dist > myCaptureAttributes_Info->CaptureMinDistThresh && 
		myCaptureAttributes_Info->CaptureMultiSamples && !myDriveAttribHs->Drive)
	{
		UT_Vector3F x, y;
		fpreal32 max_ray_dist, max_dist;

		y = { 0.f, 1.f, 0.f };
		if (min_dir.dot(UT_Vector3H(0.f, 1.f, 0.f)) > 0.99f)
			y = { 1.f, 0.f, 0.f };
		y = cross(y, min_dir);
		x = cross(y, min_dir);
		UT_Array<UT_Vector3F> dirs{ -min_dir, y, -y, x, -x };

		max_dist = 1.f;
		max_ray_dist = SYSmax(min_dist, 0.001f) * 1e+3f;
		for (UT_Vector3F &dir : dirs)
		{
			GU_RayInfo ray_info(max_ray_dist, min_dist);
			int32 hit = ray_gdp->sendRay(trn_info.Pos, dir, ray_info);
			if (hit < 1)
				continue;

			ray_info.myPrim->evaluateInteriorPoint(trn_info.PrimPosition, ray_info.myU, ray_info.myV);
			fpreal32 hit_dist = (trn_info.PrimPosition - trn_info.Pos).length();
			fpreal32 dist_ratio = min_dist / hit_dist;
			max_dist += dist_ratio;

			trn_info.CapturePrims.emplace_back(ray_info.myPrim->getMapIndex());
			trn_info.CaptureUVWs.emplace_back(ray_info.myU);
			trn_info.CaptureUVWs.emplace_back(ray_info.myV);
			trn_info.CaptureWeights.emplace_back(dist_ratio);
		}

		fpreal32 delta = 1.f / max_dist;
		for (int i = 0; i < trn_info.CaptureWeights.size(); ++i)
			trn_info.CaptureWeights[i] *= delta;
	}

	buildXform(trn_info, myGdps.RestGdp, myDriveAttribHs->RestNormal_H, myDriveAttribHs->RestUp_H);
	trn_info.Rot.invert();

	trn_info.Pos = myBasePh.get(ptoff);
	trn_info.Pos -= trn_info.WeightedPos;
	trn_info.Pos.rowVecMult(trn_info.Rot);

	myCaptureAttributes_Info->RestP_H.set(ptoff, trn_info.Pos);
	myCaptureAttributes_Info->CapturePrims_H.set(ptoff, trn_info.CapturePrims);
	myCaptureAttributes_Info->CaptureUVWs_H.set(ptoff, trn_info.CaptureUVWs);
	myCaptureAttributes_Info->CaptureWeights_H.set(ptoff, trn_info.CaptureWeights);
	if (myCaptureAttributes_Info->XformRequired)
		myCaptureAttributes_Info->Xform_H.set(ptoff, trn_info.Rot);
}

void
ThreadedPointDeform::capturePartial(GU_RayIntersect *ray_gdp, const UT_JobInfo &info)
{
	for (GA_PageIterator pit = myPtRange->beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
				pointCapture(ray_gdp, ptoff);
		}
	}
}

void
ThreadedPointDeform::captureByPieceAttribPartial(GA_ROHandleI pieceattrib_h, 
												 MapRay<int32> rest_prim_rays, 
												 const UT_JobInfo &info)
{
	const GA_Attribute *pieceattrib = pieceattrib_h.getAttribute();
	const GA_AttributeOwner &pieceattrib_owner = pieceattrib->getOwner();

	for (GA_PageIterator pit = myPtRange->beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				int32 pieceattrib_val;
				if (pieceattrib_owner == GA_ATTRIB_PRIMITIVE)
				{
					GA_OffsetArray prims;
					myGdps.Gdp->getPrimitivesReferencingPoint(prims, ptoff);
					pieceattrib_val = pieceattrib_h.get(prims[0]);
				}
				else
					pieceattrib_val = pieceattrib_h.get(ptoff);

				if (rest_prim_rays.Map.find(pieceattrib_val) == rest_prim_rays.Map.end())
					continue;

				pointCapture(rest_prim_rays.Map.at(pieceattrib_val), ptoff);
			}
		}
	}
}

void
ThreadedPointDeform::captureByPieceAttribPartial(GA_ROHandleS pieceattrib_h, 
												 MapRay<UT_StringHolder> rest_prim_rays, 
												 const UT_JobInfo &info)
{
	const GA_Attribute* pieceattrib = pieceattrib_h.getAttribute();
	const GA_AttributeOwner& pieceattrib_owner = pieceattrib->getOwner();

	for (GA_PageIterator pit = myPtRange->beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				UT_StringHolder pieceattrib_val;
				if (pieceattrib_owner == GA_ATTRIB_PRIMITIVE)
				{
					GA_OffsetArray prims;
					myGdps.Gdp->getPrimitivesReferencingPoint(prims, ptoff);
					pieceattrib_val = pieceattrib_h.get(prims[0]);
				}
				else
					pieceattrib_val = pieceattrib_h.get(ptoff);

				if (rest_prim_rays.Map.find(pieceattrib_val) == rest_prim_rays.Map.end())
					continue;

				pointCapture(rest_prim_rays.Map.at(pieceattrib_val), ptoff);
			}
		}
	}
}

void
ThreadedPointDeform::deformPartial(const UT_JobInfo &info)
{
	for (GA_PageIterator pit = myPtRange->beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				TransformInfo trn_info;
				myCaptureAttributes_Info->CapturePrims_H.get(ptoff, trn_info.CapturePrims);
				myCaptureAttributes_Info->CaptureUVWs_H.get(ptoff, trn_info.CaptureUVWs);
				myCaptureAttributes_Info->CaptureWeights_H.get(ptoff, trn_info.CaptureWeights);

				buildXform(trn_info, myGdps.DeformedGdp, myDriveAttribHs->DeformedNormal_H, myDriveAttribHs->DeformedUp_H);
				if (myCaptureAttributes_Info->XformRequired)
				{
					UT_Matrix3F final_xform = myCaptureAttributes_Info->Xform_H.get(ptoff);
					final_xform *= trn_info.Rot;

					for (size_t idx = 0; idx < myBasePtAttribsh.size(); ++idx)
					{
						UT_Vector3F vectorattrib;
						vectorattrib = myBasePtAttribsh[idx].get(ptoff);
						vectorattrib.rowVecMult(final_xform);
						myPtAttribsh[idx].set(ptoff, vectorattrib);
					}
				}

				trn_info.Pos = myCaptureAttributes_Info->RestP_H.get(ptoff);
				trn_info.Pos.rowVecMult(trn_info.Rot);
				trn_info.Pos += trn_info.WeightedPos;
				
				myPh.set(ptoff, trn_info.Pos);
			}
		}
	}
}

void
ThreadedPointDeform::buildXform(TransformInfo &trn_info,
								const GU_Detail *gdp,
								const GA_ROHandleV3 &normal_attrib_h,
								const GA_ROHandleV3 &up_attrib_h)
{
	GA_ROHandleV3 temp_ph(gdp->getP());
	const GA_IndexMap &prim_map = gdp->getIndexMap(GA_ATTRIB_PRIMITIVE);

	trn_info.WeightedPos = 0.f;
	UT_Vector3F weighted_nrm(0.f), weighted_up(0.f);

	for (exint idx = 0; idx < trn_info.CapturePrims.size(); ++idx)
	{
		const exint vec2off = idx * 2;
		const GEO_Primitive *geo_prim = gdp->getGEOPrimitive(prim_map.offsetFromIndex(trn_info.CapturePrims[idx]));
		geo_prim->evaluateInteriorPoint(
			trn_info.PrimPosition, trn_info.CaptureUVWs[vec2off], trn_info.CaptureUVWs[vec2off + 1]);
		trn_info.Pos[0] = trn_info.PrimPosition[0];
		trn_info.Pos[1] = trn_info.PrimPosition[1];
		trn_info.Pos[2] = trn_info.PrimPosition[2];

		if (myDriveAttribHs->Drive)
		{
			UT_Array<GA_Offset> vtxoffsets;
			UT_Array<fpreal32> weightlist;
			geo_prim->computeInteriorPointWeights(
				vtxoffsets, weightlist, trn_info.CaptureUVWs[vec2off], trn_info.CaptureUVWs[vec2off + 1], 0.f);

			trn_info.PrimNormal = 0.f;
			trn_info.Up = 0.f;
			for (GA_Offset j = 0; j < vtxoffsets.size(); ++j)
			{
				trn_info.PrimNormal += normal_attrib_h.get(gdp->vertexPoint(vtxoffsets[j])) * weightlist[j];
				trn_info.Up += up_attrib_h.get(gdp->vertexPoint(vtxoffsets[j])) * weightlist[j];
			}
		}
		else
		{
			geo_prim->evaluateNormalVector(
				trn_info.PrimNormal, trn_info.CaptureUVWs[vec2off], trn_info.CaptureUVWs[vec2off + 1]);
			GA_Offset primpt_off = geo_prim->getPointOffset(0);
			UT_Vector3F primpt_pos = temp_ph.get(primpt_off);
			trn_info.Up = trn_info.Pos - primpt_pos;
			trn_info.Up.normalize();
			trn_info.Up = cross(trn_info.PrimNormal, trn_info.Up);
		}

		trn_info.WeightedPos += trn_info.Pos * trn_info.CaptureWeights[idx];
		weighted_nrm += trn_info.PrimNormal * trn_info.CaptureWeights[idx];
		weighted_up += trn_info.Up * trn_info.CaptureWeights[idx];
	}

	trn_info.Rot.lookat({ 0.f, 0.f, 0.f }, weighted_nrm, weighted_up);
}