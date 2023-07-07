#include <GU/GU_Detail.h>
#include <GU/GU_RayIntersect.h>
#include <UT/UT_Assert.h>

#include "ThreadedPointDeform.h"
#include <iostream>

using namespace AKA;

ThreadedPointDeform::ThreadedPointDeform(GU_Detail *gdp,
										 const GU_Detail *base_gdp,
										 const GU_Detail *rest_gdp,
										 const GU_Detail *deformed_gdp,
										 GA_SplittableRange &&ptrange,
										 DriveAttribHandles &&drive_attrib_hs,
										 CaptureAttributes &capture_attribs,
										 AttribsToInterpolate &attribs_to_interpolate)
	: myGdp(gdp)
	, myBaseGdp(base_gdp)
	, myRestGdp(rest_gdp)
	, myDeformedGdp(deformed_gdp)
	, myPtRange(std::move(ptrange))
	, myDriveAttribHs(std::move(drive_attrib_hs))
	, myBasePh(attribs_to_interpolate.OldPAttrib)
	, myPh(attribs_to_interpolate.PAttrib)
	, myCaptureMultiSamples(capture_attribs.MultipleSamples)
	, myCaptureMinDistThresh(capture_attribs.MinDistThresh)
	, myOldPh(capture_attribs.RestP)
	, myCapturePrimsh(capture_attribs.Prims)
	, myCaptureUVWsh(capture_attribs.UVWs)
	, myCaptureWeightsh(capture_attribs.Weights)
{
	for (const GA_Attribute *basePtAttrib : attribs_to_interpolate.BasePtAttribs)
		myBasePtAttribsh.emplace_back(basePtAttrib);

	for (GA_Attribute *ptAttrib : attribs_to_interpolate.PtAttribs)
		myPtAttribsh.emplace_back(ptAttrib);
}

void
ThreadedPointDeform::captureClosestPointPartial(GU_RayIntersect &ray_gdp, const UT_JobInfo &info)
{
	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				TransformInfo trn_info;
				trn_info.Pos = myBasePh.get(ptoff);

				GU_MinInfo min_info;
				ray_gdp.minimumPoint(trn_info.Pos, min_info);

				trn_info.CapturePrims.emplace_back(min_info.prim->getMapIndex());
				trn_info.CaptureUVWs.emplace_back(min_info.u1);
				trn_info.CaptureUVWs.emplace_back(min_info.v1);
				trn_info.CaptureWeights.emplace_back(1.f);
				
				min_info.prim->evaluateInteriorPoint(trn_info.PrimPosition, min_info.u1, min_info.v1);
				UT_Vector3F min_dir = trn_info.PrimPosition - trn_info.Pos;
				fpreal32 min_dist = min_dir.length();
				min_dir.normalize();

				if (min_dist > myCaptureMinDistThresh && myCaptureMultiSamples && !myDriveAttribHs.Drive)
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
						int32 hit = ray_gdp.sendRay(trn_info.Pos, dir, ray_info);
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

				if (myDriveAttribHs.Drive)
					buildXformByAttribute(trn_info, myRestGdp, myDriveAttribHs.RestNormal_H, myDriveAttribHs.RestUp_H);
				else
					buildXformByPrimIntrinsic(trn_info, myRestGdp);
				trn_info.Rot.invert();

				trn_info.Pos = myBasePh.get(ptoff);
				trn_info.Pos -= trn_info.WeightedPos;
				trn_info.Pos.rowVecMult(trn_info.Rot);

				myOldPh.set(ptoff, trn_info.Pos);
				myCapturePrimsh.set(ptoff, trn_info.CapturePrims);
				myCaptureUVWsh.set(ptoff, trn_info.CaptureUVWs);
				myCaptureWeightsh.set(ptoff, trn_info.CaptureWeights);
			}
		}
	}
}

void
ThreadedPointDeform::captureClosestPointByPieceAttribPartial(GA_ROHandleI piece_attrib_h,
															 MapRay<int32> rest_prim_rays,
															 const UT_JobInfo &info)
{
	const GA_Attribute* pieceAttrib = piece_attrib_h.getAttribute();
	const GA_AttributeOwner& pieceAttribOwner = pieceAttrib->getOwner();

	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				TransformInfo trn_info;
				GU_MinInfo min_info;
				int32 piece_attrib_val;
				GA_OffsetArray prims;

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

				//trn_info.Pos = myBasePh.get(ptoff);
				//trn_info.CaptureUVWs = { min_info.u1, min_info.v1 };
				//trn_info.GeomPrim = min_info.prim;

				//if (myDriveAttribHs.Drive)
				//	buildXformByAttribute(trn_info, myRestGdp, myDriveAttribHs.RestNormal_H, myDriveAttribHs.RestUp_H);
				//else
				//	buildXformByPrimIntrinsic(trn_info);
				//trn_info.Rot.invert();
				//
				//trn_info.Pos -= UTverify_cast<UT_Vector3F>(trn_info.PrimPosition);
				//trn_info.Pos.rowVecMult(trn_info.Rot);
				
				myOldPh.set(ptoff, trn_info.Pos);
				myCapturePrimsh.set(ptoff, trn_info.CapturePrims);
				myCaptureUVWsh.set(ptoff, trn_info.CaptureUVWs);
				myCaptureWeightsh.set(ptoff, trn_info.CaptureWeights);
			}
		}
	}
}

void
ThreadedPointDeform::captureClosestPointByPieceAttribPartial(GA_ROHandleS piece_attrib_h,
															 MapRay<UT_StringHolder> rest_prim_rays,
															 const UT_JobInfo &info)
{
	const GA_Attribute* pieceAttrib = piece_attrib_h.getAttribute();
	const GA_AttributeOwner& pieceAttribOwner = pieceAttrib->getOwner();

	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;

		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				TransformInfo trn_info;
				GU_MinInfo min_info;
				UT_StringHolder piece_attrib_val;
				GA_OffsetArray prims;

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

				//trn_info.Pos = myBasePh.get(ptoff);
				//trn_info.CaptureUVWs = { min_info.u1, min_info.v1 };
				//trn_info.GeomPrim = min_info.prim;

				//if (myDriveAttribHs.Drive)
				//	buildXformByAttribute(trn_info, myRestGdp, myDriveAttribHs.RestNormal_H, myDriveAttribHs.RestUp_H);
				//else
				//	buildXformByPrimIntrinsic(trn_info);
				//trn_info.Rot.invert();
				//
				//trn_info.Pos -= UTverify_cast<UT_Vector3F>(trn_info.PrimPosition);
				//trn_info.Pos.rowVecMult(trn_info.Rot);
				
				myOldPh.set(ptoff, trn_info.Pos);
				myCapturePrimsh.set(ptoff, trn_info.CapturePrims);
				myCaptureUVWsh.set(ptoff, trn_info.CaptureUVWs);
				myCaptureWeightsh.set(ptoff, trn_info.CaptureWeights);
			}
		}
	}
}

void
ThreadedPointDeform::computeDeformationPartial(const UT_JobInfo &info)
{
	for (GA_PageIterator pit = myPtRange.beginPages(info); !pit.atEnd(); ++pit)
	{
		GA_Offset start, end;
		for (GA_Iterator it(pit.begin()); it.blockAdvance(start, end);)
		{
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				TransformInfo trn_info;
				myCapturePrimsh.get(ptoff, trn_info.CapturePrims);
				myCaptureUVWsh.get(ptoff, trn_info.CaptureUVWs);
				myCaptureWeightsh.get(ptoff, trn_info.CaptureWeights);

				if (myDriveAttribHs.Drive)
					buildXformByAttribute(trn_info, myDeformedGdp, myDriveAttribHs.DeformedNormal_H, myDriveAttribHs.DeformedUp_H);
				else
					buildXformByPrimIntrinsic(trn_info, myDeformedGdp);

				/*UT_Vector3 vecAttrib;
				for (size_t idx = 0; idx < myBasePtAttribsh.size(); ++idx)
				{
					vecAttrib = myBasePtAttribsh[idx].get(ptoff);
					vecAttrib.rowVecMult(myCaptureWeightsh.get(ptoff));
					vecAttrib.rowVecMult(trn_info.Rot);

					myPtAttribsh[idx].set(ptoff, vecAttrib);
				}*/

				trn_info.Pos = myOldPh.get(ptoff);
				trn_info.Pos.rowVecMult(trn_info.Rot);
				trn_info.Pos += trn_info.WeightedPos;
				
				myPh.set(ptoff, trn_info.Pos);
			}
		}
	}
}

void
ThreadedPointDeform::buildXformByPrimIntrinsic(TransformInfo &trn_info, const GU_Detail *gdp)
{
	GA_ROHandleV3 temp_ph(gdp->getP());
	const GA_IndexMap &prim_map = gdp->getIndexMap(GA_ATTRIB_PRIMITIVE);

	trn_info.WeightedPos = 0.f;
	UT_Vector3F weighted_nrm(0.f), weighted_up(0.f);
	for (int32 idx = 0; idx < trn_info.CapturePrims.size(); ++idx)
	{
		const GEO_Primitive *geo_prim = gdp->getGEOPrimitive(prim_map.offsetFromIndex(trn_info.CapturePrims[idx]));
		geo_prim->evaluateInteriorPoint(
			trn_info.PrimPosition, trn_info.CaptureUVWs[idx * 2], trn_info.CaptureUVWs[idx * 2 + 1]);
		trn_info.Pos[0] = trn_info.PrimPosition[0];
		trn_info.Pos[1] = trn_info.PrimPosition[1];
		trn_info.Pos[2] = trn_info.PrimPosition[2];

		if (myDriveAttribHs.Drive)
		{
		}
		else
		{
			geo_prim->evaluateNormalVector(
				trn_info.PrimNormal, trn_info.CaptureUVWs[idx * 2], trn_info.CaptureUVWs[idx * 2 + 1]);
			GA_Offset primpt_off = geo_prim->getPointOffset(0);
			UT_Vector3F primpt_pos = temp_ph.get(primpt_off);
			trn_info.Up = trn_info.Pos - primpt_pos;
			trn_info.Up.normalize();
			trn_info.Up = cross(trn_info.PrimNormal, trn_info.Up);
		}

		weighted_nrm += trn_info.PrimNormal * trn_info.CaptureWeights[idx];
		weighted_up += trn_info.Up * trn_info.CaptureWeights[idx];
		trn_info.WeightedPos += trn_info.Pos * trn_info.CaptureWeights[idx];
	}

	trn_info.Rot.lookat({ 0.f, 0.f, 0.f }, weighted_nrm, weighted_up);
}

void
ThreadedPointDeform::buildXformByAttribute(TransformInfo &trn_info,
										   const GU_Detail *gdp,
										   const GA_ROHandleV3 &normal_attrib_h,
										   const GA_ROHandleV3 &up_attrib_h)
{
	/*trn_info.GeomPrim->evaluateInteriorPoint(trn_info.PrimPosition, trn_info.CaptureUVWs[0], trn_info.CaptureUVWs[0]);

	UT_Array<exint> vtxoffsets;
	UT_Array<fpreal32> weightlist;
	trn_info.GeomPrim->computeInteriorPointWeights(vtxoffsets, weightlist, trn_info.CaptureUVWs[0], trn_info.CaptureUVWs[0], 0.f);

	trn_info.PrimNormal = 0.f;
	trn_info.Up = 0.f;
	for (exint i = 0; i < vtxoffsets.size(); ++i)
	{
		trn_info.PrimNormal += normal_attrib_h.get(gdp->vertexPoint(vtxoffsets[i])) * weightlist[i];
		trn_info.Up += up_attrib_h.get(gdp->vertexPoint(vtxoffsets[i])) * weightlist[i];
	}

	trn_info.Rot.lookat({ 0.f, 0.f, 0.f }, trn_info.PrimNormal, trn_info.Up);*/
}

