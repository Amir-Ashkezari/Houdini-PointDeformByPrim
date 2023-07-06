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
	, myPtRange(std::forward<GA_SplittableRange>(ptrange))
	, myDriveAttribHs(std::forward<DriveAttribHandles>(drive_attrib_hs))
	, myBasePh(attribs_to_interpolate.BasePAttrib)
	, myPh(attribs_to_interpolate.PAttrib)
	, myCaptureMultiSamples(capture_attribs.MultipleSamples)
	, myCaptureMinDistThresh(capture_attribs.MinDistThresh)
	, myRestPh(capture_attribs.RestP)
	, myCapturePrimsh(capture_attribs.Prims)
	, myCaptureUVWsh(capture_attribs.UVWs)
	, myCaptureWeightsh(capture_attribs.Weights)
{
	for (const GA_Attribute *basePtAttrib : attribs_to_interpolate.BasePtAttribs)
		myBasePtAttribsh.emplace_back(basePtAttrib);

	for (GA_Attribute *ptAttrib : attribs_to_interpolate.PtAttribs)
		myPtAttribsh.emplace_back(ptAttrib);

	for (const GA_Attribute *baseVtxAttrib : attribs_to_interpolate.BaseVtxAttribs)
		myBaseVtxAttribsh.emplace_back(baseVtxAttrib);

	for (GA_Attribute *vtxAttrib : attribs_to_interpolate.VtxAttribs)
		myVtxAttribsh.emplace_back(vtxAttrib);
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
				GU_MinInfo min_info;
				ray_gdp.minimumPoint(myBasePh.get(ptoff), min_info);

				TransformInfo trn_info;
				trn_info.Pos = myBasePh.get(ptoff);
				trn_info.CapturePrims.emplace_back(min_info.prim->getMapIndex());
				trn_info.CaptureUVWs.emplace_back(min_info.u1);
				trn_info.CaptureUVWs.emplace_back(min_info.v1);
				trn_info.CaptureWeights.emplace_back(1.f);
				trn_info.GeomPrim = min_info.prim;
				
				min_info.prim->evaluateInteriorPoint(trn_info.PrimPosition, min_info.u1, min_info.v1);
				UT_Vector3H min_dir = trn_info.PrimPosition - trn_info.Pos;
				fpreal16 min_dist = min_dir.length();
				min_dir.normalize();

				if (min_dist > myCaptureMinDistThresh && myCaptureMultiSamples && !myDriveAttribHs.Drive)
				{
					UT_Vector3H x, y;
					fpreal16 max_ray_dist, max_dist;
					
					y = std::forward<UT_Vector3H>({ 0.f, 1.f, 0.f });
					if (min_dir.dot(std::forward<UT_Vector3H>({ 0.f, 1.f, 0.f })) > 0.99f)
						y = std::forward<UT_Vector3H>({ 1.f, 0.f, 0.f });
					y = cross(y, min_dir);
					x = cross(y, min_dir);
					UT_Array<UT_Vector3H> dirs{ -min_dir, y, -y, x, -x };

					//max_ray_dist = min_dist * 1e+15f;
					max_dist = 1.f;
					for (UT_Vector3H &dir : dirs)
					{
						GU_RayInfo ray_info;
						int32 hit = ray_gdp.sendRay(myBasePh.get(ptoff), dir, ray_info);
						if (hit < 1)
							continue;
						
						ray_info.myPrim->evaluateInteriorPoint(trn_info.PrimPosition, ray_info.myU, ray_info.myV);
						fpreal16 hit_dist = (trn_info.PrimPosition - trn_info.Pos).length();
						fpreal16 dist_ratio = min_dist / hit_dist;
						max_dist += dist_ratio;

						trn_info.CapturePrims.emplace_back(ray_info.myPrim->getMapIndex());
						trn_info.CaptureUVWs.emplace_back(ray_info.myU);
						trn_info.CaptureUVWs.emplace_back(ray_info.myV);
						trn_info.CaptureWeights.emplace_back(dist_ratio);
					}

					fpreal16 delta = 1.f / max_dist;
					for (int i = 0; i < trn_info.CaptureWeights.size(); ++i)
						trn_info.CaptureWeights[i] *= delta;
				}

				//	buildXformByPrimIntrinsic(trn_info);
				//else
				//	buildXformByAttribute(trn_info, myRestGdp, myDriveAttribHs.RestNormal_H, myDriveAttribHs.RestUp_H);
				//trn_info.Rot.invert();

				//trn_info.Pos -= UTverify_cast<UT_Vector3F>(trn_info.PrimPosition);
				//trn_info.Pos.rowVecMult(trn_info.Rot);

				myRestPh.set(ptoff, trn_info.Pos);
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
			TransformInfo trn_info;
			GU_MinInfo min_info;
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
				
				myRestPh.set(ptoff, trn_info.Pos);
				myCapturePrimsh.set(ptoff, std::forward<UT_Int32Array>(trn_info.CapturePrims));
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
			TransformInfo trn_info;
			GU_MinInfo min_info;
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
				
				myRestPh.set(ptoff, trn_info.Pos);
				myCapturePrimsh.set(ptoff, std::forward<UT_Int32Array>(trn_info.CapturePrims));
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
			TransformInfo trn_info;
			
			for (GA_Offset ptoff = start; ptoff < end; ++ptoff)
			{
				/*trn_info.Pos = myRestPh.get(ptoff);
				trn_info.CapturePrims = myCapturePrimsh.get(ptoff);
				trn_info.CaptureUVWs = myCaptureUVWsh.get(ptoff);

				const GA_IndexMap &prim_map = myDeformedGdp->getIndexMap(GA_ATTRIB_PRIMITIVE);
				trn_info.GeomPrim = myDeformedGdp->getGEOPrimitive(prim_map.offsetFromIndex(trn_info.CapturePrims));

				if (myDriveAttribHs.Drive)
					buildXformByAttribute(trn_info, myDeformedGdp, myDriveAttribHs.DeformedNormal_H, myDriveAttribHs.DeformedUp_H);
				else
					buildXformByPrimIntrinsic(trn_info);

				UT_Vector3 vecAttrib;
				for (size_t idx = 0; idx < myBasePtAttribsh.size(); ++idx)
				{
					vecAttrib = myBasePtAttribsh[idx].get(ptoff);
					vecAttrib.rowVecMult(myCaptureWeightsh.get(ptoff));
					vecAttrib.rowVecMult(trn_info.Rot);

					myPtAttribsh[idx].set(ptoff, vecAttrib);
				}
				
				trn_info.Pos.rowVecMult(trn_info.Rot);
				trn_info.Pos += UTverify_cast<UT_Vector3F>(trn_info.PrimPosition);*/
				
				myPh.set(ptoff, trn_info.Pos);
			}
		}
	}
}

void
ThreadedPointDeform::buildXformByPrimIntrinsic(TransformInfo &trn_info, int32 idx)
{
	trn_info.GeomPrim->evaluateNormalVector(trn_info.PrimNormal, trn_info.CaptureUVWs[idx], trn_info.CaptureUVWs[idx]);
	trn_info.GeomPrim->evaluateInteriorPoint(trn_info.PrimPosition, trn_info.CaptureUVWs[idx], trn_info.CaptureUVWs[idx]);

	trn_info.Up = trn_info.PrimNormal;
	trn_info.Up.cross({ 0.f, 1.f, 0.f });
	trn_info.Rot.lookat({ 0.f, 0.f, 0.f }, trn_info.PrimNormal, trn_info.Up);
}

void
ThreadedPointDeform::buildXformByAttribute(TransformInfo &trn_info,
										   const GU_Detail *gdp,
										   const GA_ROHandleV3 &normal_attrib_h,
										   const GA_ROHandleV3 &up_attrib_h,
										   int32 idx)
{
	trn_info.GeomPrim->evaluateInteriorPoint(trn_info.PrimPosition, trn_info.CaptureUVWs[idx], trn_info.CaptureUVWs[idx]);

	UT_Array<exint> vtxoffsets;
	UT_Array<fpreal32> weightlist;
	trn_info.GeomPrim->computeInteriorPointWeights(vtxoffsets, weightlist, trn_info.CaptureUVWs[idx], trn_info.CaptureUVWs[idx], 0.f);

	trn_info.PrimNormal = 0.f;
	trn_info.Up = 0.f;
	for (exint i = 0; i < vtxoffsets.size(); ++i)
	{
		trn_info.PrimNormal += normal_attrib_h.get(gdp->vertexPoint(vtxoffsets[i])) * weightlist[i];
		trn_info.Up += up_attrib_h.get(gdp->vertexPoint(vtxoffsets[i])) * weightlist[i];
	}

	trn_info.Rot.lookat({ 0.f, 0.f, 0.f }, trn_info.PrimNormal, trn_info.Up);
}

