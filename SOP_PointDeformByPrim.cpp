#include "SOP_PointDeformByPrim.h"
#include "SOP_PointDeformByPrim.proto.h"
#include "ThreadedPointDeform.h"
#include "Utils.h"
#include "Timer.h"

#include <SOP/SOP_NodeVerb.h>
#include <GU/GU_Detail.h>
#include <GA/GA_AttributeRefMap.h>
#include <DEP/DEP_MicroNode.h>
#include <GU/GU_RayIntersect.h>
#include <GA/GA_Handle.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Vector.h>
#include <UT/UT_Assert.h>
#include <SYS/SYS_Math.h>
#include <iostream>
#include <limits.h>


using namespace AKA;

const UT_StringHolder SOP_PointDeformByPrim::theSOPTypeName("pointdeformbyprim"_sh);

void newSopOperator(OP_OperatorTable* table)
{
	OP_Operator* op = new OP_Operator(
		SOP_PointDeformByPrim::theSOPTypeName,
		"Point Deform By Prim",
		SOP_PointDeformByPrim::myConstructor,
		SOP_PointDeformByPrim::buildTemplates(),
		2,
		3,
		nullptr,
		0);

	op->setIconName("SOP_pointdeform");
    op->setIsThreadSafe(true);

    table->addOperator(op);         
}

const char* SOP_PointDeformByPrim::inputLabel(unsigned idx) const
{
	switch (idx)
	{
		case 0:  
            return "Mesh to Deform";
		case 1:  
            return "Rest Point Lattice";
		case 2:  
            return "Deformed Point Lattice";
		default:
            return "Invalid Source";
	}
}

static const char* theDsFile = R"THEDSFILE(
{
    name        parameters
	parm {
        name    "group"
        cppname "Group"
        label   "Group"
        type    string
        default { "" }
        parmtag { "script_action" "import soputils\nkwargs['geometrytype'] = (hou.geometryType.Points,)\nkwargs['inputindex'] = 0\nsoputils.selectGroupParm(kwargs)" }
        parmtag { "script_action_help" "Select geometry from an available viewport.\nShift-click to turn on Select Groups." }
        parmtag { "script_action_icon" "BUTTONS_reselect" }
        help    "Which points of the model (the first input) to capture and deform using the lattice. Leave this blank to deform all points in the first input. Click the Reselect button to the right to interactively select the points in the viewer."
    }
	parm {
        name    "mode"
        cppname "Mode"
        label   "Mode"
        type    ordinal
        default { "0" }
        menu {
            "capturedeform"    "Capture and Deform"
            "capture"          "Capture" 
            "deform"           "Deform"
		}
        help    "Normal operation of this node is to both capture points and deform. However, you can use this parameter to have the node only perform one of those operations. There is no speed difference to using one node to do both, or two nodes to do the two steps separately. The only reason you might want to change this is if you have a technical workflow where you want to use the capture information in another node before doing the deformation for some reason. \nNote that the capture attributes are included in the output if you turn off Delete Capture Attributes, so you don't need to use separate nodes if all you want is to use the capture attributes downstream."
	}
	parm {
        name    "captureattribshalf"
		cppname "CaptureAttribsHalf"
        label   "Capture Attributes 16-bit"
        type    toggle
        default { "1" }
        help "Store capture attributes in 16-bit(half-float) instead 32-bit(float), in order to reduce memory footprint."
    }
	groupsimple {
        name    "capture_folder"
        label   "Capture"

		parm {
            name    "radius"
			cppname "Radius"
            label   "Radius"
            type    float
            default { "0.1" }
            range   { 0 10 }
			help    "The maximum distance (in Houdini world units) away from each model point to look for nearby lattice points."
        }
        parm {
            name    "pieceattrib"
            cppname "PieceAttrib"
            label   "Piece Attribute"
            type    string
            default { "" }
			help    "The name of a string or integer point attribute to use to treat the geometry as separate pieces. The attribute must be present on both the mesh and rest lattice. Points with the same value in this attribute are considered part of the same 'piece'. When you specify a valid piece attribute, this node deforms each piece using only the lattice points with the same piece value. \nThis lets you deform independent objects (pieces) in a single pass. You can create a piece attribute based on connectivity with the Connectivity SOP."
        }
		parm {
            name    "preseparatepieces"
            cppname "PreSeparatePieces"
            label   "Pre-Separate Pieces"
            type    toggle
            default { "1" }
            help "Virtually move pieces apart (to ensure they don't overlap) before capturing. This can greatly reduce the cost of capturing if many pieces are close together."
        }
	
        hidewhen "{ mode == deform }"
	}
)THEDSFILE"
// ==== This is necessary because MSVC++ has a limit of 16380 character per
// ==== string literal
R"THEDSFILE(
	groupsimple {
        name    "deform_folder"
        label   "Deform"

		parm {
            name    "rigidprojection"
			cppname "RigidProjection"
            label   "Rigid Projection"
            type    toggle
            default { "1" }
            help "The computed local transforms may include a shear. This removes any shear, leaving only a rigid transform. You can try turning this parameter off or on to see how it affects the look of the deformation."
        }
		parm {
            name    "updateaffectednmls"
			cppname "UpdateAffectedNmls"
            label   "Recompute Affected Normals"
            type    toggle
            default { "1" }
            help "Recomputes any normals that are affected by polygons with both deformed and undeformed points. This only matters if you are deforming some points and not others. If you are deforming the whole geometry (or whole pieces), this has no effect, since the normals are transformed. \nIf P (point position) is not in the Attributes to Transform below, this has no effect."
        }
        parm {
            name    "attribs"
            cppname "Attribs"
            label   "Attributes to Transform"
            type    string
            default { "*" }
            help "A space-separated list of attribute names/patterns, specifying which attributes are transformed by the deformation. The default is *, meaning all attributes. The node modifies vector attributes according to their type info, as points, vectors, or normals."
        }
	
        hidewhen "{ mode == capture }"
	}
}
)THEDSFILE";

void SOP_PointDeformByPrim::genPointAttribList(void* thedata,
											   PRM_Name* thechoicenames,
											   int thelistsize,
											   const PRM_SpareData* thespareptr,
											   const PRM_Parm* theparm)
{
    SOP_Node* thesop = static_cast<SOP_Node*>(thedata);
    UT_ASSERT(thesop != nullptr);
    const GU_Detail* gdp = thesop->getLastGeo();
    UT_ASSERT(gdp != nullptr);
    
    size_t count = 0;
	for (GA_Attribute* pointAttrib : gdp->pointAttribs())
	{
        if (theparm->getTokenRef() == "pieceattrib")
        {
            if (pointAttrib->getTupleSize() == 1)
            {
                GA_StorageClass attribClass = pointAttrib->getStorageClass();
                if (attribClass == GA_STORECLASS_INT || attribClass == GA_STORECLASS_STRING)
                    thechoicenames[count++] = PRM_Name(pointAttrib->getFullName().buffer());
            }
        }
        else if (theparm->getTokenRef() == "attribs")
        {
			if (pointAttrib->getTupleSize() == 3)
			{
                GA_TypeInfo attribType = pointAttrib->getTypeInfo();
				if (attribType == GA_TYPE_POINT || attribType == GA_TYPE_VECTOR || attribType == GA_TYPE_NORMAL)
					thechoicenames[count++] = PRM_Name(pointAttrib->getFullName().buffer());
			}
        }
	}
    thechoicenames[count] = PRM_Name();
}

PRM_ChoiceList SOP_PointDeformByPrim::s_thePointAttribList(
    PRM_ChoiceListType::PRM_CHOICELIST_TOGGLE, &SOP_PointDeformByPrim::genPointAttribList);

PRM_Template* SOP_PointDeformByPrim::buildTemplates()
{
	static PRM_TemplateBuilder templ("SOP_PointDeformByPrim.cpp"_sh, theDsFile);
	if (templ.justBuilt())
	{
		templ.setChoiceListPtr("group", &SOP_Node::pointGroupMenu);
		templ.setChoiceListPtr("pieceattrib", &SOP_PointDeformByPrim::s_thePointAttribList);
		templ.setChoiceListPtr("attribs", &SOP_PointDeformByPrim::s_thePointAttribList);
	}
	return templ.templates();
}

OP_Node* SOP_PointDeformByPrim::myConstructor(OP_Network* net, const char* name, OP_Operator* op)
{
	return new SOP_PointDeformByPrim(net, name, op);
}

SOP_PointDeformByPrim::SOP_PointDeformByPrim(OP_Network* net, const char* name, OP_Operator* op)
	: SOP_Node(net, name, op)
{
	mySopFlags.setManagesDataIDs(true);
}

SOP_PointDeformByPrim::~SOP_PointDeformByPrim()
{
}

OP_ERROR SOP_PointDeformByPrim::cookMySop(OP_Context& context)
{
	return cookMyselfAsVerb(context);
}

class SOP_PointDeformByPrimVerb : public SOP_NodeVerb
{
public:
    SOP_PointDeformByPrimVerb() {}

	virtual ~SOP_PointDeformByPrimVerb() {}

	virtual SOP_NodeParms* allocParms() const { return new SOP_PointDeformByPrimParms(); }
	virtual UT_StringHolder name() const { return SOP_PointDeformByPrim::theSOPTypeName; }

	virtual CookMode cookMode(const SOP_NodeParms* parms) const { return COOK_GENERIC; }
	virtual void cook(const CookParms& cookparms) const;

	static const SOP_NodeVerb::Register<SOP_PointDeformByPrimVerb> theVerb;

private:
	template<typename T, typename V>
	void captureClosestPointByPieceAttrib(const GU_Detail* restGdp,
										  V pieceAttrib_h,
										  V restPrimPieceAttrib_h,
										  ThreadedPointDeform& thread_ptdeform) const;

	void captureByPieceAttrib(GU_Detail* gdp,
							  const CookParms& cookparms,
							  const GA_AttributeOwner& attribOwner,
							  ThreadedPointDeform& thread_ptdeform) const;

private:
    static UT_StringHolder s_pieceAttribCache;
    static int32 s_baseMetaCacheCount;
	static int32 s_restMetaCacheCount;
        
};

const SOP_NodeVerb::Register<SOP_PointDeformByPrimVerb> SOP_PointDeformByPrimVerb::theVerb;

UT_StringHolder SOP_PointDeformByPrimVerb::s_pieceAttribCache = "";
int32 SOP_PointDeformByPrimVerb::s_baseMetaCacheCount = -1;
int32 SOP_PointDeformByPrimVerb::s_restMetaCacheCount = -1;

const SOP_NodeVerb* SOP_PointDeformByPrim::cookVerb() const
{
	return SOP_PointDeformByPrimVerb::theVerb.get();
}

template<typename T, typename V>
void SOP_PointDeformByPrimVerb::captureClosestPointByPieceAttrib(const GU_Detail* restGdp,
																 V pieceAttrib_h,
																 V restPrimPieceAttrib_h,
																 ThreadedPointDeform& thread_ptdeform) const
{
	Timer t0("detachedgroup");
	MapPrimGroup<T> primGroupMap;
	GA_Range restPrimRange(std::move(restGdp->getPrimitiveRange()));

	T attribValue;
	for (GA_Iterator primIt(restPrimRange.begin()); primIt != restPrimRange.end(); ++primIt)
	{
		attribValue = restPrimPieceAttrib_h.get(*primIt);
		if (primGroupMap.Map.find(attribValue) == primGroupMap.Map.end())
		{
			primGroupMap.Map[attribValue] = restGdp->newDetachedPrimitiveGroup();
			primGroupMap.Map.at(attribValue)->addOffset(restGdp->primitiveOffset(*primIt));
		}
		else
			primGroupMap.Map.at(attribValue)->addOffset(*primIt);
	}
	t0.stop();

	Timer t1("map_rays");
	MapRay<T> restPrimRays;
	for (auto& kv : primGroupMap.Map)
	{
		GU_RayIntersect* ray = new GU_RayIntersect(restGdp, kv.second, true, false, true);
		restPrimRays.Map[kv.first] = ray;
	}
	t1.stop();

	Timer t2("execute");
	thread_ptdeform.captureClosestPointByPieceAttrib(pieceAttrib_h, restPrimRays);
	t2.stop();

	for (auto& kv : restPrimRays.Map)
		delete restPrimRays.Map.at(kv.first);

	for (auto& kv : primGroupMap.Map)
		primGroupMap.Map.at(kv.first)->clearEntries();
}

void SOP_PointDeformByPrimVerb::captureByPieceAttrib(GU_Detail* gdp,
													 const CookParms& cookparms,
													 const GA_AttributeOwner& attribOwner,
													 ThreadedPointDeform& thread_ptdeform) const
{
	auto&& sopparms = cookparms.parms<SOP_PointDeformByPrimParms>();
	const GU_Detail* restGdp = cookparms.inputGeo(1);
	const GU_Detail* deformedGdp = cookparms.inputGeo(2);

	const UT_StringHolder& pieceAttribParmData = sopparms.getPieceAttrib();
	const GA_Attribute* restPrimPieceAttrib = restGdp->findPrimitiveAttribute(pieceAttribParmData);
	const GA_AttributeType& restPrimPieceAttribType = restPrimPieceAttrib->getType();

	if (restPrimPieceAttribType.getTypeName() == "string")
	{
		GA_Attribute* pieceAttrib = gdp->findStringTuple(attribOwner, pieceAttribParmData, 1, 1);
		GA_ROHandleS pieceAttrib_h(pieceAttrib);
		GA_ROHandleS restPrimPieceAttrib_h(restPrimPieceAttrib);

		if (pieceAttrib_h.isValid() && restPrimPieceAttrib_h.isValid())
			captureClosestPointByPieceAttrib<UT_StringHolder, GA_ROHandleS>(restGdp, pieceAttrib_h, restPrimPieceAttrib_h, thread_ptdeform);
		else
		{
			cookparms.sopAddError(SOP_MESSAGE, "Only string/integer type is allowed for Piece attribute!\n");
			return;
		}
	}
	else if (restPrimPieceAttribType.getTypeName() == "numeric")
	{
		GA_Attribute* pieceAttrib = gdp->findIntTuple(attribOwner, pieceAttribParmData, 1, 1);
		GA_ROHandleI pieceAttrib_h(pieceAttrib);
		GA_ROHandleI restPrimPieceAttrib_h(restPrimPieceAttrib);

		if (pieceAttrib_h.isValid() && restPrimPieceAttrib_h.isValid())
			captureClosestPointByPieceAttrib<int32, GA_ROHandleI>(restGdp, pieceAttrib_h, restPrimPieceAttrib_h, thread_ptdeform);
		else
		{
			cookparms.sopAddError(SOP_MESSAGE, "Only string/integer type is allowed for Piece attribute!\n");
			return;
		}
	}
	else
	{
		cookparms.sopAddError(SOP_MESSAGE, "Only string/integer type is allowed for Piece attribute!\n");
		return;
	}
}

void SOP_PointDeformByPrimVerb::cook(const CookParms& cookparms) const
{
	const GU_Detail* baseGdp = cookparms.inputGeo(0);
	const GU_Detail* restGdp = cookparms.inputGeo(1);
	const GU_Detail* deformedGdp = cookparms.inputGeo(2);
    if (baseGdp->isEmpty() || restGdp->isEmpty() || !restGdp->getNumPrimitives())
    {
        cookparms.sopAddError(SOP_MESSAGE, "First/Second input should contain valid geometry!\n");
        return;
    }

    //GA_PrimitiveTypeMask prim_types = restGdp->getPrimitiveTypeMask();
    //for (GA_PrimitiveTypeId prim_id : prim_types)
    //{
    //    std::cout << prim_id.get() << std::endl;
    //}
    //return;

    DEP_MicroNodeList inputsdep;
    cookparms.depnode()->getInputs(inputsdep);
    if (inputsdep[0]->isTimeDependent() || inputsdep[1]->isTimeDependent())
    {
		cookparms.sopAddError(SOP_MESSAGE, "First/Second input cannot be time dependent!\n");
		return;
    }
    //OP_DataMicroNode* depnode = UTverify_cast<OP_DataMicroNode*>(inputsdep[0]);

	auto&& sopparms = cookparms.parms<SOP_PointDeformByPrimParms>();
	GU_Detail* gdp = cookparms.gdh().gdpNC();

	if (!deformedGdp || deformedGdp->isEmpty())
		return;

	if (restGdp->getNumPrimitives() != deformedGdp->getNumPrimitives())
	{
		cookparms.sopAddWarning(SOP_MESSAGE, "Rest/deformed geometry cannot have different topology!\n");
		return;
	}

	const GA_Primitive* rest_prim = restGdp->getPrimitive(restGdp->primitiveOffset(0));
	const GA_Primitive* deformed_prim = deformedGdp->getPrimitive(deformedGdp->primitiveOffset(0));
	GA_Size rest_vtxcount = rest_prim->getVertexCount();
	if (rest_vtxcount != deformed_prim->getVertexCount())
	{
		cookparms.sopAddWarning(SOP_MESSAGE, "Rest/deformed geometry cannot have different topology!\n");
		return;
	}
	GA_Offset rest_primvtx, deformed_primvtx;
	for (GA_Size i = 0; i < rest_vtxcount; ++i)
	{
		rest_primvtx = rest_prim->getVertexOffset(i);
		deformed_primvtx = deformed_prim->getVertexOffset(i);
		if (rest_primvtx != deformed_primvtx)
		{
			cookparms.sopAddWarning(SOP_MESSAGE, "Rest/deformed geometry cannot have different topology!\n");
			return;
		}
	}

    // get parms
    const UT_StringHolder& pieceAttribParmData = sopparms.getPieceAttrib();
    const UT_StringHolder& attribParmData = sopparms.getAttribs();
    bool captureAttribsHalfData = sopparms.getCaptureAttribsHalf();

	// create/get attribs
	const UT_StringHolder& restXfrom_name("__restXform");
	const UT_StringHolder& restP_name("__restP");
	const UT_StringHolder& hitPrim_name("__hitprim");
	const UT_StringHolder& hitUV_name("__hituvw");
	HitAttributes hitAttribs;

    bool reinitialize = s_baseMetaCacheCount != baseGdp->getMetaCacheCount() || 
        s_restMetaCacheCount != restGdp->getMetaCacheCount() ||
        s_pieceAttribCache != pieceAttribParmData || gdp->isEmpty();

    if (reinitialize)
    {
        std::cout << "reinitiailzed!\n";
		s_baseMetaCacheCount = baseGdp->getMetaCacheCount();
		s_restMetaCacheCount = restGdp->getMetaCacheCount();
        s_pieceAttribCache = pieceAttribParmData;
	    gdp->replaceWith(*baseGdp);

		const GA_Storage& storage= captureAttribsHalfData ? GA_STORE_REAL16 : GA_STORE_REAL32;
		hitAttribs.Xform = gdp->addFloatTuple(GA_ATTRIB_POINT, restXfrom_name, 9, (GA_Defaults)0.f, nullptr, nullptr, storage);
		hitAttribs.Xform->setTypeInfo(GA_TYPE_TRANSFORM);
		hitAttribs.RestP = gdp->addFloatTuple(GA_ATTRIB_POINT, restP_name, 3, (GA_Defaults)0.f, nullptr, nullptr, storage);
		hitAttribs.RestP->setTypeInfo(GA_TYPE_POINT);
		hitAttribs.Prim = gdp->addIntTuple(GA_ATTRIB_POINT, hitPrim_name, 1);
		hitAttribs.Prim->setTypeInfo(GA_TYPE_ARITHMETIC_INTEGER);
		hitAttribs.UV = gdp->addFloatTuple(GA_ATTRIB_POINT, hitUV_name, 2, (GA_Defaults)0.f, nullptr, nullptr, storage);
		hitAttribs.UV->setTypeInfo(GA_TYPE_VECTOR);
    }
    else
    {
		hitAttribs.Xform = gdp->findAttribute(GA_ATTRIB_POINT, restXfrom_name);
		hitAttribs.RestP = gdp->findAttribute(GA_ATTRIB_POINT, restP_name);
		hitAttribs.Prim = gdp->findAttribute(GA_ATTRIB_POINT, hitPrim_name);
		hitAttribs.UV = gdp->findAttribute(GA_ATTRIB_POINT, hitUV_name);
    }

	hitAttribs.Xform->bumpDataId();
	hitAttribs.RestP->bumpDataId();
	hitAttribs.Prim->bumpDataId();
	hitAttribs.UV->bumpDataId();

	// create an array of attribs to interpolate point/vertex based on attribParm
	// AttribsToInterpolate is part of Utils.h
	AttribsToInterpolate attribsToInterpolate;
	attribsToInterpolate.BasePAttrib = baseGdp->getP();
	attribsToInterpolate.PAttrib = gdp->getP();

	const GA_AttributeSet& baseAttribs = baseGdp->getAttributes();
	const GA_AttributeSet& attribs = gdp->getAttributes();

	if (attribParmData == "*")
	{
		UT_Array<GA_TypeInfo> allowableType{ GA_TYPE_POINT, GA_TYPE_VECTOR, GA_TYPE_NORMAL };
		UT_Array<UT_StringHolder> excludingNames{ "P", restXfrom_name, hitUV_name};

		const GA_AttributeDict& basePointAttribs = baseAttribs.getDict(GA_ATTRIB_POINT);
		for (GA_AttributeDict::iterator pit(basePointAttribs.begin()); pit != basePointAttribs.end(); ++pit)
		{
			const GA_Attribute* curAttrib = pit.attrib();

			if (curAttrib && excludingNames.find(curAttrib->getFullName()) == -1)
				if (allowableType.find(curAttrib->getTypeInfo()) != -1)
					attribsToInterpolate.BasePtAttribs.emplace_back(curAttrib);
		}

		const GA_AttributeDict& pointAttribs = attribs.getDict(GA_ATTRIB_POINT);
		for (GA_AttributeDict::iterator pit(pointAttribs.begin()); pit != pointAttribs.end(); ++pit)
		{
			GA_Attribute* curAttrib = pit.attrib();
	
			if (curAttrib && excludingNames.find(curAttrib->getFullName()) == -1)
				if (allowableType.find(curAttrib->getTypeInfo()) != -1)
					attribsToInterpolate.PtAttribs.emplace_back(curAttrib);
		}

		const GA_AttributeDict& baseVertexAttribs = baseAttribs.getDict(GA_ATTRIB_VERTEX);
		for (GA_AttributeDict::iterator vit(baseVertexAttribs.begin()); vit != baseVertexAttribs.end(); ++vit)
		{
			const GA_Attribute* curAttrib = vit.attrib();

			if (curAttrib && allowableType.find(curAttrib->getTypeInfo()) != -1)
				attribsToInterpolate.BaseVtxAttribs.emplace_back(curAttrib);
		}
	
		const GA_AttributeDict& vertexAttribs = attribs.getDict(GA_ATTRIB_VERTEX);
		for (GA_AttributeDict::iterator vit(vertexAttribs.begin()); vit != vertexAttribs.end(); ++vit)
		{
			GA_Attribute* curAttrib = vit.attrib();
	
			if (curAttrib && allowableType.find(curAttrib->getTypeInfo()) != -1)
			attribsToInterpolate.VtxAttribs.emplace_back(curAttrib);
		}
	}
	
	GA_SplittableRange ptrange(std::move(gdp->getPointRange()));
    ThreadedPointDeform thread_ptdeform(gdp, baseGdp, restGdp, deformedGdp, ptrange, hitAttribs, attribsToInterpolate);
	
    if (reinitialize)
    {
        if (pieceAttribParmData)
        {
			if (restGdp->findPrimitiveAttribute(pieceAttribParmData))
			{
				if (gdp->findAttribute(GA_ATTRIB_PRIMITIVE, pieceAttribParmData))
				{
					captureByPieceAttrib(gdp, cookparms, GA_ATTRIB_PRIMITIVE, thread_ptdeform);
				}
				else if (gdp->findAttribute(GA_ATTRIB_POINT, pieceAttribParmData))
				{
					captureByPieceAttrib(gdp, cookparms, GA_ATTRIB_POINT, thread_ptdeform);
				}
				else
				{
					cookparms.sopAddError(SOP_MESSAGE, "Cannot find the Piece attribute on the first input!\n");
					return;
				}
			}
			else
			{
				cookparms.sopAddError(SOP_MESSAGE, "Cannot find the Piece attribute on the second input(rest geometry)!\n");
				return;
			}
        }
        else
        {
            GU_RayIntersect ray_rest(restGdp, nullptr, true, false, true);
            thread_ptdeform.captureClosestPoint(ray_rest);
        }
	
		hitAttribs.Xform->bumpDataId();
		hitAttribs.RestP->bumpDataId();
		hitAttribs.Prim->bumpDataId();
		hitAttribs.UV->bumpDataId();
	
		for (GA_Attribute* ptAttrib : attribsToInterpolate.PtAttribs)
			ptAttrib->bumpDataId();
		for (GA_Attribute* vtxAttrib : attribsToInterpolate.VtxAttribs)
			vtxAttrib->bumpDataId();
    }
	
	thread_ptdeform.computeDeformation();
	attribsToInterpolate.PAttrib->bumpDataId();
	
	for (GA_Attribute* ptAttrib : attribsToInterpolate.PtAttribs)
		ptAttrib->bumpDataId();
	for (GA_Attribute* vtxAttrib : attribsToInterpolate.VtxAttribs)
		vtxAttrib->bumpDataId();
}