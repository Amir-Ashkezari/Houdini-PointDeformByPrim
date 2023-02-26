#include "SOP_PointDeformByPrim.h"
#include "SOP_PointDeformByPrim.proto.h"
#include "ThreadedPointDeform.h"
#include "Utils.h"

#include <SOP/SOP_NodeVerb.h>
#include <OP/OP_AutoLockInputs.h>
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
#include <UT/UT_SysSpecific.h>
#include <SYS/SYS_Math.h>
#include <iostream>
#include <limits.h>


using namespace AKA;

const UT_StringHolder SOP_PointDeformByPrim::theSOPTypeName("pointdeformbyprim"_sh);

void
newSopOperator(OP_OperatorTable *table)
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

const char *
SOP_PointDeformByPrim::inputLabel(unsigned idx) const
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
        parmtag { "script_action" "import soputils\nkwargs['geometrytype'] = (hou.geometryType.Points,)\nkwargs['inputindex'] = 0\nsoputils.selectgroup_parm(kwargs)" }
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
		}
        help    "Normal operation of this node is to both capture points and deform. However, you can use this parameter to have the node only perform capturing operation. There is no speed difference to using one node to do both, or two nodes to do the two steps separately. The only reason you might want to change this is if you have a technical workflow where you want to use the capture information in another node before doing the deformation for some reason. \nNote In order to avoid computing capture attributes on every cook, the node will keep capture attributes for consecutive cooks unless there is change in the upstream geometries."
	}
	parm {
        name    "captureattribshalf"
		cppname "CaptureAttribsHalf"
        label   "Capture Attributes 16-bit"
        type    toggle
        default { "1" }
        help    "In order to reduce memory footprint, by default capture attributes are stored as 16-bit(half-float) instead 32-bit(float)."
    }
	parm {
        name    "drivebyattribs"
		cppname "DriveByAttribs"
        label   "Drive By Attribs"
        type    toggle
        default { "0" }
        help    "Use normal/up vector from Rest/Deformed geomerty streams to drive the deformation."
    }
	parm {
        name    "normalattrib"
		cppname "NormalAttrib"
        label   "Normal Attrib"
        type    string
        default { "N" }
        help    "Normal vector attribute from Rest/Deformed geomerty streams for constructing transformation matrix."

		hidewhen    "{ drivebyattribs == 0 }"
		disablewhen "{ drivebyattribs == 0 }"
    }
	parm {
        name    "upattrib"
		cppname "UpAttrib"
        label   "Up Attrib"
        type    string
        default { "up" }
        help    "Up vector attribute from Rest/Deformed geomerty streams for constructing transformation matrix."

		hidewhen    "{ drivebyattribs == 0 }"
		disablewhen "{ drivebyattribs == 0 }"
    }
	groupsimple {
        name    "capture_folder"
        label   "Capture"

		parm {
            name    "radius"
			cppname "Radius"
            label   "Radius"
            type    float
            default { "0" }
            range   { 0 5 }
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
            help    "Virtually move pieces apart (to ensure they don't overlap) before capturing. This can greatly reduce the cost of capturing if many pieces are close together."
        }
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
            help    "The computed local transforms may include a shear. This removes any shear, leaving only a rigid transform. You can try turning this parameter off or on to see how it affects the look of the deformation."
        }
		parm {
            name    "updateaffectednmls"
			cppname "UpdateAffectedNmls"
            label   "Recompute Affected Normals"
            type    toggle
            default { "1" }
            help    "Recomputes any normals that are affected by polygons with both deformed and undeformed points. This only matters if you are deforming some points and not others. If you are deforming the whole geometry (or whole pieces), this has no effect, since the normals are transformed. \nIf P (point position) is not in the Attributes to Transform below, this has no effect."
        }
        parm {
            name    "attribs"
            cppname "Attribs"
            label   "Attributes to Transform"
            type    string
            default { "*" }
            help    "A space-separated list of attribute names/patterns, specifying which attributes are transformed by the deformation. The default is *, meaning all attributes. The node modifies vector attributes according to their type info, as points, vectors, or normals."
        }
	
        hidewhen "{ mode == capture }"
	}
}
)THEDSFILE";

static bool
sopApproveStringIntAttribs(const GA_Attribute *attrib, void*)
{
	if (GA_ATIString::isType(attrib))
		return true;

	const GA_ATINumeric *numeric = GA_ATINumeric::cast(attrib);
	if (!numeric)
		return false;
	return (numeric->getStorageClass() == GA_STORECLASS_INT);
}

static bool
sopApproveVectorAttribs(const GA_Attribute *attrib, void*)
{
	const GA_ATINumeric *numeric = GA_ATINumeric::cast(attrib);
	if (!numeric)
		return false;

	UT_Array<GA_TypeInfo> allowable_type{ GA_TYPE_POINT, GA_TYPE_VECTOR, GA_TYPE_NORMAL };
	return (allowable_type.find(attrib->getTypeInfo()) != -1) ? true : false;
}

static void
genPointAttribList(void *data, PRM_Name *menu_entries, 
				   int menu_size, const PRM_SpareData*, const PRM_Parm *parm)
{
	SOP_PointDeformByPrim *sop = (SOP_PointDeformByPrim *)data;
	if (!sop || !sop->getInput(1))
		return;

	if (parm->getTokenRef() == "pieceattrib")
		sop->fillAttribNameMenu(
			menu_entries, menu_size, GA_ATTRIB_PRIMITIVE, 1, sopApproveStringIntAttribs);
	else if (parm->getTokenRef() == "attribs")
		sop->fillAttribNameMenu(
			menu_entries, menu_size, GA_ATTRIB_POINT, 0, sopApproveVectorAttribs);
}

PRM_ChoiceList SOP_PointDeformByPrim::s_pointattriblist(
    PRM_ChoiceListType::PRM_CHOICELIST_TOGGLE, &genPointAttribList);

PRM_Template *
SOP_PointDeformByPrim::buildTemplates()
{
	static PRM_TemplateBuilder templ("SOP_PointDeformByPrim.cpp"_sh, theDsFile);
	if (templ.justBuilt())
	{
		templ.setChoiceListPtr("group", &SOP_Node::pointGroupMenu);
		templ.setChoiceListPtr("pieceattrib", &SOP_PointDeformByPrim::s_pointattriblist);
		templ.setChoiceListPtr("attribs", &SOP_PointDeformByPrim::s_pointattriblist);
	}
	return templ.templates();
}

OP_Node*
SOP_PointDeformByPrim::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
	return new SOP_PointDeformByPrim(net, name, op);
}

SOP_PointDeformByPrim::SOP_PointDeformByPrim(OP_Network *net, const char *name, OP_Operator *op)
	: SOP_Node(net, name, op)
{
	mySopFlags.setManagesDataIDs(true);
}

SOP_PointDeformByPrim::~SOP_PointDeformByPrim()
{
}

OP_ERROR
SOP_PointDeformByPrim::cookMySop(OP_Context &context)
{
	return cookMyselfAsVerb(context);
}

int SOP_PointDeformByPrim::isRefInput(unsigned i) const
{
	return false;
}

class SOP_PointDeformByPrimVerb : public SOP_NodeVerb
{
public:
    SOP_PointDeformByPrimVerb() {}
	virtual ~SOP_PointDeformByPrimVerb() {}

	virtual SOP_NodeParms *allocParms() const { return new SOP_PointDeformByPrimParms(); }
	virtual UT_StringHolder name() const { return SOP_PointDeformByPrim::theSOPTypeName; }

	virtual CookMode cookMode(const SOP_NodeParms *parms) const { return COOK_GENERIC; }
	virtual void cook(const CookParms &cookparms) const;

	static const SOP_NodeVerb::Register<SOP_PointDeformByPrimVerb> theVerb;

private:
	const char *currentParmsValue(const CookParms &cookparms) const;

	template<typename T, typename V>
	void captureClosestPointByPieceAttrib(const GU_Detail *rest_gdp,
										  V pieceAttrib_h,
										  V restPrimPieceAttrib_h,
										  ThreadedPointDeform &thread_ptdeform) const;

	void captureByPieceAttrib(GU_Detail *gdp,
							  const CookParms &cookparms,
							  const GA_AttributeOwner &attribOwner,
							  ThreadedPointDeform &thread_ptdeform) const;

};

const SOP_NodeVerb::Register<SOP_PointDeformByPrimVerb> SOP_PointDeformByPrimVerb::theVerb;

const SOP_NodeVerb*
SOP_PointDeformByPrim::cookVerb() const
{
	return SOP_PointDeformByPrimVerb::theVerb.get();
}

const char*
SOP_PointDeformByPrimVerb::currentParmsValue(const CookParms &cookparms) const
{
	const SOP_PointDeformByPrimParms &sopparms = cookparms.parms<SOP_PointDeformByPrimParms>();

	UT_OStringStream oss;
	oss << 
		sopparms.getGroup() <<
		UTstatic_cast(int32, sopparms.getMode()) <<
		sopparms.getCaptureAttribsHalf() <<
		sopparms.getDriveByAttribs() <<
		sopparms.getNormalAttrib() <<
		sopparms.getUpAttrib() <<
		sopparms.getRadius() <<
		sopparms.getPieceAttrib() <<
		sopparms.getPreSeparatePieces();

	return oss.str().buffer();
}

template<typename T, typename V>
void
SOP_PointDeformByPrimVerb::captureClosestPointByPieceAttrib(const GU_Detail *rest_gdp,
															V pieceAttrib_h,
															V restPrimPieceAttrib_h,
															ThreadedPointDeform &thread_ptdeform) const
{
	MapPrimGroup<T> primGroupMap;
	GA_Range restPrimRange(std::move(rest_gdp->getPrimitiveRange()));

	T attribValue;
	for (GA_Iterator primIt(restPrimRange.begin()); primIt != restPrimRange.end(); ++primIt)
	{
		attribValue = restPrimPieceAttrib_h.get(*primIt);
		if (primGroupMap.Map.find(attribValue) == primGroupMap.Map.end())
		{
			primGroupMap.Map[attribValue] = rest_gdp->newDetachedPrimitiveGroup();
			primGroupMap.Map.at(attribValue)->addOffset(rest_gdp->primitiveOffset(*primIt));
		}
		else
			primGroupMap.Map.at(attribValue)->addOffset(*primIt);
	}

	MapRay<T> restPrimRays;
	for (auto &kv : primGroupMap.Map)
	{
		GU_RayIntersect *ray = new GU_RayIntersect(rest_gdp, kv.second, true, false, true);
		restPrimRays.Map[kv.first] = ray;
	}

	thread_ptdeform.captureClosestPointByPieceAttrib(pieceAttrib_h, restPrimRays);

	for (auto &kv : restPrimRays.Map)
		delete restPrimRays.Map.at(kv.first);

	for (auto &kv : primGroupMap.Map)
		primGroupMap.Map.at(kv.first)->clearEntries();
}

void
SOP_PointDeformByPrimVerb::captureByPieceAttrib(GU_Detail *gdp,
												const CookParms &cookparms,
												const GA_AttributeOwner &attribOwner,
												ThreadedPointDeform &thread_ptdeform) const
{
	auto &&sopparms = cookparms.parms<SOP_PointDeformByPrimParms>();
	const GU_Detail *rest_gdp = cookparms.inputGeo(1);
	const GU_Detail *deformed_gdp = cookparms.inputGeo(2);

	const UT_StringHolder &pieceAttribParmData = sopparms.getPieceAttrib();
	const GA_Attribute *restPrimPieceAttrib = rest_gdp->findPrimitiveAttribute(pieceAttribParmData);
	const GA_AttributeType &restPrimPieceAttribType = restPrimPieceAttrib->getType();

	if (restPrimPieceAttribType.getTypeName() == "string")
	{
		GA_Attribute *pieceAttrib = gdp->findStringTuple(attribOwner, pieceAttribParmData, 1, 1);
		GA_ROHandleS pieceAttrib_h(pieceAttrib);
		GA_ROHandleS restPrimPieceAttrib_h(restPrimPieceAttrib);

		if (pieceAttrib_h.isValid() && restPrimPieceAttrib_h.isValid())
			captureClosestPointByPieceAttrib<UT_StringHolder, GA_ROHandleS>(rest_gdp, pieceAttrib_h, restPrimPieceAttrib_h, thread_ptdeform);
		else
		{
			cookparms.sopAddError(SOP_MESSAGE, "Only string/integer type is allowed for Piece attribute!\n");
			return;
		}
	}
	else if (restPrimPieceAttribType.getTypeName() == "numeric")
	{
		GA_Attribute *pieceAttrib = gdp->findIntTuple(attribOwner, pieceAttribParmData, 1, 1);
		GA_ROHandleI pieceAttrib_h(pieceAttrib);
		GA_ROHandleI restPrimPieceAttrib_h(restPrimPieceAttrib);

		if (pieceAttrib_h.isValid() && restPrimPieceAttrib_h.isValid())
			captureClosestPointByPieceAttrib<int32, GA_ROHandleI>(rest_gdp, pieceAttrib_h, restPrimPieceAttrib_h, thread_ptdeform);
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

void
SOP_PointDeformByPrimVerb::cook(const CookParms &cookparms) const
{
	const GU_Detail *base_gdp = cookparms.inputGeo(0);
	const GU_Detail *rest_gdp = cookparms.inputGeo(1);
	const GU_Detail *deformed_gdp = cookparms.inputGeo(2);
    if (base_gdp->isEmpty() || rest_gdp->isEmpty() || !rest_gdp->getNumPrimitives())
    {
        cookparms.sopAddError(SOP_MESSAGE, "First/Second input should contain valid geometry!\n");
        return;
    }

    DEP_MicroNodeList inputsdep;
    cookparms.depnode()->getInputs(inputsdep);
    if (inputsdep[0]->isTimeDependent() || inputsdep[1]->isTimeDependent())
    {
		cookparms.sopAddError(SOP_MESSAGE, "First/Second input cannot be time dependent!\n");
		return;
    }

	auto &&sopparms = cookparms.parms<SOP_PointDeformByPrimParms>();
	GU_Detail *gdp = cookparms.gdh().gdpNC();

	if (!deformed_gdp || deformed_gdp->isEmpty())
		return;

	if (rest_gdp->getNumPrimitives() != deformed_gdp->getNumPrimitives())
	{
		cookparms.sopAddWarning(SOP_MESSAGE, "Rest/deformed geometry cannot have different topology!\n");
		return;
	}

	const GA_Primitive *rest_prim = rest_gdp->getPrimitive(rest_gdp->primitiveOffset(0));
	const GA_Primitive *deformed_prim = deformed_gdp->getPrimitive(deformed_gdp->primitiveOffset(0));
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
	const UT_StringHolder &group_parm = sopparms.getGroup();
	const int32 mode_parm = UTstatic_cast(int32, sopparms.getMode());
	const bool half_float_parm = sopparms.getCaptureAttribsHalf();
	const bool drive_by_attribs = sopparms.getDriveByAttribs();
	const UT_StringHolder &normal_attrib_parm = sopparms.getNormalAttrib();
	const UT_StringHolder &up_attrib_parm = sopparms.getUpAttrib();
	const fpreal32 radius_parm = sopparms.getRadius();
	const UT_StringHolder &piece_parm = sopparms.getPieceAttrib();
	const bool pre_separate_parm = sopparms.getPreSeparatePieces();
    const bool rigid_proj_parm = sopparms.getRigidProjection();
    const bool update_nmls_parm = sopparms.getUpdateAffectedNmls();
    const UT_StringHolder &attribs_parm = sopparms.getAttribs();
	
	// evaluation for reinitialization
	const UT_StringHolder &parms_value_name("__parms_value");
	const UT_StringHolder &base_meta_count_name("__base_meta_count");
	const UT_StringHolder &rest_meta_count_name("__rest_meta_count");
	GA_Attribute *parms_value_attrib;
	GA_Attribute *base_meta_count_attrib;
	GA_Attribute *rest_meta_count_attrib;
	GA_RWHandleS parms_value_h;
	GA_RWHandleI base_meta_count_h;
	GA_RWHandleI rest_meta_count_h;
	parms_value_attrib = gdp->findAttribute(GA_ATTRIB_DETAIL, parms_value_name);
	base_meta_count_attrib = gdp->findAttribute(GA_ATTRIB_DETAIL, base_meta_count_name);
	rest_meta_count_attrib = gdp->findAttribute(GA_ATTRIB_DETAIL, rest_meta_count_name);

	bool reinitialize = false;
	const UT_StringHolder &cur_parms_value = currentParmsValue(cookparms);
	if (parms_value_attrib &&
		base_meta_count_attrib &&
		rest_meta_count_attrib)
	{
		parms_value_h.bind(parms_value_attrib);
		base_meta_count_h.bind(base_meta_count_attrib);
		rest_meta_count_h.bind(rest_meta_count_attrib);

		reinitialize = cur_parms_value.compare(parms_value_h.get(0)) != 0 ||
			base_meta_count_h.get(0) != base_gdp->getMetaCacheCount() ||
			rest_meta_count_h.get(0) != rest_gdp->getMetaCacheCount();
	}
	else
		reinitialize = true;

	// create/get deformation attribs
	const UT_StringHolder &rest_xform_name("__rest_xform");
	const UT_StringHolder &rest_p_name("__rest_p");
	const UT_StringHolder &hit_prim_name("__hit_prim");
	const UT_StringHolder &hit_uv_name("__hit_uv");
	HitAttributes hit_attribs;

    if (reinitialize)
    {
	    gdp->replaceWith(*base_gdp);

		parms_value_attrib = gdp->addStringTuple(GA_ATTRIB_DETAIL, parms_value_name, 1);
		base_meta_count_attrib = gdp->addIntTuple(GA_ATTRIB_DETAIL, base_meta_count_name, 1);
		rest_meta_count_attrib = gdp->addIntTuple(GA_ATTRIB_DETAIL, rest_meta_count_name, 1);
		parms_value_h.bind(parms_value_attrib);
		base_meta_count_h.bind(base_meta_count_attrib);
		rest_meta_count_h.bind(rest_meta_count_attrib);
		parms_value_h.set(0, cur_parms_value);
		base_meta_count_h.set(0, base_gdp->getMetaCacheCount());
		rest_meta_count_h.set(0, rest_gdp->getMetaCacheCount());

		const GA_Storage &storage= half_float_parm ? GA_STORE_REAL16 : GA_STORE_REAL32;
		hit_attribs.Xform = gdp->addFloatTuple(GA_ATTRIB_POINT, rest_xform_name, 9, (GA_Defaults)0.f, nullptr, nullptr, storage);
		hit_attribs.Xform->setTypeInfo(GA_TYPE_TRANSFORM);
		hit_attribs.RestP = gdp->addFloatTuple(GA_ATTRIB_POINT, rest_p_name, 3, (GA_Defaults)0.f, nullptr, nullptr, storage);
		hit_attribs.RestP->setTypeInfo(GA_TYPE_POINT);
		hit_attribs.Prim = gdp->addIntTuple(GA_ATTRIB_POINT, hit_prim_name, 1);
		hit_attribs.Prim->setTypeInfo(GA_TYPE_NONARITHMETIC_INTEGER);
		hit_attribs.UV = gdp->addFloatTuple(GA_ATTRIB_POINT, hit_uv_name, 2, (GA_Defaults)0.f, nullptr, nullptr, storage);
		hit_attribs.UV->setTypeInfo(GA_TYPE_VECTOR);
    }
    else
    {
		hit_attribs.Xform = gdp->findAttribute(GA_ATTRIB_POINT, rest_xform_name);
		hit_attribs.RestP = gdp->findAttribute(GA_ATTRIB_POINT, rest_p_name);
		hit_attribs.Prim = gdp->findAttribute(GA_ATTRIB_POINT, hit_prim_name);
		hit_attribs.UV = gdp->findAttribute(GA_ATTRIB_POINT, hit_uv_name);
    }

	parms_value_attrib->bumpDataId();
	base_meta_count_attrib->bumpDataId();
	rest_meta_count_attrib->bumpDataId();
	hit_attribs.Xform->bumpDataId();
	hit_attribs.RestP->bumpDataId();
	hit_attribs.Prim->bumpDataId();
	hit_attribs.UV->bumpDataId();

	// create an array of attribs to interpolate point/vertex based on attribParm
	// attribs_to_interpolate is part of Utils.h
	AttribsToInterpolate attribs_to_interpolate;
	attribs_to_interpolate.BasePAttrib = base_gdp->getP();
	attribs_to_interpolate.PAttrib = gdp->getP();

	const GA_AttributeSet& base_attribs = base_gdp->getAttributes();
	const GA_AttributeSet& attribs = gdp->getAttributes();

	if (attribs_parm == "*")
	{
		UT_Array<GA_TypeInfo> allowable_type{ GA_TYPE_POINT, GA_TYPE_VECTOR, GA_TYPE_NORMAL };
		UT_Array<UT_StringHolder> excluding_names{ "P", rest_p_name, rest_xform_name, hit_uv_name};

		const GA_AttributeDict &base_point_attribs = base_attribs.getDict(GA_ATTRIB_POINT);
		for (GA_AttributeDict::iterator pit(base_point_attribs.begin()); pit != base_point_attribs.end(); ++pit)
		{
			const GA_Attribute *cur_attrib = pit.attrib();

			if (cur_attrib && excluding_names.find(cur_attrib->getFullName()) == -1)
				if (allowable_type.find(cur_attrib->getTypeInfo()) != -1)
					attribs_to_interpolate.BasePtAttribs.emplace_back(cur_attrib);
		}

		const GA_AttributeDict &point_attribs = attribs.getDict(GA_ATTRIB_POINT);
		for (GA_AttributeDict::iterator pit(point_attribs.begin()); pit != point_attribs.end(); ++pit)
		{
			GA_Attribute *cur_attrib = pit.attrib();
	
			if (cur_attrib && excluding_names.find(cur_attrib->getFullName()) == -1)
				if (allowable_type.find(cur_attrib->getTypeInfo()) != -1)
					attribs_to_interpolate.PtAttribs.emplace_back(cur_attrib);
		}

		const GA_AttributeDict &base_vtx_attribs = base_attribs.getDict(GA_ATTRIB_VERTEX);
		for (GA_AttributeDict::iterator vit(base_vtx_attribs.begin()); vit != base_vtx_attribs.end(); ++vit)
		{
			const GA_Attribute *cur_attrib = vit.attrib();

			if (cur_attrib && allowable_type.find(cur_attrib->getTypeInfo()) != -1)
				attribs_to_interpolate.BaseVtxAttribs.emplace_back(cur_attrib);
		}
	
		const GA_AttributeDict &vertex_attribs = attribs.getDict(GA_ATTRIB_VERTEX);
		for (GA_AttributeDict::iterator vit(vertex_attribs.begin()); vit != vertex_attribs.end(); ++vit)
		{
			GA_Attribute *cur_attrib = vit.attrib();
	
			if (cur_attrib && allowable_type.find(cur_attrib->getTypeInfo()) != -1)
				attribs_to_interpolate.VtxAttribs.emplace_back(cur_attrib);
		}
	}
	else
	{
		// TODO implement parsing attribs
	}

	GOP_Manager group_parser;
	bool success = false;
	const GA_PointGroup *point_group = group_parser.parsePointDetached(group_parm, base_gdp, false, success);

	if (group_parm && !success)
		cookparms.sopAddWarning(SOP_ERR_BADGROUP, group_parm);

	DriveAttribHandles drive_attrib_hs;
	if (drive_by_attribs)
	{
		const GA_Attribute *rest_normal_attrib = rest_gdp->findAttribute(GA_ATTRIB_POINT, normal_attrib_parm);
		const GA_Attribute *rest_up_attrib = rest_gdp->findAttribute(GA_ATTRIB_POINT, up_attrib_parm);
		const GA_Attribute *deformed_normal_attrib = deformed_gdp->findAttribute(GA_ATTRIB_POINT, normal_attrib_parm);
		const GA_Attribute *deformed_up_attrib = deformed_gdp->findAttribute(GA_ATTRIB_POINT, up_attrib_parm);

		if (rest_normal_attrib && rest_up_attrib &&
			deformed_normal_attrib && deformed_up_attrib)
		{
			drive_attrib_hs.Drive = true;
			drive_attrib_hs.RestNormal_H = rest_normal_attrib;
			drive_attrib_hs.RestUp_H = rest_up_attrib;
			drive_attrib_hs.DeformedNormal_H = deformed_normal_attrib;
			drive_attrib_hs.DeformedUp_H = deformed_up_attrib;
		}
		else
		{
			cookparms.sopAddError(SOP_MESSAGE, "Rest or Deformed geometry stream doesn't have Normal/Up vector!\n");
			return;
		}
	}

	GA_SplittableRange ptrange(std::move(gdp->getPointRange(point_group)));
    ThreadedPointDeform thread_ptdeform(
		gdp, base_gdp, rest_gdp, deformed_gdp, std::move(ptrange), std::move(drive_attrib_hs), hit_attribs, attribs_to_interpolate);
	
    if (reinitialize)
    {
        if (piece_parm)
        {
			if (rest_gdp->findPrimitiveAttribute(piece_parm))
			{
				if (gdp->findAttribute(GA_ATTRIB_PRIMITIVE, piece_parm))
				{
					captureByPieceAttrib(gdp, cookparms, GA_ATTRIB_PRIMITIVE, thread_ptdeform);
				}
				else if (gdp->findAttribute(GA_ATTRIB_POINT, piece_parm))
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
            GU_RayIntersect ray_rest(rest_gdp, nullptr, true, false, true);
            thread_ptdeform.captureClosestPoint(ray_rest);
        }
	
		hit_attribs.Xform->bumpDataId();
		hit_attribs.RestP->bumpDataId();
		hit_attribs.Prim->bumpDataId();
		hit_attribs.UV->bumpDataId();
	
		for (GA_Attribute *pt_attrib : attribs_to_interpolate.PtAttribs)
			pt_attrib->bumpDataId();
		for (GA_Attribute *vtx_attrib : attribs_to_interpolate.VtxAttribs)
			vtx_attrib->bumpDataId();
    }

	if (mode_parm)
		return;
	
	thread_ptdeform.computeDeformation(rigid_proj_parm);
	attribs_to_interpolate.PAttrib->bumpDataId();
	
	for (GA_Attribute *pt_attrib : attribs_to_interpolate.PtAttribs)
		pt_attrib->bumpDataId();
	for (GA_Attribute *vtx_attrib : attribs_to_interpolate.VtxAttribs)
		vtx_attrib->bumpDataId();
}