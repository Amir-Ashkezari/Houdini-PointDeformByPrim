#include "SOP_PointDeformByPrim.h"
#include "SOP_PointDeformByPrim.proto.h"
#include "ThreadedPointDeform.h"
#include "Utils.h"

#include <SOP/SOP_NodeVerb.h>
#include <GU/GU_Detail.h>
#include <DEP/DEP_MicroNode.h>
#include <GU/GU_RayIntersect.h>
#include <GA/GA_Handle.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_TemplateBuilder.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Assert.h>
#include <UT/UT_SysSpecific.h>
#include <SYS/SYS_Math.h>


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

	op->setOpTabSubMenuPath("Custom");
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
	groupsimple {
        name    "capture_folder"
        label   "Capture"

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

			disablewhen "{ drivebyattribs == 0 }"
		}
		parm {
			name    "upattrib"
			cppname "UpAttrib"
			label   "Up Attrib"
			type    string
			default { "up" }
			help    "Up vector attribute from Rest/Deformed geomerty streams for constructing transformation matrix."

			disablewhen "{ drivebyattribs == 0 }"
		}
		parm {
			name    "sepparm1"
			cppname "SepParm1"
			type    separator

			default { "" }
		}
		parm {
			name    "multiplesamples"
			cppname "MultipleSamples"
			label   "Multiple Samples"
			type    toggle
			default { "0" }
			help    "Shooting multiple samples to find an average position based on weighted distance."

			disablewhen "{ drivebyattribs == 1 }"
		}
		parm {
            name    "mindistthresh"
			cppname "MinDistThresh"
            label   "Minimum Distance Threshold"
            type    float
            default { "0.001" }
            range   { 0.00001 0.1 }
			help    "The minimum distance (Houdini world units) away from a surface position to stop shooting multiple rays."

			disablewhen "{ multiplesamples == 0 }"
        }
        parm {
            name    "pieceattrib"
            cppname "PieceAttrib"
            label   "Piece Attribute"
            type    string
            default { "" }
			help    "The name of a string or integer point attribute to use to treat the geometry as separate pieces. The attribute must be present on both the mesh and rest lattice. Points with the same value in this attribute are considered part of the same 'piece'. When you specify a valid piece attribute, this node deforms each piece using only the lattice points with the same piece value. \nThis lets you deform independent objects (pieces) in a single pass. You can create a piece attribute based on connectivity with the Connectivity SOP."
        }
	}
	groupsimple {
        name    "deform_folder"
        label   "Deform"

        parm {
            name    "attribs"
            cppname "Attribs"
            label   "Attributes to Transform"
            type    string
            default { "*" }
            help    "A space-separated list of attribute names/patterns, specifying which attributes are transformed by the deformation. The default is *, meaning all attributes. The node modifies vector attributes according to their type info, as points, vectors, or normals."
        }
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
queryPointAttribMenu(void *data, 
				   PRM_Name *menu_entries, 
				   int menu_size, 
				   const PRM_SpareData*, 
				   const PRM_Parm *parm)
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

PRM_ChoiceList SOP_PointDeformByPrim::s_PieceAttribMenu(
    PRM_ChoiceListType::PRM_CHOICELIST_REPLACE, &queryPointAttribMenu);
PRM_ChoiceList SOP_PointDeformByPrim::s_AttribsMenu(
	PRM_ChoiceListType::PRM_CHOICELIST_TOGGLE, &queryPointAttribMenu);

PRM_Template *
SOP_PointDeformByPrim::buildTemplates()
{
	static PRM_TemplateBuilder templ("SOP_PointDeformByPrim.cpp"_sh, theDsFile);
	if (templ.justBuilt())
	{
		templ.setChoiceListPtr("group", &SOP_Node::pointGroupMenu);
		templ.setChoiceListPtr("pieceattrib", &SOP_PointDeformByPrim::s_PieceAttribMenu);
		templ.setChoiceListPtr("attribs", &SOP_PointDeformByPrim::s_AttribsMenu);
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
	void constructRayGroups(const Gdps &gdps,
							V pieceattrib_h, 
							V rest_prim_pieceattrib_h, 
							ThreadedPointDeform &threaded_ptdeform) const;

	void findPieceAttrib(const Gdps &gdps,
						 const CookParms &cookparms, 
						 const GA_AttributeOwner &attrib_owner, 
						 ThreadedPointDeform &threaded_ptdeform) const;

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
		sopparms.getDriveByAttribs() <<
		sopparms.getNormalAttrib() <<
		sopparms.getUpAttrib() <<
		sopparms.getMultipleSamples() <<
		sopparms.getMinDistThresh() <<
		sopparms.getPieceAttrib() <<
		sopparms.getAttribs();

	return oss.str().buffer();
}

template<typename T, typename V>
void
SOP_PointDeformByPrimVerb::constructRayGroups(const Gdps &gdps,
											  V pieceattrib_h,
											  V rest_prim_pieceattrib_h,
											  ThreadedPointDeform &threaded_ptdeform) const
{
	MapPrimGroup<T> primgrp_map;
	GA_Range rest_primrange(std::move(gdps.RestGdp->getPrimitiveRange()));

	T attrib_value;
	for (GA_Iterator primitr(rest_primrange.begin()); primitr != rest_primrange.end(); ++primitr)
	{
		attrib_value = rest_prim_pieceattrib_h.get(*primitr);
		if (primgrp_map.Map.find(attrib_value) == primgrp_map.Map.end())
		{
			primgrp_map.Map[attrib_value] = gdps.RestGdp->newDetachedPrimitiveGroup();
			primgrp_map.Map.at(attrib_value)->addOffset(gdps.RestGdp->primitiveOffset(*primitr));
		}
		else
			primgrp_map.Map.at(attrib_value)->addOffset(*primitr);
	}

	MapRay<T> restprim_rays;
	for (auto &kv : primgrp_map.Map)
	{
		GU_RayIntersect *ray = new GU_RayIntersect(gdps.RestGdp, kv.second, true, false, true);
		restprim_rays.Map[kv.first] = ray;
	}

	threaded_ptdeform.captureByPieceAttrib(pieceattrib_h, restprim_rays);

	for (auto &kv : restprim_rays.Map)
		delete restprim_rays.Map.at(kv.first);

	for (auto &kv : primgrp_map.Map)
		primgrp_map.Map.at(kv.first)->clearEntries();
}

void
SOP_PointDeformByPrimVerb::findPieceAttrib(const Gdps &gdps, 
										   const CookParms &cookparms, 
										   const GA_AttributeOwner &attrib_owner, 
										   ThreadedPointDeform &threaded_ptdeform) const
{
	auto &&sopparms = cookparms.parms<SOP_PointDeformByPrimParms>();

	const UT_StringHolder &pieceattrib_parmdata = sopparms.getPieceAttrib();
	const GA_Attribute *restprim_pieceattrib = gdps.RestGdp->findPrimitiveAttribute(pieceattrib_parmdata);
	const GA_AttributeType &restprim_pieceattribtype = restprim_pieceattrib->getType();

	if (restprim_pieceattribtype.getTypeName() == "string")
	{
		GA_Attribute *pieceattrib = gdps.Gdp->findStringTuple(attrib_owner, pieceattrib_parmdata, 1, 1);
		GA_ROHandleS pieceattrib_h(pieceattrib);
		GA_ROHandleS rest_prim_pieceattrib_h(restprim_pieceattrib);

		if (pieceattrib_h.isValid() && rest_prim_pieceattrib_h.isValid())
			constructRayGroups<UT_StringHolder, GA_ROHandleS>(gdps, pieceattrib_h, rest_prim_pieceattrib_h, threaded_ptdeform);
		else
		{
			cookparms.sopAddError(SOP_MESSAGE, "Only string/integer type is allowed for Piece attribute!\n");
			return;
		}
	}
	else if (restprim_pieceattribtype.getTypeName() == "numeric")
	{
		GA_Attribute *pieceattrib = gdps.Gdp->findIntTuple(attrib_owner, pieceattrib_parmdata, 1, 1);
		GA_ROHandleI pieceattrib_h(pieceattrib);
		GA_ROHandleI rest_prim_pieceattrib_h(restprim_pieceattrib);

		if (pieceattrib_h.isValid() && rest_prim_pieceattrib_h.isValid())
			constructRayGroups<int32, GA_ROHandleI>(gdps, pieceattrib_h, rest_prim_pieceattrib_h, threaded_ptdeform);
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
	Gdps gdps;
	gdps.BaseGdp = cookparms.inputGeo(0);
	gdps.RestGdp = cookparms.inputGeo(1);
	gdps.DeformedGdp = cookparms.inputGeo(2);

    if (gdps.BaseGdp->isEmpty() || gdps.RestGdp->isEmpty() || !gdps.RestGdp->getNumPrimitives())
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
	gdps.Gdp = cookparms.gdh().gdpNC();

	if (!gdps.DeformedGdp || gdps.DeformedGdp->isEmpty())
		return;

	if (gdps.RestGdp->getNumPrimitives() != gdps.DeformedGdp->getNumPrimitives())
	{
		cookparms.sopAddWarning(SOP_MESSAGE, "Rest/deformed geometry cannot have different topology!\n");
		return;
	}

	const GA_Primitive *rest_prim = gdps.RestGdp->getPrimitive(gdps.RestGdp->primitiveOffset(0));
	const GA_Primitive *deformed_prim = gdps.DeformedGdp->getPrimitive(gdps.DeformedGdp->primitiveOffset(0));
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
	const bool drivebyattribs_parm = sopparms.getDriveByAttribs();
	const UT_StringHolder &normalattrib_parm = sopparms.getNormalAttrib();
	const UT_StringHolder &upattrib_parm = sopparms.getUpAttrib();
	const bool multisamples_parm = sopparms.getMultipleSamples();
	const fpreal32 mindistthresh_parm = sopparms.getMinDistThresh();
	const UT_StringHolder &piece_parm = sopparms.getPieceAttrib();
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
	parms_value_attrib = gdps.Gdp->findAttribute(GA_ATTRIB_DETAIL, parms_value_name);
	base_meta_count_attrib = gdps.Gdp->findAttribute(GA_ATTRIB_DETAIL, base_meta_count_name);
	rest_meta_count_attrib = gdps.Gdp->findAttribute(GA_ATTRIB_DETAIL, rest_meta_count_name);

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
			base_meta_count_h.get(0) != gdps.BaseGdp->getMetaCacheCount() ||
			rest_meta_count_h.get(0) != gdps.RestGdp->getMetaCacheCount();
	}
	else
		reinitialize = true;

	// create/get deformation attribsCaptureAttributes
	const UT_StringHolder &rest_p_name("__rest_p");
	const UT_StringHolder &capture_prims_name("__capture_prims");
	const UT_StringHolder &capture_uvws_name("__capture_uvws");
	const UT_StringHolder &capture_weights_name("__capture_weights");
	const UT_StringHolder &capture_xform_name("__capture_xform");

	CaptureAttributes capture_attribs;
	CaptureAttributes_Info captureattribs_info;
	captureattribs_info.CaptureMultiSamples = multisamples_parm;
	captureattribs_info.CaptureMinDistThresh = mindistthresh_parm;

    if (reinitialize)
    {
	    gdps.Gdp->replaceWith(*gdps.BaseGdp);

		parms_value_attrib = gdps.Gdp->addStringTuple(GA_ATTRIB_DETAIL, parms_value_name, 1);
		base_meta_count_attrib = gdps.Gdp->addIntTuple(GA_ATTRIB_DETAIL, base_meta_count_name, 1);
		rest_meta_count_attrib = gdps.Gdp->addIntTuple(GA_ATTRIB_DETAIL, rest_meta_count_name, 1);
		parms_value_h.bind(parms_value_attrib);
		base_meta_count_h.bind(base_meta_count_attrib);
		rest_meta_count_h.bind(rest_meta_count_attrib);
		parms_value_h.set(0, cur_parms_value);
		base_meta_count_h.set(0, gdps.BaseGdp->getMetaCacheCount());
		rest_meta_count_h.set(0, gdps.RestGdp->getMetaCacheCount());

		capture_attribs.RestP = gdps.Gdp->addFloatTuple(GA_ATTRIB_POINT, rest_p_name, 3, (GA_Defaults)0.f, nullptr, nullptr, GA_STORE_REAL32);
		capture_attribs.RestP->setTypeInfo(GA_TYPE_POINT);
		capture_attribs.Prims = gdps.Gdp->addIntArray(GA_ATTRIB_POINT, capture_prims_name, 1);
		capture_attribs.Prims->setTypeInfo(GA_TYPE_NONARITHMETIC_INTEGER);
		capture_attribs.UVWs = gdps.Gdp->addFloatArray(GA_ATTRIB_POINT, capture_uvws_name, 2, nullptr, nullptr, GA_STORE_REAL16);
		capture_attribs.UVWs->setTypeInfo(GA_TYPE_VECTOR);
		capture_attribs.Weights = gdps.Gdp->addFloatArray(GA_ATTRIB_POINT, capture_weights_name, 1, nullptr, nullptr, GA_STORE_REAL16);
    }
    else
    {
		capture_attribs.RestP = gdps.Gdp->findAttribute(GA_ATTRIB_POINT, rest_p_name);
		capture_attribs.Prims = gdps.Gdp->findAttribute(GA_ATTRIB_POINT, capture_prims_name);
		capture_attribs.UVWs = gdps.Gdp->findAttribute(GA_ATTRIB_POINT, capture_uvws_name);
		capture_attribs.Weights = gdps.Gdp->findAttribute(GA_ATTRIB_POINT, capture_weights_name);
    }

	captureattribs_info.RestP_H.bind(capture_attribs.RestP);
	captureattribs_info.CapturePrims_H.bind(capture_attribs.Prims);
	captureattribs_info.CaptureUVWs_H.bind(capture_attribs.UVWs);
	captureattribs_info.CaptureWeights_H.bind(capture_attribs.Weights);

	// based on the attribs parameter,
	// find any vector attribs to interpolate
	UT_Array<UT_StringHolder> attribnames_to_interpolate;

	if (attribs_parm)
	{
		UT_Array<GA_TypeInfo> types{ GA_TYPE_VECTOR, GA_TYPE_NORMAL };

		if (attribs_parm == "*")
		{
			const GA_AttributeDict &base_point_attribs = gdps.BaseGdp->getAttributes().getDict(GA_ATTRIB_POINT);
			for (GA_AttributeDict::iterator pit(base_point_attribs.begin()); pit != base_point_attribs.end(); ++pit)
			{
				const GA_Attribute *cur_attrib = pit.attrib();
				if (cur_attrib && cur_attrib->getFullName() != "P" && 
					cur_attrib->getTupleSize() == 3 && types.find(cur_attrib->getTypeInfo()) != -1)
					attribnames_to_interpolate.emplace_back(cur_attrib->getFullName());
			}
		}
		else
		{
			std::stringstream ss(attribs_parm.buffer());
			std::string attribname;

			while (getline(ss, attribname, ' '))
			{
				if (attribname != "P")
				{
					const GA_Attribute *cur_attrib = gdps.BaseGdp->findAttribute(GA_ATTRIB_POINT, attribname.c_str());
					if (cur_attrib && cur_attrib->getTupleSize() == 3 && 
						types.find(cur_attrib->getTypeInfo()) != -1)
						attribnames_to_interpolate.emplace_back(attribname);
				}
			}
		}

		if (attribnames_to_interpolate.size())
		{
			captureattribs_info.XformRequired = true;

			if (reinitialize)
			{
				capture_attribs.Xform = gdps.Gdp->addFloatTuple(
					GA_ATTRIB_POINT, capture_xform_name, 9, (GA_Defaults)0.f, nullptr, nullptr, GA_STORE_REAL16);
				capture_attribs.Xform->setTypeInfo(GA_TYPE_TRANSFORM);
			}
			else
				capture_attribs.Xform = gdps.Gdp->findAttribute(GA_ATTRIB_POINT, capture_xform_name);

			capture_attribs.Xform->bumpDataId();
			captureattribs_info.Xform_H.bind(capture_attribs.Xform);
		}
		else
		{
			if (gdps.Gdp->findAttribute(GA_ATTRIB_POINT, capture_xform_name))
				gdps.Gdp->destroyAttribute(GA_ATTRIB_POINT, capture_xform_name);
		}
	}
	else
	{
		if (gdps.Gdp->findAttribute(GA_ATTRIB_POINT, capture_xform_name))
			gdps.Gdp->destroyAttribute(GA_ATTRIB_POINT, capture_xform_name);
	}

	parms_value_attrib->bumpDataId();
	base_meta_count_attrib->bumpDataId();
	rest_meta_count_attrib->bumpDataId();
	capture_attribs.RestP->bumpDataId();
	capture_attribs.Prims->bumpDataId();
	capture_attribs.UVWs->bumpDataId();
	capture_attribs.Weights->bumpDataId();

	GOP_Manager group_parser;
	bool success = false;
	const GA_PointGroup *point_group = group_parser.parsePointDetached(group_parm, gdps.BaseGdp, false, success);

	if (group_parm && !success)
		cookparms.sopAddWarning(SOP_ERR_BADGROUP, group_parm);

	DriveAttrib_Info drive_attrib_hs;
	if (drivebyattribs_parm)
	{
		const GA_Attribute *rest_normal_attrib = gdps.RestGdp->findAttribute(GA_ATTRIB_POINT, normalattrib_parm);
		const GA_Attribute *rest_up_attrib = gdps.RestGdp->findAttribute(GA_ATTRIB_POINT, upattrib_parm);
		const GA_Attribute *deformed_normal_attrib = gdps.DeformedGdp->findAttribute(GA_ATTRIB_POINT, normalattrib_parm);
		const GA_Attribute *deformed_up_attrib = gdps.DeformedGdp->findAttribute(GA_ATTRIB_POINT, upattrib_parm);

		if (rest_normal_attrib && rest_up_attrib &&
			deformed_normal_attrib && deformed_up_attrib)
		{
			drive_attrib_hs.Drive = true;
			drive_attrib_hs.RestNormal_H.bind(rest_normal_attrib);
			drive_attrib_hs.RestUp_H.bind(rest_up_attrib);
			drive_attrib_hs.DeformedNormal_H.bind(deformed_normal_attrib);
			drive_attrib_hs.DeformedUp_H.bind(deformed_up_attrib);
		}
		else
		{
			cookparms.sopAddError(SOP_MESSAGE, "Rest or Deformed geometry stream doesn't have Normal/Up vector!\n");
			return;
		}
	}

	GA_SplittableRange ptrange(std::move(gdps.Gdp->getPointRange(point_group)));
    ThreadedPointDeform threaded_ptdeform(gdps, &ptrange, &drive_attrib_hs, &captureattribs_info, attribnames_to_interpolate);
	
    if (reinitialize)
    {
        if (piece_parm)
        {
			if (gdps.RestGdp->findPrimitiveAttribute(piece_parm))
			{
				if (gdps.Gdp->findAttribute(GA_ATTRIB_PRIMITIVE, piece_parm))
					findPieceAttrib(gdps, cookparms, GA_ATTRIB_PRIMITIVE, threaded_ptdeform);
				else if (gdps.Gdp->findAttribute(GA_ATTRIB_POINT, piece_parm))
					findPieceAttrib(gdps, cookparms, GA_ATTRIB_POINT, threaded_ptdeform);
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
            GU_RayIntersect ray_rest(gdps.RestGdp, nullptr, true, false, true);
            threaded_ptdeform.capture(&ray_rest);
        }
	
		capture_attribs.RestP->bumpDataId();
		capture_attribs.Prims->bumpDataId();
		capture_attribs.UVWs->bumpDataId();
		capture_attribs.Weights->bumpDataId();
		if (captureattribs_info.XformRequired)
			capture_attribs.Xform->bumpDataId();
	
		for (UT_StringHolder &attribname : attribnames_to_interpolate)
			gdps.Gdp->findAttribute(GA_ATTRIB_POINT, attribname)->bumpDataId();
    }
		
	threaded_ptdeform.deform();
	gdps.Gdp->getP()->bumpDataId();

	for (UT_StringHolder &attribname : attribnames_to_interpolate)
		gdps.Gdp->findAttribute(GA_ATTRIB_POINT, attribname)->bumpDataId();
}