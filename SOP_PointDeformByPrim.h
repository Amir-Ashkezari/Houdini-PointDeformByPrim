#pragma once

#ifndef __SOP_PointDeformByPrim_h__
#define __SOP_PointDeformByPrim_h__

#include <SOP/SOP_Node.h>
#include <UT/UT_StringHolder.h>

namespace AKA
{

class SOP_PointDeformByPrim : public SOP_Node
{
public:
	static const UT_StringHolder theSOPTypeName;

	static PRM_Template* buildTemplates();
	static OP_Node* myConstructor(OP_Network* net, const char* name, OP_Operator* op);

	const SOP_NodeVerb* cookVerb() const override;

protected:
	SOP_PointDeformByPrim(OP_Network* net, const char* name, OP_Operator* op);
	~SOP_PointDeformByPrim();

	virtual const char* inputLabel(unsigned idx) const override;

	OP_ERROR cookMySop(OP_Context& context) override;

private:
	static void genPointAttribList(void* thedata,
								   PRM_Name* thechoicenames,
								   int thelistsize,
								   const PRM_SpareData* thespareptr,
								   const PRM_Parm* theparm);

	static PRM_ChoiceList s_thePointAttribList;

};

} // end AKA

#endif