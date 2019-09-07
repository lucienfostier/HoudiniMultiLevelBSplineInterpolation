/*
 * Copyright (c) 2019
 *	Side Effects Software Inc.  All rights reserved.
 *
 * Redistribution and use of Houdini Development Kit samples in source and
 * binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. The name of Side Effects Software may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY SIDE EFFECTS SOFTWARE `AS IS' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL SIDE EFFECTS SOFTWARE BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *----------------------------------------------------------------------------
 */

/// This is the pure C++ implementation of the wave SOP.
/// @see @ref HOM/SOP_HOMWave.py, @ref HOM/SOP_HOMWaveNumpy.py, @ref HOM/SOP_HOMWaveInlinecpp.py, @ref HOM/SOP_HOMWave.C, @ref SOP/SOP_VEXWave.vfl

#include "SOP_CPPWave.h"
#include "mba.h"

#include <GU/GU_Detail.h>
#include <GA/GA_Handle.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_Director.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <UT/UT_DSOVersion.h>
#include <SYS/SYS_Math.h>

using namespace HDK_Sample;

void
newSopOperator(OP_OperatorTable *table)
{
     table->addOperator(new OP_Operator(
        "cpp_wave",
        "CPP Wave ",
        SOP_CPPWave::myConstructor,
        SOP_CPPWave::myTemplateList,
        3,
        3,
        0));
}

PRM_Template
SOP_CPPWave::myTemplateList[] = {
    PRM_Template(),
};

OP_Node *
SOP_CPPWave::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CPPWave(net, name, op);
}

SOP_CPPWave::SOP_CPPWave(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    // This indicates that this SOP manually manages its data IDs,
    // so that Houdini can identify what attributes may have changed,
    // e.g. to reduce work for the viewport, or other SOPs that
    // check whether data IDs have changed.
    // By default, (i.e. if this line weren't here), all data IDs
    // would be bumped after the SOP cook, to indicate that
    // everything might have changed.
    // If some data IDs don't get bumped properly, the viewport
    // may not update, or SOPs that check data IDs
    // may not cook correctly, so be *very* careful!
    mySopFlags.setManagesDataIDs(true);
}

SOP_CPPWave::~SOP_CPPWave()
{
}

const char  *SOP_CPPWave::inputLabel(unsigned idx) const
{
	switch( idx )
	{
		case 0: return "geometry";
		case 1: return "rest pose";
		case 2: return "rest animated";
		default: return SOP_Node::inputLabel( idx );
	}
}

OP_ERROR
SOP_CPPWave::cookMySop(OP_Context &context)
{
	std::vector<mba::point<2>> coo = {
	    {0.0, 0.0},
	    {0.0, 1.0},
	    {1.0, 0.0},
	    {1.0, 1.0},
	    {0.4, 0.4},
	    {0.6, 0.6}
	};
	mba::point<2> lo = {-0.1, -0.1};
	mba::point<2> hi = { 1.1,  1.1};

	unsigned int res = 3;
	mba::index<2> grid = {res,res};
	std::vector<double> val = {
	    0.01, 0.7, 0.05, 1, .12, -.12
	};

	mba::MBA<2> interp(lo, hi, grid, coo, val);

	// We must lock our inputs before we try to access their geometry.
	// OP_AutoLockInputs will automatically unlock our inputs when we return.
	// NOTE: Don't call unlockInputs yourself when using this!
	OP_AutoLockInputs inputs(this);
	if (inputs.lock(context) >= UT_ERROR_ABORT)
	return error();

	// Duplicate input geometry
	duplicateSource(0, context);

	const GU_Detail *restGdp =  inputGeo( 1 , context );
	const GU_Detail *restAnimatedGdp =  inputGeo( 2 , context );

	std::vector<mba::point<3>> cooRest;
	GA_Offset start, end;
	GA_ROHandleV3 restPhandle(restGdp->findAttribute(GA_ATTRIB_POINT, "P"));
	for( GA_Iterator it( restGdp->getPointRange() ); it.blockAdvance( start, end ); )
	{
		for( GA_Offset offset = start; offset < end; ++offset )
		{
			//std::cout<<"index"<<restPhandle.get( offset )<<std::endl;
			cooRest.push_back( { restPhandle.get( offset ).x(), restPhandle.get( offset ).y(), restPhandle.get( offset ).z() } );
		}
	}

	std::vector<double> restAnimatedX;	
	std::vector<double> restAnimatedY;	
	std::vector<double> restAnimatedZ;
	GA_ROHandleV3 restAnimatedPhandle(restAnimatedGdp->findAttribute(GA_ATTRIB_POINT, "P"));
	for( GA_Iterator it( restAnimatedGdp->getPointRange() ); it.blockAdvance( start, end ); )
	{
		for( GA_Offset offset = start; offset < end; ++offset )
		{
			UT_Vector3 Pvalue = restPhandle.get(offset);
			restAnimatedX.push_back( restAnimatedPhandle.get( offset ).x() - Pvalue.x()  );
			restAnimatedY.push_back( restAnimatedPhandle.get( offset ).y() - Pvalue.y() );
			restAnimatedZ.push_back( restAnimatedPhandle.get( offset ).z() - Pvalue.z() );
		}
	}
	
	mba::index<3> grid3 = {res,res, res};
	mba::point<3> lo3 = {-10.1, -10.1, -10.1};
	mba::point<3> hi3 = { 10.1,  10.1, 10.1};
	mba::MBA<3> interpX(lo3, hi3, grid3, cooRest, restAnimatedX, 12);
	mba::MBA<3> interpY(lo3, hi3, grid3, cooRest, restAnimatedY, 12);
	mba::MBA<3> interpZ(lo3, hi3, grid3, cooRest, restAnimatedZ, 12);

	// Flag the SOP as being time dependent (i.e. cook on time changes)
	flags().setTimeDep(true);

	fpreal frame = OPgetDirector()->getChannelManager()->getSample(context.getTime());
	frame *= 0.03;

	// NOTE: If you are only interested in the P attribute, use gdp->getP(),
	//       or don't bother making a new GA_RWHandleV3, and just use
	//       gdp->getPos3(ptoff) and gdp->setPos3(ptoff, Pvalue).
	//       This just gives an example supplying an attribute name.
	GA_RWHandleV3 Phandle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
	GA_Offset ptoff;
	GA_FOR_ALL_PTOFF(gdp, ptoff)
	{
		UT_Vector3 Pvalue = Phandle.get(ptoff);
		float x = float( interpX(mba::point<3>{Pvalue.x()/float( 100 ), Pvalue.y() / float( 100 ), Pvalue.z() / float( 100 )}) );
		float y = float( interpY(mba::point<3>{Pvalue.x()/float( 100 ), Pvalue.y() / float( 100 ), Pvalue.z() / float( 100 )}) );
		float z = float( interpZ(mba::point<3>{Pvalue.x()/float( 100 ), Pvalue.y() / float( 100 ), Pvalue.z() / float( 100 )}) );

		//Pvalue.y() = sin(Pvalue.x()*.2 + Pvalue.z()*.3 + frame);
		Pvalue.x() += x * 100;
		Pvalue.y() += y * 100;
		Pvalue.z() += z * 100;
		Phandle.set(ptoff, Pvalue);
	}

	// If we've modified an attribute, and we're managing our own data IDs,
	// we must bump the data ID for that attribute.
	Phandle.bumpDataId();

	return error();
}
