/***************************************************************************
 *   Copyright (C) 2006-2008 by Neuronics AG                               *
 *   support@neuronics.ch                                                  *
 ***************************************************************************/

#include "KatanaKinematicsDecisionAlgorithms.h"
#include "MathHelperFunctions.h"

namespace AnaGuess {


KinematicsDefaultEncMinAlgorithm::t_iter
KinematicsDefaultEncMinAlgorithm::operator() (t_iter targetEnc_begin, t_iter targetEnc_end, c_iter currentEnc_begin, c_iter currentEnc_end ) {
	double dist(0), sum(0), mindist=1000000;
	t_iter index = targetEnc_end;

	for(t_iter target = targetEnc_begin; target != targetEnc_end; ++target) {
		sum = 0;

		c_iter t = (*target).begin();
		c_iter c = currentEnc_begin;
		while( t != (*target).end() && c != currentEnc_end) {
			sum += MHF::pow2<double>( *t - *c);
			++t;
			++c;
		}
		dist = sqrt(sum);
		if(dist < mindist) {
			index = target;
			mindist = dist;
		}
	}

	return index;
}
////////////////////////////////////////////////////////////////////////////

KinematicsDefaultRadMinAlgorithm::t_iter
KinematicsDefaultRadMinAlgorithm::operator() (t_iter targetRad_begin, t_iter targetRad_end, c_iter currentRad_begin, c_iter currentRad_end ) {
	double dist(0), sum(0), mindist=1000000;
	t_iter index = targetRad_end;

	for(t_iter target = targetRad_begin; target != targetRad_end; ++target) {
		sum = 0;

		c_iter t = (*target).begin();
		c_iter c = currentRad_begin;
		while( t != (*target).end() && c != currentRad_end) {
			sum += MHF::pow2<double>( *t - *c);
			++t;
			++c;
		}
		dist = sqrt(sum);
		if(dist < mindist) {
			index = target;
			mindist = dist;
		}
	}

	return index;
}
////////////////////////////////////////////////////////////////////////////

} // namespace
