/***************************************************************************
 *   Copyright (C) 2008 by Neuronics AG                               *
 *   support@neuronics.ch                                                  *
 ***************************************************************************/

#ifndef _KATANAKINEMATICSDECISIONALGORITHMS_H
#define _KATANAKINEMATICSDECISIONALGORITHMS_H

#include <vector>

namespace AnaGuess {


struct KinematicsDefaultEncMinAlgorithm {
    typedef std::vector<int>    encoders;
    typedef encoders::const_iterator c_iter;
    typedef std::vector< encoders >::const_iterator t_iter;

    t_iter operator() ( t_iter targetEnc_begin, t_iter targetEnc_end, c_iter currentEnc_begin, c_iter currentEnc_end );
};


struct KinematicsDefaultRadMinAlgorithm {
    typedef std::vector<double>    radians;
    typedef radians::const_iterator c_iter;
    typedef std::vector< radians >::const_iterator t_iter;

    t_iter operator() ( t_iter targetRad_begin, t_iter targetRad_end, c_iter currentRad_begin, c_iter currentRad_end );
};


} // namespace

#endif
