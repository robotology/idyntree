/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Istituto Italiano di Tecnologia
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author:  Silvio Traversaro */

#ifndef SYMORO_PAR_MODEL_H
#define SYMORO_PAR_MODEL_H

#include <string>
#include <sstream>
#include <iostream>
#include <vector>

namespace iDynTree
{

/**
 * Class for representing the content of a SyMoRo PAR file (only geometric parameters for now)
 */
class symoro_par_model {


private:
    static std::string vector2string(const std::vector<int> & vec)
    {
        std::stringstream ss;
        for(unsigned int l=0; l < vec.size(); l++ ) { ss << " " << vec[l]; }
        return ss.str();
    }

    static std::string vector2string(const std::vector<double> & vec)
    {
        std::stringstream ss;
        for(size_t l=0; l < vec.size(); l++ ) { ss << " " << vec[l]; }
        return ss.str();
    }

public:
    std::string name;

    size_t NF;
    size_t NL;
    size_t NJ;
    int Type;

    std::vector<int> Ant;
    std::vector<int> Sigma;
    std::vector<double> B;
    std::vector<double> d;
    std::vector<double> R;
    std::vector<double> gamma;
    std::vector<double> Alpha;
    std::vector<int> Mu;
    std::vector<double> Theta;

    std::string toString() const {
        std::stringstream ss;
        ss << "Robot name:\t" << name << std::endl;
        ss << "NF\t" << NF << std::endl;
        ss << "NL\t" << NL << std::endl;
        ss << "NJ\t" << NJ << std::endl;
        ss << "Type\t" << Type << std::endl;
        ss << "Ant\t" << vector2string(Ant) << std::endl;
        ss << "Sigma\t" << vector2string(Sigma) << std::endl;
        ss << "B\t" << vector2string(B) << std::endl;
        ss << "d\t" << vector2string(d) << std::endl;
        ss << "R\t" << vector2string(R) << std::endl;
        ss << "gamma\t" << vector2string(gamma) << std::endl;
        ss << "Alpha\t" << vector2string(Alpha) << std::endl;
        ss << "Mu\t" << vector2string(Mu) << std::endl;
        ss << "Theta\t" << vector2string(Theta) << std::endl;
        return ss.str();
    }

    bool isConsistent() const {
        if( Type < 0 || Type >= 3 ) return false;

        //Type specific checks
        if( Type == 0 ) {
            if( NL != NJ || NL != NF ) return false;
            if( NL != Ant.size() || NL != Sigma.size() || NL != Mu.size() || NL != B.size() ||
                NL != d.size()   || NL != R.size()     || NL != gamma.size() || NL != Alpha.size() || NL != Theta.size() ) return false;
            for(size_t j=0; j < NL; j++ ) { if( B[j] != 0 || gamma[j] != 0 ) { return false; } }
        } else if ( Type == 1 ) {
            if( NL != NJ || NL != NF ) return false;
            if( NL != Ant.size() || NL != Sigma.size() || NL != Mu.size() || NL != B.size() ||
                NL != d.size()   || NL != R.size()     || NL != gamma.size() || NL != Alpha.size() || NL != Theta.size() ) return false;
        } else if ( Type == 2 ) {
        }

        return true;
    }
};
}

#endif
