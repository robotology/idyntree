/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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
