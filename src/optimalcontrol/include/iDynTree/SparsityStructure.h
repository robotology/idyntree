/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H
#define IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H

#include <vector>
#include <cstddef>

namespace iDynTree {
    namespace optimalcontrol {
        class SparsityStructure;
    }
}

class iDynTree::optimalcontrol::SparsityStructure {
public:
    std::vector<size_t> nonZeroElementRows;
    std::vector<size_t> nonZeroElementColumns;

    SparsityStructure();

    ~SparsityStructure();

    bool merge(const SparsityStructure& other);

    void addNonZeroIfNotPresent(size_t newRow, size_t newCol);

    void resize(size_t newSize);

    size_t size() const;

    bool isValid() const;
};

#endif // IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H