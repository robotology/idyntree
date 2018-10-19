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

#include <iDynTree/Core/Utils.h>
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

    void addDenseBlock(size_t startRow, size_t startColumn, size_t numberOfRows, size_t numberOfColumns);

    bool addDenseBlock(long startRow, long startColumn, long numberOfRows, long numberOfColumns);

    bool addDenseBlock(const iDynTree::IndexRange& rowsRange, const iDynTree::IndexRange& columnsRange);

    void addIdentityBlock(size_t startRow, size_t startColumn, size_t dimension);

    bool addIdentityBlock(long startRow, long startColumn, long dimension);

    void addNonZeroIfNotPresent(size_t newRow, size_t newCol);

    bool isValuePresent(size_t row, size_t col) const;

    void resize(size_t newSize);

    void clear();

    size_t size() const;

    bool isValid() const;
};

#endif // IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H
