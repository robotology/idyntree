// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#ifndef IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H
#define IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H

#include <iDynTree/Utils.h>
#include <vector>
#include <cstddef>
#include <unordered_set>
#include <string>

namespace iDynTree {
    namespace optimalcontrol {
        class SparsityStructure;

        struct NonZero {
            size_t row;
            size_t col;
        };
    }
}

class iDynTree::optimalcontrol::SparsityStructure {
    std::unordered_set<std::string> m_register;

    void addNonZero(size_t row, size_t col);

    std::vector<size_t> m_nonZeroElementRows;
    std::vector<size_t> m_nonZeroElementColumns;

public:

    SparsityStructure();

    ~SparsityStructure();

    bool merge(const SparsityStructure& other);

    void addDenseBlock(size_t startRow, size_t startColumn, size_t numberOfRows, size_t numberOfColumns);

    bool addDenseBlock(const iDynTree::IndexRange& rowsRange, const iDynTree::IndexRange& columnsRange);

    void addIdentityBlock(size_t startRow, size_t startColumn, size_t dimension);

    bool addBlock(size_t startRow, size_t startColumn, const SparsityStructure& other);

    void add(size_t newRow, size_t newCol);

    void add(NonZero newElement);

    bool isValuePresent(size_t row, size_t col) const;

    void reserve(size_t newSize);

    void clear();

    size_t size() const;

    bool isValid() const;

    NonZero operator[](size_t index) const;

    const std::vector<size_t>& nonZeroElementRows() const;

    const std::vector<size_t>& nonZeroElementColumns() const;
};

#endif // IDYNTREE_OPTIMALCONTROL_SPARSITYSTRUCTURE_H
