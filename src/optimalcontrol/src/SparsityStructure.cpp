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

#include <iDynTree/SparsityStructure.h>
#include <iDynTree/Core/Utils.h>
#include <cassert>

iDynTree::optimalcontrol::SparsityStructure::SparsityStructure()
{ }

iDynTree::optimalcontrol::SparsityStructure::~SparsityStructure()
{ }

bool iDynTree::optimalcontrol::SparsityStructure::merge(const iDynTree::optimalcontrol::SparsityStructure &other)
{
    if (!other.isValid()) {
        reportError("SparsityStructure", "merge", "The other SparsityStructure vectors have different size.");
        return false;
    }

    for (size_t i = 0; i < other.nonZeroElementRows.size(); ++i) {
        addNonZeroIfNotPresent(other.nonZeroElementRows[i], other.nonZeroElementColumns[i]);
    }

    return true;
}

void iDynTree::optimalcontrol::SparsityStructure::addNonZeroIfNotPresent(size_t newRow, size_t newCol)
{
    for (size_t i = 0; i < nonZeroElementRows.size(); ++i) {
        if ((newRow == nonZeroElementRows[i]) && (newCol == nonZeroElementColumns[i])) {
            return;
        }
    }
    nonZeroElementRows.push_back(newRow);
    nonZeroElementColumns.push_back(newCol);
}

void iDynTree::optimalcontrol::SparsityStructure::resize(size_t newSize)
{
    nonZeroElementRows.resize(newSize);
    nonZeroElementColumns.resize(newSize);
}

size_t iDynTree::optimalcontrol::SparsityStructure::size() const
{
    assert(isValid());
    return nonZeroElementRows.size();
}

bool iDynTree::optimalcontrol::SparsityStructure::isValid() const
{
    return (nonZeroElementColumns.size() == nonZeroElementRows.size());
}
