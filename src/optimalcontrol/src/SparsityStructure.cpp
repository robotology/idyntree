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

void iDynTree::optimalcontrol::SparsityStructure::addDenseBlock(size_t startRow, size_t startColumn, size_t numberOfRows, size_t numberOfColumns)
{
    for (size_t i = 0; i < numberOfRows; ++i) {
        for (size_t j = 0; j < numberOfColumns; ++j) {
            addNonZeroIfNotPresent(startRow + i, startColumn + j);
        }
    }
}

bool iDynTree::optimalcontrol::SparsityStructure::addDenseBlock(long startRow, long startColumn, long numberOfRows, long numberOfColumns)
{
    if (startRow < 0) {
        reportError("SparsityStructure", "addDenseBlock", "The startRow is negative.");
        return false;
    }

    if (startColumn < 0) {
        reportError("SparsityStructure", "addDenseBlock", "The startColumn is negative.");
        return false;
    }

    if (numberOfRows < 0) {
        reportError("SparsityStructure", "addDenseBlock", "The numberOfRows is negative.");
        return false;
    }

    if (numberOfColumns < 0) {
        reportError("SparsityStructure", "addDenseBlock", "The numberOfColumns is negative.");
        return false;
    }

    addDenseBlock(static_cast<size_t>(startRow),
                  static_cast<size_t>(startColumn),
                  static_cast<size_t>(numberOfRows),
                  static_cast<size_t>(numberOfColumns));
    return true;
}

bool iDynTree::optimalcontrol::SparsityStructure::addDenseBlock(const iDynTree::IndexRange &rowsRange, const iDynTree::IndexRange &columnsRange)
{
    if (!rowsRange.isValid()) {
        reportError("SparsityStructure", "addDenseBlock", "The rowsRange is not valid.");
        return false;
    }

    if (!columnsRange.isValid()) {
        reportError("SparsityStructure", "addDenseBlock", "The columnsRange is not valid.");
        return false;
    }

    addDenseBlock(rowsRange.offset, columnsRange.offset, rowsRange.size, columnsRange.size);

    return true;
}

void iDynTree::optimalcontrol::SparsityStructure::addIdentityBlock(size_t startRow, size_t startColumn, size_t dimension)
{
    for (size_t i = 0; i < dimension; ++i) {
        addNonZeroIfNotPresent(startRow + i, startColumn + i);
    }
}

bool iDynTree::optimalcontrol::SparsityStructure::addIdentityBlock(long startRow, long startColumn, long dimension)
{
    if (startRow < 0) {
        reportError("SparsityStructure", "addIdentityBlock", "The startRow is negative.");
        return false;
    }

    if (startColumn < 0) {
        reportError("SparsityStructure", "addIdentityBlock", "The startColumn is negative.");
        return false;
    }

    if (dimension < 0) {
        reportError("SparsityStructure", "addIdentityBlock", "The dimension is negative.");
        return false;
    }

    addIdentityBlock(static_cast<size_t>(startRow),
                     static_cast<size_t>(startColumn),
                     static_cast<size_t>(dimension)
                     );
    return true;
}

void iDynTree::optimalcontrol::SparsityStructure::addNonZeroIfNotPresent(size_t newRow, size_t newCol)
{
    if (!isValuePresent(newRow, newCol)) {
        nonZeroElementRows.push_back(newRow);
        nonZeroElementColumns.push_back(newCol);
    }
}

bool iDynTree::optimalcontrol::SparsityStructure::isValuePresent(size_t row, size_t col) const
{
    for (size_t i = 0; i < nonZeroElementRows.size(); ++i) {
        if ((row == nonZeroElementRows[i]) && (col == nonZeroElementColumns[i])) {
            return true;
        }
    }
    return false;
}

void iDynTree::optimalcontrol::SparsityStructure::resize(size_t newSize)
{
    nonZeroElementRows.resize(newSize);
    nonZeroElementColumns.resize(newSize);
}

void iDynTree::optimalcontrol::SparsityStructure::clear()
{
    nonZeroElementRows.clear();
    nonZeroElementColumns.clear();
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
