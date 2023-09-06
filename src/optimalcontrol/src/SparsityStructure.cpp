// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/SparsityStructure.h>
#include <iDynTree/Utils.h>
#include <cassert>

void iDynTree::optimalcontrol::SparsityStructure::addNonZero(size_t row, size_t col)
{
    m_nonZeroElementRows.push_back(row);
    m_nonZeroElementColumns.push_back(col);
    m_register.insert(std::to_string(row) + "_" + std::to_string(col));
}

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

    for (size_t i = 0; i < other.size(); ++i) {
        add(other.nonZeroElementRows()[i], other.nonZeroElementColumns()[i]);
    }

    return true;
}

void iDynTree::optimalcontrol::SparsityStructure::addDenseBlock(size_t startRow, size_t startColumn, size_t numberOfRows, size_t numberOfColumns)
{
    for (size_t i = 0; i < numberOfRows; ++i) {
        for (size_t j = 0; j < numberOfColumns; ++j) {
            add(startRow + i, startColumn + j);
        }
    }
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

    addDenseBlock(static_cast<size_t>(rowsRange.offset),
                  static_cast<size_t>(columnsRange.offset),
                  static_cast<size_t>(rowsRange.size),
                  static_cast<size_t>(columnsRange.size));

    return true;
}

void iDynTree::optimalcontrol::SparsityStructure::addIdentityBlock(size_t startRow, size_t startColumn, size_t dimension)
{
    for (size_t i = 0; i < dimension; ++i) {
        add(startRow + i, startColumn + i);
    }
}

bool iDynTree::optimalcontrol::SparsityStructure::addBlock(size_t startRow, size_t startColumn, const iDynTree::optimalcontrol::SparsityStructure &other)
{
    if (!other.isValid()) {
        reportError("SparsityStructure", "addBlock", "The other SparsityStructure vectors have different size.");
        return false;
    }

    for (size_t i = 0; i < other.size(); ++i) {
        add(startRow + other.nonZeroElementRows()[i], startColumn + other.nonZeroElementColumns()[i]);
    }

    return true;
}

void iDynTree::optimalcontrol::SparsityStructure::add(size_t newRow, size_t newCol)
{
    if (!isValuePresent(newRow, newCol)) {
        addNonZero(newRow, newCol);
    }
}

void iDynTree::optimalcontrol::SparsityStructure::add(NonZero newElement)
{
    if (!isValuePresent(newElement.row, newElement.col)) {
        addNonZero(newElement.row, newElement.col);
    }
}

bool iDynTree::optimalcontrol::SparsityStructure::isValuePresent(size_t row, size_t col) const
{
    return (m_register.find(std::to_string(row) + "_" + std::to_string(col)) != m_register.end());
}

void iDynTree::optimalcontrol::SparsityStructure::reserve(size_t newSize)
{
    m_nonZeroElementRows.reserve(newSize);
    m_nonZeroElementColumns.reserve(newSize);
    m_register.reserve(newSize);
}

void iDynTree::optimalcontrol::SparsityStructure::clear()
{
    m_register.clear();
    m_nonZeroElementRows.clear();
    m_nonZeroElementColumns.clear();
}

size_t iDynTree::optimalcontrol::SparsityStructure::size() const
{
    assert(isValid());
    return m_nonZeroElementRows.size();
}

bool iDynTree::optimalcontrol::SparsityStructure::isValid() const
{
    return (m_nonZeroElementColumns.size() == m_nonZeroElementRows.size());
}

iDynTree::optimalcontrol::NonZero iDynTree::optimalcontrol::SparsityStructure::operator[](size_t index) const
{
    NonZero element;
    element.row = m_nonZeroElementRows[index];
    element.col = m_nonZeroElementColumns[index];
    return element;
}

const std::vector<size_t> &iDynTree::optimalcontrol::SparsityStructure::nonZeroElementRows() const
{
    return m_nonZeroElementRows;
}

const std::vector<size_t> &iDynTree::optimalcontrol::SparsityStructure::nonZeroElementColumns() const
{
    return m_nonZeroElementColumns;
}
