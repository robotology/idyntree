/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Direction.h>

#include <Eigen/Dense>

#include <cstdio>
#include <sstream>

namespace iDynTree
{

    Direction::Direction()
    {
        this->setToDefault();
    }

    Direction::Direction(double x, double y, double z)
    {
        this->m_data[0] = x;
        this->m_data[1] = y;
        this->m_data[2] = z;

        this->Normalize();
    }

    Direction::Direction(const double* in_data, const unsigned int in_size):
                 VectorFixSize< 3 >(in_data,in_size)
    {
        this->Normalize();
    }

    Direction::Direction(const Direction& other)
    {
        this->m_data[0] = other.m_data[0];
        this->m_data[1] = other.m_data[1];
        this->m_data[2] = other.m_data[2];

        this->Normalize();
    }

    Direction::~Direction()
    {

    }

    void Direction::setToDefault()
    {
        this->m_data[0] = 1.0;
        this->m_data[1] = 0.0;
        this->m_data[2] = 0.0;
    }

    void Direction::Normalize(double tol)
    {
        Eigen::Map<Eigen::Vector3d> vec(this->m_data);
        double nrm = vec.norm();
        if( nrm < tol )
        {
            this->setToDefault();
        }
        else
        {
            vec.normalize();
        }

        return;
    }

    std::string Direction::toString() const
    {
        std::stringstream ss;

        ss << " x " << this->m_data[0]
        << " y " << this->m_data[1]
        << " z " << this->m_data[2];

        return ss.str();
    }

    std::string Direction::reservedToString() const
    {
        return this->toString();
    }

}