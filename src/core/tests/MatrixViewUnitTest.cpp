/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/MatrixView.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace iDynTree;

void foo(MatrixView<double> m)
{
    std::cerr << "m has " << m.rows() << " rows" << std::endl;
}

int main()
{
    MatrixDynSize m_idyntree(5, 3);
    MatrixDynSize m_idyntree_2(5, 3);
    toEigen(m_idyntree).setRandom();

    Eigen::MatrixXd m_eigen(5, 3);

    MatrixView<double, false> eigenView(m_eigen);

    std::cerr << "******************" << std::endl;
    std::cerr << "Here it is not working because the wrong copy assignment operator has been called" << std::endl;
    eigenView = m_idyntree;
    std::cerr << "Eigen " << std::endl;
    std::cerr << m_eigen << std::endl;

    std::cerr << "iDynTree " << std::endl;
    std::cerr << m_idyntree.toString() << std::endl;
    std::cerr << "******************" << std::endl;


    std::cerr << "******************" << std::endl;
    std::cerr << "Eigen " << std::endl;
    eigenView = MatrixView<double, true>(m_idyntree);
    std::cerr << m_eigen << std::endl;

    std::cerr << "iDynTree " << std::endl;
    std::cerr << m_idyntree.toString() << std::endl;
    std::cerr << "******************" << std::endl;

    foo(m_idyntree);

    // This does not compile
    // foo(m_eigen);

    // This works thanks to the (CTAD)
    // https://en.cppreference.com/w/cpp/language/class_template_argument_deduction
    // https://devblogs.microsoft.com/cppblog/how-to-use-class-template-argument-deduction/
    MatrixView eigenView2(m_eigen);

    // // This is not compiling
    //  error: could not convert ‘iDynTree::MatrixView<double, false>(m_eigen)’ from ‘iDynTree::MatrixView<double, false>’ to ‘iDynTree::MatrixView<double, true>’
    // foo(MatrixView(m_eigen));

    return 0;
}
