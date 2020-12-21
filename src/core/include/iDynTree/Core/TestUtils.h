/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_TEST_UTILS_H
#define IDYNTREE_TEST_UTILS_H

#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Core/Utils.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <cstdlib>
#include <cmath>


namespace iDynTree
{
    class Transform;
    class SpatialMotionVector;
    class SpatialForceVector;
    class Axis;
    class SpatialForceVector;
    class SpatialMotionVector;
    class SpatialInertia;
    class Position;
    class Rotation;

#define ASSERT_IS_TRUE(prop) iDynTree::assertTrue(prop,__FILE__,__LINE__)
#define ASSERT_IS_FALSE(prop) iDynTree::assertTrue(!(prop),__FILE__,__LINE__)
#define ASSERT_EQUAL_STRING(val1,val2) iDynTree::assertStringAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_DOUBLE(val1,val2) iDynTree::assertDoubleAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_DOUBLE_TOL(val1,val2,tol) iDynTree::assertDoubleAreEqual(val1,val2,tol,__FILE__,__LINE__)
#define ASSERT_EQUAL_VECTOR(val1,val2) assertVectorAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_VECTOR_TOL(val1,val2,tol) assertVectorAreEqual(val1,val2,tol,__FILE__,__LINE__)
#define ASSERT_EQUAL_VECTOR_REL_TOL(val1,val2,relTol,minAbsTol) assertVectorAreEqualWithRelativeTol(val1,val2,relTol,minAbsTol,__FILE__,__LINE__)
#define ASSERT_EQUAL_SPATIAL_MOTION(val1,val2) assertSpatialMotionAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_SPATIAL_FORCE(val1,val2) assertSpatialForceAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_SPATIAL_FORCE_TOL(val1,val2,tol) assertSpatialForceAreEqual(val1,val2,tol,__FILE__,__LINE__)
#define ASSERT_EQUAL_MATRIX(val1,val2) assertMatrixAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_MATRIX_TOL(val1,val2,tol) assertMatrixAreEqual(val1,val2,tol,__FILE__,__LINE__)
#define ASSERT_EQUAL_TRANSFORM(val1,val2) assertTransformsAreEqual(val1,val2,iDynTree::DEFAULT_TOL,__FILE__,__LINE__)
#define ASSERT_EQUAL_TRANSFORM_TOL(val1,val2,tol) assertTransformsAreEqual(val1,val2,tol,__FILE__,__LINE__)


    struct TestMatrixMismatch {
        bool match;
        double expected;
        double realElement;

        TestMatrixMismatch(bool _match, double _expected, double _realElement)
        : match(_match), expected(_expected), realElement(_realElement) {}
    };

    void assertStringAreEqual(const std::string & val1, const std::string & val2, double tol = DEFAULT_TOL, std::string file="", int line=-1);


    void assertDoubleAreEqual(const double & val1, const double & val2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

     /**
     * Assert that two transforms are equal, and
     * exit with EXIT_FAILURE if they are not.
     *
     */
    void assertTransformsAreEqual(const Transform & trans1, const Transform & trans2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

    /**
     * Assert that two spatial motion vectors are equal,
     * and exit with EXIT_FAILURE if they are not.
     *
     */
    void assertSpatialMotionAreEqual(const SpatialMotionVector & t1, const SpatialMotionVector & t2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

    /**
     * Assert that two spatial force vectors are equal,
     * and exit with EXIT_FAILURE if they are not.
     *
     */
    void assertSpatialForceAreEqual(const SpatialForceVector & f1, const SpatialForceVector & f2, double tol = DEFAULT_TOL, std::string file="", int line=-1);

    void assertTrue(bool prop,std::string file="", int line=-1);

    /**
     * Get random bool
     */
    bool getRandomBool();

    /**
     * Get a random double between min and max .
     */
    double getRandomDouble(double min=0.0, double max=1.0);

    /**
     * Get a random integer between min and max (included).
     * For example a dice could be simulated with getRandomInteger(1,6);
     */
    int getRandomInteger(int min, int max);

    /**
     * Fill a vector with random double.
     */
    template<typename VectorType>
    void getRandomVector(VectorType & vec, double min=0.0, double max=1.0)
    {
        for(unsigned int i=0; i<vec.size(); i++)
        {
            vec(i) = getRandomDouble(min,max);
        }
    }

    /**
     * Fill a matrix of random doubles.
     */
    template<typename MatrixType>
    void getRandomMatrix(MatrixType & mat)
    {
        for(unsigned int i=0; i<mat.rows(); i++)
        {
            for(unsigned int j=0; j<mat.cols(); j++)
            {
                mat(i,j) = getRandomDouble();
            }
        }
    }

    /**
     * Get a random position.
     */
    Position getRandomPosition();

    /**
     * Get a random rotation.
     */
    Rotation getRandomRotation();

    /**
     * Get a random transform.
     */
    Transform getRandomTransform();

    /**
     * Get a random axis.
     */
    Axis getRandomAxis();

    /**
     * Get a random (but physically consistent) inertia.
     */
    SpatialInertia getRandomInertia();

    /**
     * Get a random twist-like 6D vector.
     */
    SpatialMotionVector getRandomTwist();

    /**
     * Get a random wrench-like 6D object.
     */
    SpatialForceVector getRandomWrench();

     /**
     * Helper for printing vectors
     */
    template<typename VectorType>
    void printVector(std::string /*name*/, const VectorType& vec)
    {
        for(unsigned int i=0; i < vec.size(); i++ )
        {
            std::cerr << vec(i) << "\n";
        }
        std::cerr << "\n";
    }

    /**
     * Helper for printing difference of two vectors
     */
    template<typename VectorType1, typename VectorType2>
    void printVectorDifference(std::string name, const VectorType1& vec1, const VectorType2& vec2)
    {
        std::cerr << name << " : \n";
        size_t minSize = vec1.size();

        if( vec2.size() < minSize )
        {
            minSize = vec2.size();
        }

        for(unsigned int i=0; i < minSize; i++ )
        {
            std::cerr << vec1(i) - vec2(i) << " ( " << (vec1(i) - vec2(i))/std::max(vec1(i),vec2(i)) << " ) " << "\n";
        }
    }

    /**
     * Helper for printing the patter of wrong elements
     * in between two vectors
     */
    inline void printVectorWrongElements(std::string name, std::vector<bool> & correctElems)
    {
        std::cerr << name << " ( . match, X mismatch): \n";

        for(unsigned int i=0; i < correctElems.size(); i++ )
        {
            if( correctElems[i] )
            {
                std::cerr << "." << "\n";
            }
            else
            {
                std::cerr << "X" << "\n";
            }
        }
    }

    /**
     * Helper for printing the patter of wrong elements
     * in between two matrix
     */
    inline void printMatrixWrongElements(std::string name, std::vector< std::vector<TestMatrixMismatch> > & correctElems)
    {
#ifndef _WIN32
#define IDYNTREE_MATCH_CHARACTER "\u2714"
#else
#define IDYNTREE_MATCH_CHARACTER "o"
#endif
        std::cerr << name << "( " IDYNTREE_MATCH_CHARACTER " match, (expected,got:error) mismatch): \n";

        size_t rows = correctElems.size();
        size_t cols = correctElems[0].size();
        for(unsigned int row=0; row < rows; row++ )
        {
            for( unsigned int col = 0; col < cols; col++ )
            {
                if( correctElems[row][col].match )
                {
                    std::cerr << IDYNTREE_MATCH_CHARACTER;
                }
                else
                {
                    std::cerr << "(" <<  correctElems[row][col].expected << "," << correctElems[row][col].realElement
                    << ":" << correctElems[row][col].realElement - correctElems[row][col].expected << ")";
                }

                std::cerr << " ";
            }

            std::cerr << "\n";
        }
    }

    /**
     * Helper for printing the patter of wrong elements
     * in between two matrix
     */
    template<typename MatrixType1, typename MatrixType2>
    void printMatrixPercentageError(const MatrixType1& mat1, const MatrixType2& mat2)
    {
        size_t rows = mat1.rows();
        size_t cols = mat2.cols();
        for(unsigned int row=0; row < rows; row++ )
        {
            for( unsigned int col = 0; col < cols; col++ )
            {
                double mat1el = mat1(row,col);
                double mat2el = mat2(row,col);
                double percentageError = std::abs(mat1el-mat2el)/std::max(mat1el,mat2el);
                std::cerr << std::fixed << std::setprecision(3) << percentageError << " ";
            }

            std::cerr << "\n";
        }
    }


    /**
     * Assert that two vectors are equal, and
     * exit with EXIT_FAILURE if they are not.
     */
    template<typename VectorType1, typename VectorType2>
    void assertVectorAreEqual(const VectorType1& vec1, const VectorType2& vec2, double tol, std::string file, int line)
    {
        if( vec1.size() != vec2.size() )
        {
            std::cerr << file << ":" << line << " : assertVectorAreEqual failure: vec1 has size " << vec1.size()
                    << " while vec2 has size " << vec2.size() << std::endl;
            exit(EXIT_FAILURE);
        }

        std::vector<bool> correctElements(vec1.size(),true);
        bool checkCorrect = true;

        for( unsigned int i = 0; i < vec1.size(); i++ )
        {
            if( !(fabs(vec1(i)-vec2(i)) < tol) )
            {
                checkCorrect = false;
                correctElements[i] = false;
            }
        }

        if( !checkCorrect )
        {
            std::cerr << file << ":" << line << " : assertVectorAreEqual failure: " << std::endl;
            printVector("vec1",vec1);
            printVector("vec2",vec2);
            printVectorDifference("vec1-vec2",vec1,vec2);
            printVectorWrongElements("wrong el:",correctElements);
            exit(EXIT_FAILURE);
        }
    }

    /**
     * Assert that two vectors are equal, and exit with EXIT_FAILURE if they are not.
     *
     * The tolerance passed in this function is a relative tolerance on the max element of the comparison, i.e.
     * absoluteTol = max(relativeTol*max(val1,val2), minAbsoluteTol)
     */
    template<typename VectorType1, typename VectorType2>
    void assertVectorAreEqualWithRelativeTol(const VectorType1& vec1, const VectorType2& vec2, double relativeTol, double minAbsoluteTol, std::string file, int line)
    {
        if( vec1.size() != vec2.size() )
        {
            std::cerr << file << ":" << line << " : assertVectorAreEqualWithRelativeTol failure: vec1 has size " << vec1.size()
                      << " while vec2 has size " << vec2.size() << std::endl;
            exit(EXIT_FAILURE);
        }

        std::vector<bool> correctElements(vec1.size(),true);
        bool checkCorrect = true;

        for( unsigned int i = 0; i < vec1.size(); i++ )
        {
            double absoluteTol = std::max(relativeTol*std::max(std::abs(vec1(i)), std::abs(vec2(i))), minAbsoluteTol);
            if( fabs(vec1(i)-vec2(i)) >= absoluteTol )
            {
                checkCorrect = false;
                correctElements[i] = false;
            }
        }

        if( !checkCorrect )
        {
            std::cerr << file << ":" << line << " : assertVectorAreEqualWithRelativeTol failure: " << std::endl;
            printVector("vec1",vec1);
            printVector("vec2",vec2);
            printVectorDifference("vec1-vec2",vec1,vec2);
            printVectorWrongElements("wrong el:",correctElements);
            exit(EXIT_FAILURE);
        }
    }

   /**
    * Assert that two matrices are equal, and
    * exit with EXIT_FAILURE if they are not.
    *
    */
    template<typename MatrixType1, typename MatrixType2>
    void assertMatrixAreEqual(const MatrixType1& mat1, const MatrixType2& mat2, double tol, std::string file, int line)
    {
        if( mat1.rows() != mat2.rows() ||
            mat2.cols() != mat1.cols() )
        {
            std::cerr << file << ":" << line << " : assertMatrixAreEqual failure: mat1 has size " << mat1.rows() << " " << mat1.cols()
                    << " while mat2 has size " << mat2.rows() << " " << mat2.cols() << std::endl;
            exit(EXIT_FAILURE);
        }

        std::vector< std::vector<TestMatrixMismatch> > correctElements(mat2.rows(), std::vector<TestMatrixMismatch>(mat1.cols(), TestMatrixMismatch(true, 0, 0)) );
        bool checkCorrect = true;

        for( unsigned int row = 0; row < mat2.rows(); row++ )
        {
            for( unsigned int col = 0; col < mat2.cols(); col++ )
            {
                if( fabs(mat1(row,col)-mat2(row,col)) >= tol )
                {
                    checkCorrect = false;
                    correctElements[row][col].match = false;
                    correctElements[row][col].expected = mat1(row,col);
                    correctElements[row][col].realElement = mat2(row,col);
                }
            }
        }

        if( !checkCorrect )
        {
            std::cerr << file << ":" << line << " : assertMatrixAreEqual failure with tol " << tol << " : " << std::endl;
            printMatrixWrongElements("wrong el:",correctElements);
            //std::cerr << "percentage error : " << std::endl;
            //printMatrixPercentageError(mat1,mat2);
            exit(EXIT_FAILURE);
        }

    }


}

#endif
