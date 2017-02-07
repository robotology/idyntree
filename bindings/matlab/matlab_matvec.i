// Matlab specific swig code (to be executed after header parsing)

namespace iDynTree
{

%define VECTORSPARSECOPY(sparsevector, dynVector, cols)
    //getting pointer to sparse structure
    mwIndex* ir = mxGetIr(sparsevector);
    mwIndex* jc = mxGetJc(sparsevector);
    double *data = mxGetPr(sparsevector);

    //If cols == 1 assume it is a column vector (or a 1 element vector)
    bool isColumnVector = (cols == 1);

    //zero the matrix (iDynTree vector is dense)
    dynVector->zero();
    for (mwIndex col = 0; col < cols; col++)
    {
        /*
            jc[col] contains information about the nonzero values.
            jc[col + 1] - jc[col] = number of nonzero elements in column col
            These nonzero elements can be access with a for loop starting at
            jc[col] and ending at jc[col + 1] - 1.

        */
        mwIndex startingRowIndex = jc[col];
        mwIndex endRowIndex = jc[col + 1];
        if (startingRowIndex == endRowIndex)
        {
            //no elements in this column
            continue;
        }
        for (mwIndex currentIndex = startingRowIndex; currentIndex < endRowIndex; currentIndex++)
        {
            //access the element
            mwIndex row = ir[currentIndex];
            //iDynTree has only one size.
            dynVector->operator()(isColumnVector ? row : col) = data[currentIndex];
        }
    }
%enddef

%define MATRIXSPARSECOPY(sparsematrix, dynMatrix, cols)
    //getting pointer to sparse structure
    mwIndex* ir = mxGetIr(sparsematrix);
    mwIndex* jc = mxGetJc(sparsematrix);
    double *data = mxGetPr(sparsematrix);

    //zero the matrix (iDynTree matrix is dense)
    dynMatrix->zero();
    for (mwIndex col = 0; col < cols; col++)
    {
        /*
            jc[col] contains information about the nonzero values.
            jc[col + 1] - jc[col] = number of nonzero elements in column col
            These nonzero elements can be access with a for loop starting at
            jc[col] and ending at jc[col + 1] - 1.

        */
        mwIndex startingRowIndex = jc[col];
        mwIndex endRowIndex = jc[col + 1];
        if (startingRowIndex == endRowIndex)
        {
            //no elements in this column
            continue;
        }
        for (mwIndex currentIndex = startingRowIndex; currentIndex < endRowIndex; currentIndex++)
        {
            //access the element
            mwIndex row = ir[currentIndex];
            dynMatrix->operator()(row, col) = data[currentIndex];
        }
    }
%enddef

%extend VectorFixSize
{
    // Convert to a dense matrix
    mxArray * toMatlab() const
    {
        mxArray *p  = mxCreateDoubleMatrix($self->size(), 1, mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        $self->fillBuffer(d); // Column-major
        return p;
    }

    // Convert from a dense matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const size_t * dims = mxGetDimensions(in);
        size_t fixValSize = $self->size();
        size_t nonSingletonDimension = (dims[0] == 1 ? dims[1] : dims[0]);

        if (nonSingletonDimension == fixValSize)
        {
            if (mxIsSparse(in))
            {
                VECTORSPARSECOPY(in, $self, dims[1])
            } else {
                double* d = static_cast<double*>(mxGetData(in));
                double* selfData = $self->data();
                for(size_t i=0; i < fixValSize; i++ )
                {
                    selfData[i] = d[i];
                }
            }
            return;
        } else {
            mexErrMsgIdAndTxt("iDynTree:Core:wrongDimension",
              "Wrong vector size. Matlab size: %d. iDynTree size: %d", nonSingletonDimension, fixValSize);
        }
    }
}

%extend MatrixFixSize
{
    // Convert to a dense matrix
    mxArray * toMatlab() const
    {
        mxArray *p  = mxCreateDoubleMatrix($self->rows(), $self->cols(), mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        $self->fillColMajorBuffer(d); // Column-major
        return p;
    }

    // Convert from a dense or sparse matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const size_t * dims = mxGetDimensions(in);
        size_t fixValRows = $self->rows();
        size_t fixValCols = $self->cols();
        if (dims[0] == fixValRows && dims[1] == fixValCols)
        {
            if (mxIsSparse(in))
            {
                MATRIXSPARSECOPY(in, $self, fixValCols)
            } else {
                double* d = static_cast<double*>(mxGetData(in));
                for (size_t row = 0; row < fixValRows; row++)
                {
                    for (size_t col = 0; col < fixValCols; col++)
                    {
                        $self->operator()(row,col) = d[col*fixValRows + row];
                    }
                }
                return;
            }
         } else {
            mexErrMsgIdAndTxt("iDynTree:Core:wrongDimension",
              "Wrong matrix size. Matlab size: (%d,%d). iDynTree size: (%d,%d)", dims[0], dims[1], fixValRows, fixValCols);
        }
    }
}


%extend VectorDynSize
{
    // Convert to a dense matrix
    mxArray * toMatlab() const
    {
        mxArray *p  = mxCreateDoubleMatrix($self->size(), 1, mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        $self->fillBuffer(d); // Column-major
        return p;
    }

    // Convert from a dense matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const size_t * dims = mxGetDimensions(in);
        $self->size();
        if (( dims[0] == 1 || dims[1] == 1))
        {
            // Get the size of the input vector
            size_t inSize;
            if( dims[0] == 1 )
            {
                inSize = dims[1];
            }
            else
            {
                inSize = dims[0];
            }

            // If the input vector has a size different
            // from the one of the iDynTree::VectorDynSize,
            // we resize iDynTree::VectorDynSize
            if( $self->size() != inSize )
            {
                $self->resize(inSize);
                mexWarnMsgIdAndTxt("iDynTree:Core:perfomance", "Resizing iDynTree vector to %d", inSize);
            }

            if (mxIsSparse(in))
            {
                VECTORSPARSECOPY(in, $self, dims[1])
            } else {
                double* d = static_cast<double*>(mxGetData(in));
                double* selfData = $self->data();
                for(size_t i=0; i < inSize; i++ )
                {
                    selfData[i] = d[i];
                }
            }
            return;
        }
    }
}

%extend MatrixDynSize
{
    // Convert to a dense matrix
    mxArray * toMatlab() const
    {
        mxArray *p  = mxCreateDoubleMatrix($self->rows(), $self->cols(), mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        $self->fillColMajorBuffer(d); // Column-major
        return p;
    }

    // Convert from a dense or sparse matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const size_t * dims = mxGetDimensions(in);
        size_t rows = $self->rows();
        size_t cols = $self->cols();
        if (dims[0] != rows || dims[1] == cols)
        {
            $self->resize(dims[0], dims[1]);
            mexWarnMsgIdAndTxt("iDynTree:Core:perfomance", "Resizing iDynTree vector to (%d,%d)", rows, cols);
        }

        if (mxIsSparse(in))
        {
            MATRIXSPARSECOPY(in, $self, cols)
        } else {
            double* d = static_cast<double*>(mxGetData(in));
            for (size_t row = 0; row < rows; row++)
            {
                for (size_t col = 0; col < cols; col++)
                {
                    $self->operator()(row,col) = d[col*rows + row];
                }
            }
            return;
        }
    }
}

%extend SparseMatrix
{
    // Convert to a sparse matrix
    mxArray * toMatlab() const
    {
        mxArray *p = mxCreateSparse($self->rows(), $self->columns(),
                                    $self->numberOfNonZeros(), mxREAL);
        if (!p) return 0;

        mwIndex* ir = mxGetIr(p);
        mwIndex* jc = mxGetJc(p);
        double *data = mxGetPr(p);

        //for compatibility with the interface I need to create a wrapper of int*
        //not of mwIndex*
        int *tempIr = new int[self->numberOfNonZeros()];
        if (!tempIr) return p;

        int *tempJc = new int[self->columns() + 1];
        if (!tempJc) {
            delete [] tempJc;
            return p;
        }


        self->convertToColumnMajor(data, tempIr, tempJc);

        //copy back into real arrays
        for (unsigned i = 0; i < self->numberOfNonZeros(); ++i) {
            ir[i] = tempIr[i];
        }
        for (unsigned i = 0; i < self->columns() + 1; ++i) {
            jc[i] = tempJc[i];
        }
        //delete buffers
        delete [] tempIr;
        delete [] tempJc;

        return p;
    }

    // Convert to a dense matrix
    mxArray * toMatlabDense() const
    {
        mxArray *p  = mxCreateDoubleMatrix($self->rows(), $self->columns(), mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        //mapping output matrix to dense 
        memset(d, 0, sizeof(double) * self->rows() * self->columns());
        //mapping output matrix to dense
        for (iDynTree::SparseMatrix::const_iterator it(self->begin());
             it != self->end(); ++it) {
            d[self->rows() * it->column + it->row] = it->value;

        }
        return p;
    }


    // Convert from a dense or sparse matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const size_t * dims = mxGetDimensions(in);
        size_t rows = $self->rows();
        size_t cols = $self->columns();
        if (dims[0] != rows || dims[1] == cols)
        {
            $self->resize(dims[0], dims[1]);
            mexWarnMsgIdAndTxt("iDynTree:Core:perfomance", "Resizing iDynTree vector to (%d,%d)", rows, cols);
        }

        if (mxIsSparse(in))
        {
            mwIndex* ir = mxGetIr(in);
            mwIndex* jc = mxGetJc(in);
            double *data = mxGetPr(in);
            const size_t * dims = mxGetDimensions(in);
            mwIndex nnz = jc[dims[1]];

            int *tempIr = new int[nnz];
            if (!tempIr) return;

            int *tempJc = new int[dims[1] + 1];
            if (!tempJc) {
                delete [] tempIr;
                return;
            }

            for (unsigned i = 0; i < nnz; ++i) {
                tempIr[i] = ir[i];
            }

            for (unsigned i = 0; i < dims[1] + 1; ++i) {
                tempJc[i] = jc[i];
            }

            self->convertFromColumnMajor(dims[0], dims[1], 
                                          (unsigned)nnz,
                                          data, tempIr, tempJc);

            delete [] tempIr;
            delete [] tempJc;

        } else {
            double* d = static_cast<double*>(mxGetData(in));
            //for now very slow
            for (size_t row = 0; row < rows; row++)
            {
                for (size_t col = 0; col < cols; col++)
                {
                    $self->operator()(row, col) = d[col * rows + row];
                }
            }
            return;
        }
    }
}

}
