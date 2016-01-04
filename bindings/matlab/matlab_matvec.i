// Matlab specific swig code (to be executed after header parsing)

namespace iDynTree
{

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
        if( ( dims[0] == fixValSize && dims[1] == 1) ||
            ( dims[0] == 1 && dims[1] == fixValSize ) )
        {
            double* d = static_cast<double*>(mxGetData(in));
            double* selfData = $self->data();
            for(size_t i=0; i < fixValSize; i++ )
            {
                selfData[i] = d[i];
            }
            return;
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

    // Convert from a dense matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const size_t * dims = mxGetDimensions(in);
        size_t fixValRows = $self->rows();
        size_t fixValCols = $self->cols();
        if( dims[0] == fixValRows && dims[1] == fixValCols )
        {
            double* d = static_cast<double*>(mxGetData(in));
            for(size_t row=0; row < fixValRows; row++ )
            {
                for(size_t col=0; col < fixValCols; col++ )
                {
                    $self->operator()(row,col) = d[col*fixValRows + row];
                }
            }
            return;
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
        if( ( dims[0] == 1 || dims[1] == 1) )
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
            }

            double* d = static_cast<double*>(mxGetData(in));
            double* selfData = $self->data();
            for(size_t i=0; i < inSize; i++ )
            {
                selfData[i] = d[i];
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
}

}
