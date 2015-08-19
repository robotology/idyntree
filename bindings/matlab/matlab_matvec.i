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
        int fixValSize = $self->size();
        if( ( dims[0] == fixValSize && dims[1] == 1) ||
            ( dims[0] == 1 && dims[1] == fixValSize ) )
        {
            double* d = static_cast<double*>(mxGetData(in));
            double* selfData = $self->data();
            for(int i=0; i < fixValSize; i++ )
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