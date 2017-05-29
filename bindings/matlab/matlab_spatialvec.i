// Matlab specific swig code for SpatialVector (to be executed after header parsing)

namespace iDynTree
{

%extend SpatialVector
{
    // Convert to a dense matrix
    mxArray * toMatlab() const
    {
        mxArray *p  = mxCreateDoubleMatrix(6, 1, mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        for(unsigned int i=0; i < 6; i++ )
        {
            d[i] = $self->operator()(i);
        }

        return p;
    }

    // Convert from a dense matrix
    void fromMatlab(mxArray * in)
    {
        // check size
        const mwSize * dims = mxGetDimensions(in);
        size_t fixValSize = $self->size();
        if( ( dims[0] == fixValSize && dims[1] == 1) ||
            ( dims[0] == 1 && dims[1] == fixValSize ) )
        {
            double* d = static_cast<double*>(mxGetData(in));
            for(size_t i=0; i < fixValSize; i++ )
            {
                $self->operator()(i) = d[i];
            }
            return;
        }
    }
}

}
