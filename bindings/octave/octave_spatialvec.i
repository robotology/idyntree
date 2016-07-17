// Matlab specific swig code for SpatialVector (to be executed after header parsing)

namespace iDynTree
{

%extend SpatialVector
{
    // Convert to a dense matrix
    octave_value toMatlab() const
    {
        Matrix mat = Matrix($self->size(), 1);
        for(unsigned int i=0; i < 6; i++ )
        {
            mat.fortran_vec()[i] = $self->operator()(i);
        }
        return octave_value(mat);
    }

    // Convert from a dense matrix
    void fromMatlab(octave_value oct_val)
    {
        if( !oct_val.is_matrix_type() )
        {
            std::cerr << "VectorFixSize::fromMatlab : expected matrix argument" << std::endl;
            return;
        }

        Matrix mat = oct_val.matrix_value();

        // check size
        size_t fixValSize = 6;
        if( ( mat.rows() == fixValSize && mat.cols() == 1) ||
            ( mat.rows() == 1 && mat.cols() == fixValSize ) )
        {
            double* d = static_cast<double*>(mat.fortran_vec());
            for(size_t i=0; i < fixValSize; i++ )
            {
                $self->operator()(i) = d[i];
            }
            return;
        }
    }
}

}
