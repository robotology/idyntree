// Matlab specific swig code (to be executed after header parsing)

namespace iDynTree
{

%extend VectorFixSize
{
    // Convert to a dense matrix
    octave_value toMatlab() const
    {
        Matrix p = Matrix($self->size(), 1);
        $self->fillBuffer(p.fortran_vec()); // Column-major
        return octave_value(p);
    }
}

}
