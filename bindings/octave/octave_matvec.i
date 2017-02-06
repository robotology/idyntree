// Matlab specific swig code (to be executed after header parsing)

namespace iDynTree
{

%extend VectorFixSize
{
    // Convert to a dense matrix
    octave_value toMatlab() const
    {
        Matrix mat = Matrix($self->size(), 1);
        $self->fillBuffer(mat.fortran_vec()); // Column-major
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
        size_t fixValSize = $self->size();
        if( ( mat.rows() == fixValSize && mat.cols() == 1) ||
            ( mat.rows() == 1 && mat.cols() == fixValSize ) )
        {
            double* d = static_cast<double*>(mat.fortran_vec());
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
    octave_value toMatlab() const
    {
        Matrix mat = Matrix($self->rows(), $self->cols());
        $self->fillColMajorBuffer(mat.fortran_vec()); // Column-major
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
        size_t fixValRows = $self->rows();
        size_t fixValCols = $self->cols();
        if( mat.rows() == fixValRows && mat.cols() == fixValCols )
        {
            double* d = static_cast<double*>(mat.fortran_vec());
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
    octave_value toMatlab() const
    {
        Matrix mat = Matrix($self->size(), 1);
        $self->fillBuffer(mat.fortran_vec()); // Column-major
        return octave_value(mat);
    }

    // Convert from a dense matrix
    void fromMatlab(octave_value oct_val)
    {
        Matrix mat = oct_val.matrix_value();
        // check size
        if( ( mat.rows() == 1 || mat.cols() == 1) )
        {
            // Get the size of the input vector
            size_t inSize;
            if( mat.rows() == 1 )
            {
                inSize = mat.cols();
            }
            else
            {
                inSize = mat.rows();
            }

            // If the input vector has a size different
            // from the one of the iDynTree::VectorDynSize,
            // we resize iDynTree::VectorDynSize
            if( $self->size() != inSize )
            {
                $self->resize(inSize);
            }

            double* d = static_cast<double*>(mat.fortran_vec());
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
    octave_value toMatlab() const
    {
        Matrix mat = Matrix($self->rows(), $self->cols());
        $self->fillColMajorBuffer(mat.fortran_vec()); // Column-major
        return octave_value(mat);
    }

    // Convert from a dense matrix
    void fromMatlab(octave_value oct_val)
    {
        if( !oct_val.is_matrix_type() && !oct_val.is_sparse_type() )
        {
            std::cerr << "VectorFixSize::fromMatlab : expected matrix or sparse matrix argument" << std::endl;
            return;
        }

        // check size
        size_t currRows = $self->rows();
        size_t currCols = $self->cols();

        // Copy dense matrix
        if (oct_val.is_matrix_type())
        {
            Matrix mat = oct_val.matrix_value();

            if (mat.rows() != currRows && mat.cols() != currCols)
            {
                $self->resize(mat.rows(),mat.cols());
            }

            double* d = static_cast<double*>(mat.fortran_vec());
            for (size_t row=0; row < mat.rows(); row++)
            {
                for(size_t col=0; col < mat.cols(); col++ )
                {
                    $self->operator()(row,col) = d[col*mat.rows() + row];
                }
            }
            return;
        }

        // Copy sparse matrix
        if (oct_val.is_sparse_type())
        {
            SparseMatrix mat = oct_val.sparse_matrix_value();

            if (mat.rows() != currRows && mat.cols() != currCols)
            {
                $self->resize(mat.rows(),mat.cols());
            }


            for (size_t row=0; row < mat.rows(); row++)
            {
                for (size_t col=0; col < mat.cols(); col++ )
                {
                    $self->operator()(row,col) = mat(row,col);
                }
            }

        }
    }
}

}
