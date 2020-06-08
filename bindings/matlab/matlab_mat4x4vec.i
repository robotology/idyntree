// Matlab specific swig code (to be executed after header parsing)


%extend std::vector<iDynTree::Matrix4x4>
{
    // Convert to matlab matrix array
    mxArray * toMatlab() const
    {
        const mwSize NDIMS=3;
        mwSize nMatrices=$self->size();
        std::vector<iDynTree::Matrix4x4> :: const_iterator it =$self->begin();
        mwSize nRows=it->rows();
        mwSize nCols=it->cols();
        const mwSize dims[]={nMatrices,nRows,nCols};
        mxArray *p = mxCreateNumericArray(NDIMS,dims,mxDOUBLE_CLASS,mxREAL);
        double* d = static_cast<double*>(mxGetData(p));
        for (int matrixN = 0; matrixN < nMatrices; ++matrixN)
        {
            const double* content=it->data();
            for (int row = 0; row != nRows; ++row)
            {
                for (int col = 0; col != nCols; ++col)
                {
                    int index_sequence=(nCols*row + col);
                    size_t overall_index=matrixN+ nMatrices*(row +nRows* col);
                    d[overall_index]=content[index_sequence];
                }
            }
            
            ++it;
        }

        return p;


    }


}
