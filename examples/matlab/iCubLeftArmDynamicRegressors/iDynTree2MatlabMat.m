function [ matrix_matlab ] = iDynTree2MatlabeMat( matrix_idyntree )
%iDynTree2MatlabeMat Convert a Matlab matrix to a iDynTree MatrixDynSize
%   Temporary extremly slow function, proper support should 
%    be implemented at the C++ SWIG level. 

rows = matrix_idyntree.rows();
cols = matrix_idyntree.cols();

matrix_matlab = zeros(rows,cols);


for row = [0:(rows-1)]
    for col = [0:(cols-1)]
        matrix_matlab(row+1,col+1) = matrix_idyntree.getVal(row,col);
    end
end

end

