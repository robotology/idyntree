function [ matrix_idyntree ] = Matlab2iDynTreeMat( matrix_matlab )
%Matlab2iDynTreeMat Convert an Matlab matrix to a iDynTree MatrixDynSize
%   Temporary extremly slow function, proper support should 
%    be implemented at the C++ SWIG level. 

rows = size(matrix_matlab,1);
cols = size(matrix_matlab,2);

matrix_idyntree = iDynTree.MatrixDynSize(rows,cols);

for row = [0:(rows-1)]
    for col = [0:(cols-1)]
        matrix_idyntree.setVal(row,col,matrix_matlab(row+1,col+1));
    end
end

end

