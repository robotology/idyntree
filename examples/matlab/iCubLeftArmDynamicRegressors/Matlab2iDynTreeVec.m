function [ vector_idyntree ] = Matlab2iDynTreeVec( vector_matlab )
%iDynTree2MatlabMatrix Convert an Matlab vector to a iDynTree VectorDynSize
%   Temporary extremly slow function, proper support should 
%    be implemented at the C++ SWIG level. 

n = length(vector_matlab);

vector_idyntree = iDynTree.VectorDynSize(n);

for i = [0:(n-1)]
    vector_idyntree.setVal(i,vector_matlab(i+1));
end

end

