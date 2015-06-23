function [ vector_matlab ] = iDynTree2MatlabVector( vector_idyntree )
%iDynTree2MatlabMatrix Convert an iDynTree vector to a Matlab vector
%   Temporary extremly slow function, proper support should 
%    be implemented at the C++ SWIG level. 

n = vector_idyntree.size();

vector_matlab = zeros(n,1);
for i = [0,n-1]
    vector_matlab(i+1) = vector_idyntree.getVal(i);
end

end

