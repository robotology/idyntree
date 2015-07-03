function [ ] = iDynTreeAssertEqual( val1, val2 , tol, errorMsg  )
%iDynTreeAssertEqual custom assert function
%      don't do anything if val1 == val2, print the error message
%      and exit otherwise
%
    if( norm(val1-val2) > tol )
        disp([ 'iDynTree Tests: ' errorMsg])
        disp([ 'mismatch between ' num2str(val1) ' and ' num2str(val2)])
        exit(1)
    end
end

