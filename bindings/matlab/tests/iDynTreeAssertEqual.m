function [ ] = iDynTreeAssertEqual( val1, val2 , errorMsg  )
%iDynTreeAssertEqual custom assert function
%      don't do anything if val1 == val2, print the error message
%      and exit otherwise 
%      
    if( val1 ~= val2 )
        disp([ 'iDynTree Tests: ' errorMsg])
        exit(1)
    end
end

