function assertElementsAlmostEqual(a,b,varargin)
% assert that two arrays are equal within certain tolerance
%
% assertElementsAlmostEqual(a,b,[tol_type,tol,floor_tol][msg])
%
% Inputs:
%   a               float array
%   b               float array
%   tol_type        'relative' or 'absolute' (default: 'relative')
%   tol             tolerance       } default: sqrt(eps) if a is double,
%   floor_tol       floor_tolerance } sqrt(eps('single')) otherwise)
%   msg             optional custom message
%
% Raises:
%   'moxunit:notFloat'              a or b is not a float array
%   'moxunit:differentClass         a and b are of different class
%   'moxunit:differentSize'         a and b are of different size
%   'moxunit:differentSparsity'     a is sparse and b is not, or
%                                         vice versa
%   'moxunit:floatsDiffer'          values in a and b are not
%                                       almost equal (see note below)
%
% Notes:
%   - If tol_type is 'relative', a and b are almost equal if
%
%           all(abs(a(:)-b(:))<=tol*max(abs(a(:)),abs(b(:)))+floor_tol);
%
%   - If tol_type is 'absolute', a and b are almost equal if
%
%           all(abs(a(:)-b(:))<=tol);
%
%   - It follows that if any value in a or b is not finite (+Inf, -Inf, or
%     NaN), then a and b are not almost equal.
%   - This function attempts to show similar behaviour as in
%     Steve Eddins' MATLAB xUnit Test Framework (2009-2012)
%     URL: http://www.mathworks.com/matlabcentral/fileexchange/
%                           22846-matlab-xunit-test-framework
%
% NNO Jan 2014

    metric=@abs;
    [message,error_id,whatswrong]=moxunit_util_floats_almost_equal(...
                                             a,b,metric,true,varargin{:});

    if isempty(error_id)
        return;
    end

    full_message=moxunit_util_input2str(message,whatswrong,a,b);

    if moxunit_util_platform_is_octave()
        error(error_id,full_message);
    else
        throwAsCaller(MException(error_id, full_message));
    end



