function assertVectorsAlmostEqual(a,b,varargin)
% assert that two vectors are equal within certain tolerance
%
% assertVectorsAlmostEqual(a,b,[tol_type,tol,floor_tol][msg])
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
%   'moxunit:notVector'             a or b is not a vector
%   'moxunit:notFloat'              a or b is not a float array
%   'moxunit:differentClass         a and b are of different class
%   'moxunit:differentSize'         a and b are of different size
%   'moxunit:differentSparsity'     a is sparse and b is not, or
%                                       vice versa
%   'moxunit:floatsDiffer'          values in a and b are not
%                                       almost equal (see note below)
%
% Examples:
%   assertVectorsAlmostEqual([1 2 3 4],[1 2 3 4]);
%   %||  % ok
%
%   assertVectorsAlmostEqual([1 2; 3 4],[1 2; 3 4]);
%   %|| error('first input is not a vector')
%
%   assertVectorsAlmostEqual(1,1.0001);
%   %|| error('inputs are not equal within relative tolerance 1.49011e-08')
%
%   assertVectorsAlmostEqual(1,1.0001,'absolute',1e-3);
%   %|| % no error
%
%   assertVectorsAlmostEqual('foo','bar')
%   %|| error('first input is not float');
%
%
% Notes:
%   - If tol_type is 'relative', a and b are almost equal if
%
%           all(norm(a-b)<=tol*max(norm(a),norm(b))+floor_tol);
%
%   - If tol_type is 'absolute', a and b are almost equal if
%
%           all(norm(a-b)<=tol);
%
%   - Values are considered equal if they are both NaN, or both infinite
%     with the same sign.
%   - If a custom message is provided, then any error message is prefixed
%     by this custom message
%   - This function attempts to show similar behaviour as in
%     Steve Eddins' MATLAB xUnit Test Framework (2009-2012)
%     URL: http://www.mathworks.com/matlabcentral/fileexchange/
%                           22846-matlab-xunit-test-framework
%
% NNO Jan 2014

    metric=@norm;
    [message,error_id,whatswrong]=moxunit_util_floats_almost_equal(...
                                            a,b,metric,false,varargin{:});

    if isempty(error_id)
        if ~isvector(a)
            whatswrong='first input is not a vector';
            error_id='assertVectorsAlmostEqual:notVector';
        else
            % if a is a vector but b is not, then we cannot get here
            % because the size of a and b must be different
            return;
        end
    end

    full_message=moxunit_util_input2str(message,whatswrong,a,b);

    if moxunit_util_platform_is_octave()
        error(error_id,'%s',full_message);
    else
        throwAsCaller(MException(error_id,'%s',full_message));
    end

