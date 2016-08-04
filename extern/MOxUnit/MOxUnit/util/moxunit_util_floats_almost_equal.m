function [message,error_id,whatswrong]=moxunit_util_floats_almost_equal(a,b,f,nonfin_eq,varargin)
% compare equality of two float arrays up to certain tolerance
%
% [message,error_id,whatswrong]=moxunit_util_floats_almost_equal(a,b,f,nonfin_eq,[,tol_type,tol,floor_tol,msg])
%
% Inputs:
%   a           float array
%   b           float array
%   f           function handle to a function that takes one vector input
%               and returns a scalar output.
%   nonfin_eq   if true, non-finite elements are treated equal if they
%               are both NaN or both Infinity with the same sign
%   tol_type    'relative' or 'absolute' (default: 'relative')
%   tol         tolerance       } default: sqrt(eps) if a is double,
%   floor_tol   floor_tolerance } sqrt(eps('single')) otherwise)
%   msg         optional custom message
%
% Output:
%   message     the contents of msg, if provided; empty ('') otherwise
%   id          the empty string ('') if a and b are almost equal,
%               otherwise:
%               'moxunit:notFloat'           a or b is not a float array
%               'moxunit:differentClass      a and b are of different class
%               'moxunit:differentSize'      a and b are of different size
%               'moxunit:differentSparsity'  a is sparse and b is not, or
%                                            vice versa
%               'moxunit:floatsDiffer'       values in a and b are not
%                                            almost equal (see note below)
%   whatswrong  if id is not empty, a human-readible description of the
%               inequality between a and b
%
% Notes:
%   - Typical values for the function handle f are:
%     * @abs:  element-wise comparison
%     * @norm: vector comparison
%   - If tol_type is 'relative', a and b are almost equal if
%
%           all(f(a(:)-b(:))<=tol*max(f(a(:)),f(b(:)))+floor_tol);
%
%   - If tol_type is 'absolute', a and b are almost equal if
%
%           all(f(a(:)-b(:))<=tol);
%
%   - It follows that if any value in a or b is not finite (+Inf, -Inf, or
%     NaN), then a and b are not almost equal.
%   - This is a helper function for assertElementsAlmostEqual and
%     assertVectorsAlmostEqual
%
% See also: assertElementsAlmostEqual, assertVectorsAlmostEqual
%
% NNO Jan 2014


    [message,tol_type,tol,floor_tol]=get_params(a,varargin{:});


    if ~isequal(size(a), size(b))
        whatswrong='inputs are not of the same size';
        error_id=get_error_id(f, 'sizeMismatch');
    elseif ~isfloat(a)
        whatswrong='first input is not float';
        error_id=get_error_id(f, 'notFloat');
    elseif ~isnumeric(b)
        whatswrong='second input is not float';
        error_id=get_error_id(f, 'notFloat');
    else
        whatswrong='';
        error_id=[];
    end

    if ~isempty(error_id)
        return;
    end


    switch tol_type
        case 'relative'
            test_func=@(x,y) f(x-y)<=tol*max(f(y),f(x))+floor_tol;
        case 'absolute'
            test_func=@(x,y) f(x-y)<=tol;

        otherwise
            error('compareFloats:unrecognizedToleranceType',...
                    'unsupported tolerance type %s', tol_type);
    end

    a_vec=a(:);
    b_vec=b(:);
    if isreal(a)
        cmp_func=@()test_func(a_vec,b_vec);
    else
        cmp_func=@()test_func(real(a_vec),real(b_vec)) & ...
                        test_func(imag(a_vec),imag(b_vec));
    end

    msk_equal=cmp_func();

    if nonfin_eq
        msk_equal(~isfinite(a_vec))=false;
        msk_equal(nonfinite_elements_equal(a_vec,b_vec))=true;
    else
        msk_equal=msk_equal & isfinite(a_vec) & isfinite(b_vec);
    end

    all_equal=all(msk_equal);

    if ~all_equal
        whatswrong=sprintf(['inputs are not equal within '...
                                '%s tolerance %d'],tol_type,tol);

        error_id=get_error_id(f, 'tolExceeded');
    end

function msk_equal=nonfinite_elements_equal(a,b)
    msk_nan=isnan(a) & isnan(b);
    msk_inf=isinf(a) & isinf(b) & sign(a)==sign(b);

    msk_equal=msk_nan | msk_inf;


function error_id=get_error_id(f, postfix)
    f_name=func2str(f);
    switch f_name
        case 'abs'
            prefix='assertElementsAlmostEqual';
        case 'norm'
            prefix='assertVectorsAlmostEqual';
        otherwise
            prefix=sprintf('assertAlmostEqual%s',f_name);
    end

    error_id=[prefix ':' postfix];


function [message,tol_type,tol,floor_tol]=get_params(a,varargin)
    n=numel(varargin);

    tol_type=[];
    tol=[];
    floor_tol=[];
    message='';

    for k=1:n
        arg=varargin{k};
        if ischar(arg)
            if (strcmp(arg,'relative') || strcmp(arg,'absolute')) && ...
                    isnumeric(tol_type)

                tol_type=arg;
                continue;

            elseif k==n && isempty(message)
                message=arg;
                continue
            else
                error('compareFloats:unrecognizedToleranceType',...
                        ['Tolerance type must be ''absolute'' '...
                            'or ''relative''']);
            end

        elseif isscalar(arg)
            if isempty(tol)
                tol=arg;
                continue
            elseif isempty(floor_tol)
                floor_tol=arg;
                continue
            end
        end

        error('compareFloats:illegalParameter',...
                    'Illegal argument at position %d', k);
    end

    % set defaults for values that were not set
    if isempty(floor_tol)
        switch class(a)
            case 'double'
                default_tol=sqrt(eps);
            case 'single'
                default_tol=sqrt(eps('single'));
            otherwise
                % set to illegal value; it is never used because
                % the calling function raises an error before tol or
                % floor_tol are used
                default_tol=NaN;
        end

        floor_tol=default_tol;
    end

    if isempty(tol)
        tol=default_tol;
    end

    if isempty(tol_type)
        tol_type='relative';
    end


