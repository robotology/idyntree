function result=run(obj,result,test_partition_index,test_partition_count)
% Run the test suite
%
% result=run(obj[,result])
%
% Inputs:
%   obj                     MoxUnitTestSuite instance with tests to be run.
%   result                  MoxUnitTestResult instance to which test
%                           results are to be reported (default: empty
%                           MoxUnitTestResult instance).
%   partition_index         } Either both or neither of these arguments
%   partition_count         } must be passed to this function. It causes
%                             a subset of all tests to be run, namely the
%                             ones indexed by:
%                                partition_index+partition_count*K
%                             for all values of K. Default values are
%                             partition_index=1 and partition_count=1,
%                             meaning that all tests are run. A use case is
%                             parallelization of test cases over multiple
%                             processes.
%
% Output:
%   result          MoxUnitTestResult containing tests results
%                   after running the tests in obj.
%
% See also: MoxUnitTestResult
%
% NNO 2015

    if nargin<2
        result=MOxUnitTestReport();
    end

    if nargin<3
        test_partition_index=1;
        test_partition_count=1;
    elseif nargin<4
            error(['With third argument provided, the fourth argument '...
                        'is required']);
    end

    ensure_integer(test_partition_index,'test_partition_index');
    ensure_integer(test_partition_count,'test_partition_count');

    if test_partition_index>test_partition_count
        error('Test index %d greater than number of partitions %d',...
                test_partition_index,test_partition_count);
    end


    for j=test_partition_index:test_partition_count:numel(obj.tests)
        test_case=obj.tests{j};
        result=run(test_case, result);
    end

function ensure_integer(value, label)
    msg='';

    if ~isnumeric(value) || ~isscalar(value)
        msg='input must be scalar numeric';
    elseif round(value)~=value
        msg='input must be integer';
    elseif ~(value>0)
        msg='input must be positive';
    end

    if ~isempty(msg)
        error('''%s'' argument: %s',label,msg);
    end
