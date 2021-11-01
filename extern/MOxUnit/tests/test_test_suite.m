function test_suite=test_test_suite
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_test_suite_default_name()
    suite=MOxUnitTestSuite();
    assertEqual('MOxUnitTestSuite',getName(suite));

function test_test_suite_name()
    name=rand_str();
    suite=MOxUnitTestSuite(name);

    assertEqual(name,getName(suite));

function test_test_suite_tests()
    suite=MOxUnitTestSuite();

    assertEqual(0,countTestNodes(suite));

    n_nodes=ceil(rand()*5+20);
    for k=1:n_nodes
        nd=MOxUnitTestNode(rand_str());

        suite=addTest(suite,nd);
        assertEqual(k,countTestNodes(suite))
        nd_again=getTestNode(suite,k);
        assertEqual(nd,nd_again);
    end

    s_double=addFromSuite(suite,suite);
    assertEqual(2*n_nodes,countTestNodes(s_double));


function test_test_suite_set_test()
    suite=MOxUnitTestSuite();
    nd=MOxUnitTestNode(rand_str());
    suite=addTest(suite,nd);

    assertEqual(1,countTestNodes(suite))

    replacements={MOxUnitTestNode(['b' rand_str()]),...
                  MOxUnitTestNode(['c' rand_str()]),...
                  MOxUnitTestSuite(rand_str())};

    for k=1:numel(replacements)
        prev_nd=getTestNode(suite,1);
        replacement_nd=replacements{k};
        suite=setTestNode(suite,1,replacement_nd);
        assertFalse(isequal(prev_nd,getTestNode(suite,1)));
        assertEqual(getTestNode(suite,1),replacement_nd);
    end

function test_test_suite_set_test_exceptions()
    suite=MOxUnitTestSuite();
    nd=MOxUnitTestNode(rand_str());
    suite=addTest(suite,nd);

    bad_idxs={[1 1],.5,-1,2};
    for k=1:numel(bad_idxs)
        assertExceptionThrown(@()setTestNode(suite,bad_idxs{k},nd),'');
    end


function test_test_suite_run_partition_exceptions
    aet_run=@(varargin)assertExceptionThrown(@()...
                                       run(MOxUnitTestSuite(),...
                                                varargin{:}),'');
    aet_run(MOxUnitTestReport(),2);   % missing argument
    aet_run(MOxUnitTestReport(),3,2); % 3rd greater than 2nd argument
    aet_run(MOxUnitTestReport(),.5,2); % non-integer
    aet_run(MOxUnitTestReport(),struct(),2); % non-integer
    aet_run(MOxUnitTestReport(),-1,2); % negative
    aet_run(MOxUnitTestReport(),0,2); % zero
    aet_run(MOxUnitTestReport(),NaN,2); % zero
    aet_run(MOxUnitTestReport(),1,2.5); % non-integer
    aet_run(MOxUnitTestReport(),1,struct); % non-integer
    aet_run(MOxUnitTestReport(),1,-1); % negative
    aet_run(MOxUnitTestReport(),1,0); % zero
    aet_run(MOxUnitTestReport(),1,NaN); % zero




function test_test_suite_run()
    suite=MOxUnitTestSuite();

    test_partition_count=3+ceil(rand()*5);
    ntests_per_partitions=3+ceil(rand()*5);

    ntests=ntests_per_partitions*test_partition_count;
    test_cell=cell(ntests,1);
    should_pass=false(ntests,1);
    for k=1:ntests
        p=rand()>.5;
        should_pass(k)=p;

        if p
            func=@do_nothing;
        else
            func=@()error(rand_str());
        end

        test_name=sprintf('%d',k);
        test_case=MOxUnitFunctionHandleTestCase(test_name,...
                                                rand_str(),func);
        test_cell{k}=test_case;
        suite=addTest(suite,test_case);
    end

    verbosity=false;
    has_been_tested=false(ntests,1);
    for test_partition_index=1:test_partition_count
        empty_report=MOxUnitTestReport(verbosity);

        report=run(suite,empty_report,test_partition_index,...
                                        test_partition_count);
        test_outcome_count=countTestOutcomes(report);
        assertEqual(test_outcome_count,ntests_per_partitions);
        for j=1:countTestOutcomes(report)
            outcome=getTestOutcome(report,j);
            test_case=getTest(outcome);
            test_name=getName(test_case);
            test_number=sscanf(test_name,'%d');

            % no duplicates
            assert(~has_been_tested(test_number));
            has_been_tested(test_number)=true;

            assertEqual(should_pass(test_number),isSuccess(outcome));
        end

    end

    assert(all(has_been_tested));





function do_nothing()
    % do nothing

function s=rand_str()
    s=char(20*rand(1,10)+65);
