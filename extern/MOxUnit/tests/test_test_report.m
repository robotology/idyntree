function test_suite=test_test_report
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function s=randstr()
    s=char(20*rand(1,10)+65);

function test_test_report_output
    test_outcomes={{ @MOxUnitPassedTestOutcome},...
                    {@MOxUnitSkippedTestOutcome,randstr()},...
                    {@MOxUnitFailedTestOutcome,randstr()},...
                    {@MOxUnitErroredTestOutcome,randstr()}};

    for verbosity=0:2
        for report_test=[false,true];
            % open temporary output file
            temp_fn=tempname();
            fid=fopen(temp_fn,'w');
            file_closer=onCleanup(@()fclose(fid));

            % empty report
            rep=MOxUnitTestReport(verbosity,fid);
            assertEqual(0,countTestOutcomes(rep));
            assertTrue(wasSuccessful(rep));

            % add tests
            test_count=ceil(rand()*5+20);
            test_idxs=ceil(rand(test_count,1)*numel(test_outcomes));
            all_passed=true;

            outcome_str_cell=cell(test_count,1);
            test_stats=struct();
            assertEqual(test_stats,getTestOutputStatistics(rep));

            for k=1:test_count
                test_idx=test_idxs(k);

                % add test outcome
                outcome_cell=test_outcomes{test_idx};
                outcome_constructor=outcome_cell{1};
                outcome_args=outcome_cell(2:end);

                name=randstr();
                location=randstr();
                test_=MOxUnitFunctionHandleTestCase(name,location,'foo');
                duration=rand()*1e3;
                test_outcome=outcome_constructor(test_,duration,...
                                    outcome_args{:});

                is_non_failure=(sum(strcmp(class(test_outcome),...
                                    {'MOxUnitPassedTestOutcome',...
                                    'MOxUnitSkippedTestOutcome'}))>=1);

                % either report the test (which also adds it), or
                % only add the test (without reporting)
                if report_test
                    rep=reportTestOutcome(rep,test_outcome);
                else
                    rep=addTestOutcome(rep,test_outcome);
                end

                % verify number of tests
                assert(countTestOutcomes(rep)==k);
                all_passed=all_passed && is_non_failure;

                % very that wasSuccessful works properly
                assertEqual(all_passed,wasSuccessful(rep));

                % expected output from this test
                if report_test
                    outcome_str=getOutcomeStr(test_outcome,verbosity);
                    if verbosity==2
                        outcome_str=sprintf(' %s in %4.2f s: %s:  %s ',...
                                    outcome_str,duration,...
                                    name,location);
                    end
                else
                    outcome_str='';
                end

                % check getTestOutputStatistics
                label=getOutcomeStr(test_outcome,2);

                if ~isfield(test_stats,label)
                    test_stats.(label)=0;
                end
                test_stats.(label)=test_stats.(label)+1;

                assertEqual(test_stats,getTestOutputStatistics(rep));

                outcome_str_cell{k}=outcome_str;
            end

            % close temporary file
            clear file_closer;

            % read from temporary file
            fid=fopen(temp_fn);
            file_closer=onCleanup(@()fclose(fid));
            data=fread(fid,inf,'char=>char')';
            clear file_closer;

            % test that all elements are present
            expected_data=moxunit_util_strjoin(outcome_str_cell,'');
            assert_equal_modulo_whitespace(data, expected_data);
        end
    end

function test_test_report_duration
    rep=MOxUnitTestReport(0,1);

    assertEqual(getDuration(rep),0);

    duration=rand();
    outcome=MOxUnitPassedTestOutcome(1,duration);

    for k=1:3
        rep=addTestOutcome(rep,outcome);
        assertElementsAlmostEqual(getDuration(rep),k*duration);
    end



function test_test_report_name
    % default name
    rep=MOxUnitTestReport(0,1);
    assertEqual('MOxUnitTestReport',getName(rep));

    name=randstr();
    rep=MOxUnitTestReport(0,1,name);
    assertEqual(name,getName(rep));

function test_test_report_verbosity()
    stream=1;
    for verbosity=0:2
        rep=MOxUnitTestReport(verbosity,stream);
        assertEqual(getVerbosity(rep),verbosity);
    end

function test_test_report_stream()
    verbosity=1;
    for stream=[1 2]
        rep=MOxUnitTestReport(verbosity,stream);
        assertEqual(getStream(rep),stream);
    end


function test_test_report_get_statistics_str_xml
    helper_test_get_statistics_str('xml');

function test_test_report_get_statistics_str_text
    helper_test_get_statistics_str('text');

function helper_test_get_statistics_str(format)

    a_test=MOxUnitFunctionHandleTestCase(randstr,randstr,@abs);
    outcomes.passed=MOxUnitPassedTestOutcome(a_test,rand());
    outcomes.skipped=MOxUnitSkippedTestOutcome(a_test,rand(),'foo');
    outcomes.failure=MOxUnitFailedTestOutcome(a_test,rand(),struct());
    outcomes.error=MOxUnitErroredTestOutcome(a_test,rand(),struct());

    skip_pos=2;
    failure_pos=3;

    keys=fieldnames(outcomes);
    n_keys=numel(keys);


    for repeat=1:5
        rep=MOxUnitTestReport(0,1);
        counts=zeros(n_keys,1);

        for k=1:6
            idx=ceil(rand()*n_keys);
            key=keys{idx};

            outcome=outcomes.(key);
            rep=addTestOutcome(rep,outcome);
            counts(idx)=counts(idx)+1;

            s=getStatisticsStr(rep,format);

            switch format
                case 'xml'
                    assertEqual('',s);

                case 'text'
                    has_ok=all(counts(failure_pos:n_keys)==0);

                    assertEqual(has_ok,contains(s,'OK'));
                    assertEqual(~has_ok,contains(s,'FAILED'));

                    for j=skip_pos:n_keys
                        infix=sprintf('%s=%d',keys{j},counts(j));
                        assertEqual(contains(s,infix),counts(j)>0);
                    end

                otherwise
                    assert(false);
            end
        end
    end


function tf=contains(haystack,needle)
    tf=~isempty(strfind(haystack,needle));

function assert_contains(haystack,needle)
    assertTrue(contains(haystack,needle));

function test_write_xml_report()
    a_test=MOxUnitFunctionHandleTestCase(randstr,randstr,@abs);
    err=struct();
    err.message=randstr();
    err.stack=struct('file',randstr(),'line',rand(),'name',randstr());
    outcomes=struct();
    outcomes.passed=MOxUnitPassedTestOutcome(a_test,rand());
    outcomes.failure=MOxUnitFailedTestOutcome(a_test,rand(),err);

    keys=fieldnames(outcomes);
    for k=1:numel(keys)
        key=keys{k};
        outcome=outcomes.(key);

        rep=MOxUnitTestReport(0,1);
        rep=addTestOutcome(rep,outcome);


        fn=tempname();
        writeXML(rep,fn);

        fid=fopen(fn);
        content=fread(fid,inf,'*char')';
        file_closer=onCleanup(@()fclose(fid));


        assert_contains(content,'tests="1"');
        passed=wasSuccessful(rep);
        if passed
            assert_contains(content,'failures="0"')
            assert_contains(content,'errors="0"');
        else
            assert_contains(content,'failures="1"');
            assert_contains(content,'errors="0"');
        end

        assert_contains(content,'<?xml version');
        assert_contains(content,'<testsuites>');
        assert_contains(content,'</testsuites>');

        clear file_closer
    end


function assert_equal_modulo_whitespace(a,b)
    % In GNU Octave, strings may both be empty but of different size
    % (such as [1,0] and [0,0]). Such cases should not fail the test.
    both_empty=isempty(a)  && isempty(b);
    strsplit_by_whitespace=@(x)regexp(x,'\s+','split');
    if ~both_empty
        assertEqual(strsplit_by_whitespace(a),strsplit_by_whitespace(b));
    end




