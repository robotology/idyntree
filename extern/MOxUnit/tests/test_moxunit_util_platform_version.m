function test_suite=test_moxunit_util_platform_version()
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;


function test_platform_version
    % get start of version
    v=moxunit_util_platform_version();
    assert(isnumeric(v));
    assert(numel(v)>=2);
    version_start=sprintf('%d.%d',v(1:2));

    % now compare with output from 'ver'
    if moxunit_util_platform_is_octave()
        name='Octave';
    else
        name='MATLAB';
    end

    version_struct=ver(name);
    version_s=version_struct.Version;
    n=numel(version_start);

    assertEqual(version_s(1:n),version_start);


function v=dots_to_vec(s)
    parts=regexp(s,'\.','split');
    v=cellfun(@str2num,parts);



