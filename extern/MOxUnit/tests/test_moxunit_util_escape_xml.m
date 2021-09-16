function test_suite=test_moxunit_util_escape_xml
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;

function test_moxunit_util_escape_xml_basics
    aeq=@(a,b)assertEqual(moxunit_util_escape_xml(a),b);

    aeq('asdk','asdk');
    aeq('&','&amp;');
    aeq('"','&quot;');
    aeq('''','&apos;');
    aeq('<','&lt;');
    aeq('>','&gt;');

    aeq('apos&&&apos','apos&amp;&amp;&amp;apos');

    aeq('&amp&','&amp;amp&amp;');
    aeq('&quot','&amp;quot');
    aeq('&apos','&amp;apos');
    aeq('&lt','&amp;lt');
    aeq('&gt','&amp;gt');



