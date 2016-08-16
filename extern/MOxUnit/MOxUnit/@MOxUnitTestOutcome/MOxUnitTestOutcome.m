function obj=MOxUnitTestOutcome(test_,duration)
    s=struct();
    s.test=test_;
    s.duration=duration;
    obj=class(s,'MOxUnitTestOutcome');

