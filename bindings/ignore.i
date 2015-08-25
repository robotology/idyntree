
// List of ignore rules, mostly to avoid warning related to const methods
//namespace iDynTree{

%ignore *::data() const;
%ignore *::operator()(const unsigned int index) const;
%ignore *::operator()(const unsigned int row, const unsigned int col) const;
%ignore *::getSemantics() const;
%ignore *::pos() const;
%ignore *::vel() const;
%ignore *::acc() const;
%ignore iDynTree::Model::getLink const;
%ignore iDynTree::Model::getJoint const;

// We use Curiously Recurring Template Pattern a lot in SpatialVector classes, but we want to avoid the warnings
#pragma SWIG nowarn=401

// Note that apparently CRTP is supported in SWIG,
// see http://swig.10945.n7.nabble.com/Does-SWIG-support-quot-template-of-current-class-quot-inheritance-td12158.html
// but I don't see the again in wrapping also the CRTP templates

// for some reason ignore ::operator= does not work, we disable directly the warning instead
#pragma SWIG nowarn=362

//}