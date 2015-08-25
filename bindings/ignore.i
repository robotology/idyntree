
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

// for some reason ignore ::operator= does not work, we disable directly the warning instead
#pragma SWIG nowarn=362

//}