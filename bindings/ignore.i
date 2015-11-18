
// List of ignore rules
//namespace iDynTree{

// Avoid warning related to const methods (of type:
// Warning 512: Overloaded method .. const ignored,
// Warning 512: using non-const method ... instead)

%ignore *::data() const;
%ignore *::operator+=;
%ignore *::operator()(const unsigned int index) const;
%ignore *::operator()(const unsigned int row, const unsigned int col) const;
%ignore *::getSemantics() const;
%ignore *::pos() const;
%ignore *::vel() const;
%ignore *::acc() const;
%ignore iDynTree::Model::getLink const;
%ignore iDynTree::Model::getJoint const;
%ignore *::getAngularVec3() const;
%ignore *::getLinearVec3() const;

// for some reason ignore ::operator= does not work, we disable directly the warning instead
#pragma SWIG nowarn=362

//}
