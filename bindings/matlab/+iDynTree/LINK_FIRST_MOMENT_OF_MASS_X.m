function v = LINK_FIRST_MOMENT_OF_MASS_X()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',4,'LINK_FIRST_MOMENT_OF_MASS_X');
  end
  v = vInitialized;
end
