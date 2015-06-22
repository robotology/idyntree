function v = LINK_FIRST_MOMENT_OF_MASS_Y()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',5,'LINK_FIRST_MOMENT_OF_MASS_Y');
  end
  v = vInitialized;
end
