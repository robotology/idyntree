function v = LINK_FIRST_MOMENT_OF_MASS_Y()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 7);
  end
  v = vInitialized;
end
