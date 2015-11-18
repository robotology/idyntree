function v = LINK_MOMENT_OF_INERTIA_YZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 13);
  end
  v = vInitialized;
end
