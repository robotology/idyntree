function v = LINK_MOMENT_OF_INERTIA_XX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 7);
  end
  v = vInitialized;
end
