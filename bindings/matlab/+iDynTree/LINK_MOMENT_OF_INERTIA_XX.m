function v = LINK_MOMENT_OF_INERTIA_XX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 9);
  end
  v = vInitialized;
end
