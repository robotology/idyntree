function v = LINK_MOMENT_OF_INERTIA_ZZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 14);
  end
  v = vInitialized;
end
