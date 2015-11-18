function v = LINK_MOMENT_OF_INERTIA_XZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 11);
  end
  v = vInitialized;
end
