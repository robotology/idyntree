function v = LINK_MOMENT_OF_INERTIA_XX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',7,'LINK_MOMENT_OF_INERTIA_XX');
  end
  v = vInitialized;
end
