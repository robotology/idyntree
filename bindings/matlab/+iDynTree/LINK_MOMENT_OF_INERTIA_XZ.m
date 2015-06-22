function v = LINK_MOMENT_OF_INERTIA_XZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',9,'LINK_MOMENT_OF_INERTIA_XZ');
  end
  v = vInitialized;
end
