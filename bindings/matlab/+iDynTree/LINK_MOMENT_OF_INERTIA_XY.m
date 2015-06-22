function v = LINK_MOMENT_OF_INERTIA_XY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',8,'LINK_MOMENT_OF_INERTIA_XY');
  end
  v = vInitialized;
end
