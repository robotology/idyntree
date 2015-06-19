function v = LINK_MOMENT_OF_INERTIA_YZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',11,'LINK_MOMENT_OF_INERTIA_YZ');
  end
  v = vInitialized;
end
