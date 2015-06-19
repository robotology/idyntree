function v = LINK_MOMENT_OF_INERTIA_ZZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',12,'LINK_MOMENT_OF_INERTIA_ZZ');
  end
  v = vInitialized;
end
