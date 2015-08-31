function v = SIX_AXIS_FORCE_TORQUE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 2);
  end
  v = vInitialized;
end
