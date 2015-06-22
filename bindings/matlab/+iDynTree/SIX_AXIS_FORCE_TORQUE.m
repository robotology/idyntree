function v = SIX_AXIS_FORCE_TORQUE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',0,'SIX_AXIS_FORCE_TORQUE');
  end
  v = vInitialized;
end
