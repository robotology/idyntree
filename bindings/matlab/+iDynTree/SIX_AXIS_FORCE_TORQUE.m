function v = SIX_AXIS_FORCE_TORQUE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 3);
  end
  v = vInitialized;
end
