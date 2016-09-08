function v = DOF_TORQUE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 14);
  end
  v = vInitialized;
end
