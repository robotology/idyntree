function v = DOF_TORQUE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 17);
  end
  v = vInitialized;
end
