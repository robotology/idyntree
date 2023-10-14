function v = DOF_TORQUE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 22);
  end
  v = vInitialized;
end
