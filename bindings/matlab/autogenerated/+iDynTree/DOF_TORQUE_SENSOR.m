function v = DOF_TORQUE_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 30);
  end
  v = vInitialized;
end
