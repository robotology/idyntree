function v = DOF_ACCELERATION_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 28);
  end
  v = vInitialized;
end
