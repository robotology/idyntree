function v = SENSOR_FT_OFFSET_TORQUE_X()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 39);
  end
  v = vInitialized;
end
