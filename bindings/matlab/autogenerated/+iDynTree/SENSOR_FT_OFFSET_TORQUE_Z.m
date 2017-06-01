function v = SENSOR_FT_OFFSET_TORQUE_Z()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 43);
  end
  v = vInitialized;
end
