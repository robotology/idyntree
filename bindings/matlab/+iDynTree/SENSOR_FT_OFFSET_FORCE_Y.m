function v = SENSOR_FT_OFFSET_FORCE_Y()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 16);
  end
  v = vInitialized;
end
