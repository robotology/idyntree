function v = SENSOR_FT_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 25);
  end
  v = vInitialized;
end
