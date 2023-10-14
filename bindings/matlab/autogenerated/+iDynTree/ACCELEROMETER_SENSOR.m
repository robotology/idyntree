function v = ACCELEROMETER_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 27);
  end
  v = vInitialized;
end
