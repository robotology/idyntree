function v = ACCELEROMETER_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 15);
  end
  v = vInitialized;
end
