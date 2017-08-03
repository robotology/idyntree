function v = GYROSCOPE_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 22);
  end
  v = vInitialized;
end
