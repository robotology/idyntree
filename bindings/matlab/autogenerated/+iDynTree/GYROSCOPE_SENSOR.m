function v = GYROSCOPE_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 21);
  end
  v = vInitialized;
end
