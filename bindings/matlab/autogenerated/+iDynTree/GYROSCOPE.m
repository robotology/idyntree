function v = GYROSCOPE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 9);
  end
  v = vInitialized;
end
