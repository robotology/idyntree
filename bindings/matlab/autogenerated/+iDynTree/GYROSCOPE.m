function v = GYROSCOPE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 6);
  end
  v = vInitialized;
end
