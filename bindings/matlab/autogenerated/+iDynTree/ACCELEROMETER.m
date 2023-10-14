function v = ACCELEROMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 5);
  end
  v = vInitialized;
end
