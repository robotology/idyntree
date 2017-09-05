function v = THREE_AXIS_ANGULAR_ACCELEROMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 8);
  end
  v = vInitialized;
end
