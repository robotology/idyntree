function v = THREE_AXIS_ANGULAR_ACCELEROMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 7);
  end
  v = vInitialized;
end
