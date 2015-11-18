function v = ACCELEROMETER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 1);
  end
  v = vInitialized;
end
