function v = GYROSCOPE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 2);
  end
  v = vInitialized;
end
