function v = RCM_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 35);
  end
  v = vInitialized;
end
