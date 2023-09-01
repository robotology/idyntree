function v = NET_EXT_WRENCH_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 33);
  end
  v = vInitialized;
end
