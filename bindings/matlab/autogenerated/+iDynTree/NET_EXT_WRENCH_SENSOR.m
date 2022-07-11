function v = NET_EXT_WRENCH_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 31);
  end
  v = vInitialized;
end
