function v = NET_EXT_WRENCH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 23);
  end
  v = vInitialized;
end
