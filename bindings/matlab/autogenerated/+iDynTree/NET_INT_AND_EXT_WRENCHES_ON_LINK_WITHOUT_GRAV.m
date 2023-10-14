function v = NET_INT_AND_EXT_WRENCHES_ON_LINK_WITHOUT_GRAV()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 20);
  end
  v = vInitialized;
end
