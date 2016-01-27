function v = LINK_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 6);
  end
  v = vInitialized;
end
