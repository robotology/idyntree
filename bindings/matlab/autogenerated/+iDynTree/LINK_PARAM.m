function v = LINK_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 27);
  end
  v = vInitialized;
end
