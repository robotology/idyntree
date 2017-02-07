function v = LINK_BODY_PROPER_ACCELERATION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 13);
  end
  v = vInitialized;
end
