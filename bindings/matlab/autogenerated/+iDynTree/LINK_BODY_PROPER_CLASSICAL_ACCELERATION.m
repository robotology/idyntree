function v = LINK_BODY_PROPER_CLASSICAL_ACCELERATION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 25);
  end
  v = vInitialized;
end
