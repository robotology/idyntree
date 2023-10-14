function v = PURE_FORCE_WITH_KNOWN_DIRECTION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 14);
  end
  v = vInitialized;
end
