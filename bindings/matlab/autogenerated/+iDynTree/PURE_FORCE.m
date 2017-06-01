function v = PURE_FORCE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 9);
  end
  v = vInitialized;
end
