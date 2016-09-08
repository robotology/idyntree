function v = PURE_FORCE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 7);
  end
  v = vInitialized;
end
