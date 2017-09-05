function v = BERDY_FLOATING_BASE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 15);
  end
  v = vInitialized;
end
