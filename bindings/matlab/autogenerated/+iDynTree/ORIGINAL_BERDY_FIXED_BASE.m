function v = ORIGINAL_BERDY_FIXED_BASE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 14);
  end
  v = vInitialized;
end
