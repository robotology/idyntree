function v = BERDY_FLOATING_BASE_NON_COLLOCATED_EXT_WRENCHES()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 18);
  end
  v = vInitialized;
end
