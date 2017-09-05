function v = FULL_WRENCH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 10);
  end
  v = vInitialized;
end
