function v = NO_UNKNOWNS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 13);
  end
  v = vInitialized;
end
