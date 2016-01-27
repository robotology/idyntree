function v = LINK_FIRST_MOMENT_OF_MASS_Z()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 11);
  end
  v = vInitialized;
end
