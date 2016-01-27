function v = LINK_MOMENT_OF_INERTIA_ZZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 17);
  end
  v = vInitialized;
end
