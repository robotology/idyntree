function v = LINK_MOMENT_OF_INERTIA_XZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 35);
  end
  v = vInitialized;
end
