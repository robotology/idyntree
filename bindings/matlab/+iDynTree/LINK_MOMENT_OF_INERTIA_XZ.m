function v = LINK_MOMENT_OF_INERTIA_XZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 32);
  end
  v = vInitialized;
end
