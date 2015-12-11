function v = LINK_MOMENT_OF_INERTIA_XY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 10);
  end
  v = vInitialized;
end
