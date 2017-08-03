function v = LINK_MOMENT_OF_INERTIA_XY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 34);
  end
  v = vInitialized;
end
