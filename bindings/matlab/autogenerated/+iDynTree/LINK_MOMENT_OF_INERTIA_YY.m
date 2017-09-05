function v = LINK_MOMENT_OF_INERTIA_YY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 41);
  end
  v = vInitialized;
end
