function v = BODY_FIXED_REPRESENTATION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 3);
  end
  v = vInitialized;
end
