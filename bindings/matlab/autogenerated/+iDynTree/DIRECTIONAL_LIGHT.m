function v = DIRECTIONAL_LIGHT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 46);
  end
  v = vInitialized;
end
