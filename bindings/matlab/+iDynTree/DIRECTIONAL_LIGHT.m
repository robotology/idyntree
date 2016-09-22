function v = DIRECTIONAL_LIGHT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 43);
  end
  v = vInitialized;
end
