function v = POINT_LIGHT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 44);
  end
  v = vInitialized;
end
