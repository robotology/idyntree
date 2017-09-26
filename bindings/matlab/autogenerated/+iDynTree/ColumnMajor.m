function v = ColumnMajor()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 1);
  end
  v = vInitialized;
end
