function v = add()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 0);
  end
  v = vInitialized;
end
