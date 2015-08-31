function v = LINK_MASS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 5);
  end
  v = vInitialized;
end
