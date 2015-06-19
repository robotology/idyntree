function v = LINK_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',1,'LINK_PARAM');
  end
  v = vInitialized;
end
