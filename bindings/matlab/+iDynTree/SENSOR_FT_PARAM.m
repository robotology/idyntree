function v = SENSOR_FT_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 4);
  end
  v = vInitialized;
end
