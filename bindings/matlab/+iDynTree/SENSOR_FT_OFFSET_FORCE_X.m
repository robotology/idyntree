function v = SENSOR_FT_OFFSET_FORCE_X()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 15);
  end
  v = vInitialized;
end
