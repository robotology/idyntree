function v = SENSOR_FT_OFFSET_FORCE_Y()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 16);
  end
  v = vInitialized;
end
