function v = SENSOR_FT_OFFSET_FORCE_Y()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',14,'SENSOR_FT_OFFSET_FORCE_Y');
  end
  v = vInitialized;
end
