function v = SENSOR_FT_OFFSET_FORCE_Z()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',15,'SENSOR_FT_OFFSET_FORCE_Z');
  end
  v = vInitialized;
end
