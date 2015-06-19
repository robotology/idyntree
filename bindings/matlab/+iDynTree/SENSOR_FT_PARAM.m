function v = SENSOR_FT_PARAM()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',2,'SENSOR_FT_PARAM');
  end
  v = vInitialized;
end
