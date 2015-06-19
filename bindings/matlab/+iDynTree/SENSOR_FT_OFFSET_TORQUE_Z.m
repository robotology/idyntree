function v = SENSOR_FT_OFFSET_TORQUE_Z()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',18,'SENSOR_FT_OFFSET_TORQUE_Z');
  end
  v = vInitialized;
end
