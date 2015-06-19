function v = SENSOR_FT_OFFSET_TORQUE_X()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0,'swigConstant',16,'SENSOR_FT_OFFSET_TORQUE_X');
  end
  v = vInitialized;
end
