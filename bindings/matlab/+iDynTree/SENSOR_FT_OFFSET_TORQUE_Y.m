function v = SENSOR_FT_OFFSET_TORQUE_Y()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMATLAB_wrap(0, 17);
  end
  v = vInitialized;
end
