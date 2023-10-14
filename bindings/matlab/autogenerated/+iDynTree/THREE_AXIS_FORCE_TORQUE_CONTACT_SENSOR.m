function v = THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 30);
  end
  v = vInitialized;
end
