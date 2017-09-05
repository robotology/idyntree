function v = THREE_AXIS_FORCE_TORQUE_CONTACT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 9);
  end
  v = vInitialized;
end
