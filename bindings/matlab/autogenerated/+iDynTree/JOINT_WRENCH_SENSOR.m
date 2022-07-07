function v = JOINT_WRENCH_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 32);
  end
  v = vInitialized;
end
