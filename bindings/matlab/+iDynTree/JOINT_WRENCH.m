function v = JOINT_WRENCH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 15);
  end
  v = vInitialized;
end
