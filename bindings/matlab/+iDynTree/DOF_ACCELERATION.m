function v = DOF_ACCELERATION()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 16);
  end
  v = vInitialized;
end
