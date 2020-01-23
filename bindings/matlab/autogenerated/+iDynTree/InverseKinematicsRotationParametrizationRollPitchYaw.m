function v = InverseKinematicsRotationParametrizationRollPitchYaw()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 53);
  end
  v = vInitialized;
end
