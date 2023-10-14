function v = InverseKinematicsRotationParametrizationRollPitchYaw()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 39);
  end
  v = vInitialized;
end
