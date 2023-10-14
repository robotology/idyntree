function v = InverseKinematicsRotationParametrizationQuaternion()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 38);
  end
  v = vInitialized;
end
