function v = InverseKinematicsTreatTargetAsConstraintRotationOnly()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 56);
  end
  v = vInitialized;
end
