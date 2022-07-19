function v = InverseKinematicsTreatTargetAsConstraintRotationOnly()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 40);
  end
  v = vInitialized;
end
