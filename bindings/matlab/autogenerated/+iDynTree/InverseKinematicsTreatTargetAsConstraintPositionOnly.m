function v = InverseKinematicsTreatTargetAsConstraintPositionOnly()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 41);
  end
  v = vInitialized;
end
