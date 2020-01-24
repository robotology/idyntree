function v = InverseKinematicsTreatTargetAsConstraintPositionOnly()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 55);
  end
  v = vInitialized;
end
