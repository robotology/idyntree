function v = InverseKinematicsTreatTargetAsConstraintFull()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 57);
  end
  v = vInitialized;
end
