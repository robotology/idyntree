function v = InverseKinematicsTreatTargetAsConstraintFull()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 43);
  end
  v = vInitialized;
end
