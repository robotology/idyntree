function v = InverseKinematicsTreatTargetAsConstraintNone()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 54);
  end
  v = vInitialized;
end
