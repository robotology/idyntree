function v = URDFJointDynamics()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 3);
  end
  v = vInitialized;
end
