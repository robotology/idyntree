tol = 1e-9;

dynComp = iDynTree.DynamicsComputations();

dynComp.loadRobotModelFromFile('./model.urdf');

dynComp.getBaseLinkName();

disp('Test of DynamicsComputations completed successfully.');