tol = 1e-9;

dynComp = iDynTree.DynamicsComputations();

dynComp.loadRobotModelFromFile('./model.urdf');

dynComp.getFloatingBase();

disp('Test of DynamicsComputations completed successfully.');