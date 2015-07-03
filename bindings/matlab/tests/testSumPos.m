pos1_m = [10;20;30];
pos2_m = [1;2;3];

tol = 1e-9;

pos1 = iDynTree.Position();
pos2 = iDynTree.Position();

pos1.setVal(0,pos1_m(1));
pos1.setVal(1,pos1_m(2));
pos1.setVal(2,pos1_m(3));

pos2.setVal(0,pos2_m(1));
pos2.setVal(1,pos2_m(2));
pos2.setVal(2,pos2_m(3));

pos3 = pos1+pos2;
pos3_m = pos1_m+pos2_m;

iDynTreeAssertEqual(pos3.getVal(0),pos3_m(1),tol,'Error in summing points, x coord')
iDynTreeAssertEqual(pos3.getVal(1),pos3_m(2),tol,'Error in summing points, y coord')
iDynTreeAssertEqual(pos3.getVal(2),pos3_m(3),tol,'Error in summing points, z coord')
