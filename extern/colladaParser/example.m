%% load data 

%select any of these to load 
%fname = 'virtualCity';         %model credit: esri CityEngine (http://www.esri.com/software/cityengine)
fname = 'fire_truck.dae';       %model credit: Antics (http://sketchup.google.com/3dwarehouse/details?mid=a8d99bb217f94f3a1ab9df4be75138d0)
%fname = 'us_police_car.dae';   %model credit: Antics (http://sketchup.google.com/3dwarehouse/details?mid=ce77b7e28b298ce21ab9df4be75138d0&ct=mdrm)

colFile = xml2struct(fname); 

%% unrolls scene geometry 

[sceneElements, sceneT] = colladaParser(colFile);



%% collect triangles into one mesh (for speed of visualizing)

tri = []; 
vertex = []; 
curTotal =0;

for i=1:length(sceneElements)
    nVertex = size(sceneElements(i).vertex,1); 
    vertex = cat(1,vertex,sceneElements(i).vertex); 
    tri = cat(1,tri,sceneElements(i).tri'+curTotal);
    curTotal = curTotal+nVertex;
end



%% plot scene

    figure(1); 
    set(gcf,'render','opengl'); 
    clf; 

    trimesh(tri,vertex(:,1),vertex(:,2),vertex(:,3),vertex(:,3));

    axis equal; 
    rotate3d on;
    grid on
    
    xlabel('x'); ylabel('y'); zlabel('z')