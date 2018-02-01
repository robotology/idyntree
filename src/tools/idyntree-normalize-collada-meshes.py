# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Simple script to remove any transformation from the 
scene of the dae file and directly modify the points 
of the geometry instead. 

Useful for libraries (such as Irrlicht or assimp) that 
have problems in dealing with collada scenes with transformations.

This script assumes that every geometry is instantiated only once 
in the scene.

Dependencies: 
  * pycollada (pip: pycollada, apt : python-collada)
  * numpy (pip: numpy, apt : python-numpy)


@author: Silvio Traversaro
"""

import collada
import numpy as np
import os
import argparse
import sys

def getMatrixFromTransforms(transforms):
    ret_mat = np.identity(4);
    
    for trans in transforms:
        ret_mat = ret_mat.dot(trans.matrix);
        
    return ret_mat;

def getTransOfGeometriesFromNode(node):
    transformMap = {};
    # Get transformMap from children
    if hasattr(node,'children'):
        for child in node.children:
            transformMap = dict(list(transformMap.items()) + list(getTransOfGeometriesFromNode(child).items())) 
    
    # If node is geometry node, add an element to the transform map 
    if hasattr(node,'geometry'):
        transformMap[node.geometry.id] = np.identity(4);
        
    # If a node has a transforms, we should update the transform map 
    if hasattr(node,'transforms'):
        parent_H_node = getMatrixFromTransforms(node.transforms);
    else:
        parent_H_node = np.identity(4);
    
    for key in transformMap :
        transformMap[key] = parent_H_node.dot(transformMap[key]);
        
    return transformMap;
    

def getTransOfGeometriesFromScene(scene):
    transformMap = {};
    for node in scene.nodes:
        transformMap = dict(list(transformMap.items()) + list(getTransOfGeometriesFromNode(node).items())) 

    return transformMap;
    
def applyTransformToGeom(dae,scene_H_geom):
    for geom in dae.geometries:
        if( geom.id in scene_H_geom.keys() ):
            trans = scene_H_geom[geom.id];
            rot   = trans[0:3,0:3];
            pos   = trans[0:3,3];
            # The normal and vertices are shared among the different 
            # primitives, so it is ok to just run this on the first primitive
            primitiv = geom.primitives[0];
            for ind in range(0,len(primitiv.normal)):
                primitiv.normal[ind] = rot.dot(primitiv.normal[ind]);
            for ind in range(0,len(primitiv.vertex)):
                primitiv.vertex[ind] = rot.dot(primitiv.vertex[ind])+pos;
    return dae;

def removeTransformsFromNode(node):
    if hasattr(node,'children'):
        for child in node.children:
            child = removeTransformsFromNode(child);
            
    if hasattr(node,'transforms'):
        node.transforms = [];
        
    return node;


def removeTransformsFromScene(scene):
    for node in scene.nodes:
        node = removeTransformsFromNode(node);

    return scene
    
def normalizeColladaMesh(input_file,output_file):
    print('Normalizing ',input_file,' in ',output_file);
    dae_to_normalize = collada.Collada(input_file)

    # Check our hypothesis 
    # We support only one scene
    assert(len(dae_to_normalize.scenes) == 1)

    scene = dae_to_normalize.scenes[0];

    scene_H_geom = getTransOfGeometriesFromScene(scene);
    dae_to_normalize = applyTransformToGeom(dae_to_normalize,scene_H_geom);
    scene = removeTransformsFromScene(scene);

    dae_to_normalize.write(output_file)
    
    return
    
def normalizeColladaMeshDirectory(input_directory, output_directory):
    listOfFiles = os.listdir(input_directory);
  
    # Create output directory if it does not exist 
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)    
    
    
    inputDaeFiles = [];
    outputDaeFiles = [];
    for complete_filename in listOfFiles:
        filename, file_extension = os.path.splitext(complete_filename);
        if( file_extension == ".dae" ):
            inputDaeFiles.append(os.path.join(input_directory,complete_filename))
            outputDaeFiles.append(os.path.join(output_directory,os.path.basename(complete_filename)));
    
    for i in range(0,len(inputDaeFiles)):
        normalizeColladaMesh(inputDaeFiles[i],outputDaeFiles[i]);
    
    return
    
    
def main():
    parser = argparse.ArgumentParser(description="idyntree-normalize-collada-meshes.py : a simple program to normalize collada meshes to be used with Irrlicht or assimp", epilog="Remove transforms from the scene and put them in the meshes." ) 

    group = parser.add_mutually_exclusive_group()   
    group.add_argument('-f','--file', dest='file', nargs='?', action='store', help='file to convert')
    group.add_argument('-d','--directory', dest='directory', nargs='?', action='store', help='directory containing files to normalized')
    
    parser.add_argument('-o','--output', dest='outputs', nargs='?', action='store', help='either output file or directory where to put normalized files')

    args = parser.parse_args()
    
    if( args.file ):
        normalizeColladaMesh(args.file,args.outputs)
        
    if( args.directory ):
        normalizeColladaMeshDirectory(args.directory,args.outputs)


# This is the standard boilerplate that calls the main() function.
if __name__ == '__main__':
    rc = main()

    # This function will set the result value of this utility
    sys.exit( rc )
