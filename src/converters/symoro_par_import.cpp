/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2013, Istituto Italiano di Tecnologia
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Silvio Traversaro */

#include "kdl_format_io/symoro_par_import.hpp"

#include "../expression_parser/parser.h"
#include <string>
#include <boost/iterator/iterator_concepts.hpp>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <kdl/tree.hpp>

using namespace KDL;
using namespace std;

namespace kdl_format_io {

/**
 * Matrix defined in Symoro+ documentation. It is obtained as:

 * return_matrix = Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)
 * That for the chain case (gamma = 0, b = 0) is:
 * return_matrix = Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)
 */
Frame DH_Khalil1986_Tree(double d, double alpha, double r, double theta, double gamma, double b)
{
        double ct,st,ca,sa,cgamma,sgamma;
        ct = cos(theta);
        st = sin(theta);
        sa = sin(alpha);
        ca = cos(alpha);
        cgamma = cos(gamma);
        sgamma = sin(gamma);
        return Frame(Rotation(
                              cgamma*ct-sgamma*ca*st,   -cgamma*st-sgamma*ca*ct,  sgamma*sa,
                              sgamma*ct+cgamma*ca*st,   -sgamma*st+cgamma*ca*ct, -cgamma*sa,
                                               st*sa,                     ct*sa,       ca   ),
                     Vector(
                                cgamma*d+sgamma*r*sa,      d*sgamma-cgamma*sa*r,  ca*r+b   )
                     );
}

    
bool treeFromSymoroParFile(const string& parfile_name, Tree& tree, const bool consider_first_link_inertia)
{
    ifstream ifs(parfile_name.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return treeFromSymoroParString(xml_string,tree,consider_first_link_inertia);
}

bool treeFromSymoroParString(const string& parfile_name, Tree& tree, const bool consider_first_link_inertia)
{
    symoro_par_model par_model;
    if( !parModelFromString(parfile_name,par_model) ) return false;
    
    return treeFromParModel(par_model,tree,consider_first_link_inertia);
}



bool parModelFromFile(const string& parfile_name, symoro_par_model& tree)
{
    ifstream ifs(parfile_name.c_str());
    std::string xml_string( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

    return parModelFromString(xml_string,tree);
}


/**
 * Check if a string contains a substring
 */
bool contains(const std::string & str, const std::string & substr)
{
    return (str.find(substr) != std::string::npos);
}

/**
 * Check if string str start with a given keyword
 * 
 */
bool begins_with(const std::string & str, const std::string & keyword)
{
    if( str.size() < keyword.size() ) return false;
    return( str.substr(0,keyword.size()) == keyword );
    
}

/**
 * Separate a given string with a given delimitator
 * 
 */

std::vector<std::string> explode(const string& str, const char& ch) {
    string next;
    vector<string> result;

    // For each character in the string
    for (string::const_iterator it = str.begin(); it != str.end(); it++) {
        // If we've hit the terminal character
        if (*it == ch) {
            // If we have some characters accumulated
            if (!next.empty()) {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
         result.push_back(next);
    return result;
}

/*
std::vector<std::string> explode(std::string const & s, char delim)
{
    std::vector<std::string> result;
    std::istringstream iss(s);

    for (std::string token; std::getline(iss, token, delim); )
    {
        result.push_back(std::move(token));
    }

    return result;
}*/

/**
 * Convert a string to a double
 */
double str2double(const std::string & str)
{
    double ret;
    std::istringstream ss(str);
    ss >> ret;
    return ret;
}

std::string int2string(const int in)
{
    std::stringstream ss;
    ss << in;
    return ss.str();
}




/**
 * Convert a string containing a declaration of a vector of T
 * in a std::vector<T>. Fails if the expression contains a unknown variable 
 * or constant. 
 */
template <class T>
bool extract_vector(const std::string & vector_string, std::string vector_name, std::vector<T> & vec, const std::vector<string> variables=std::vector<string>(0), const std::vector<T> var_values=std::vector<T>(0))
{
    if( variables.size() != var_values.size() ) return false;
    if( !contains(vector_string,"={") || !contains(vector_string,"}") ) return false;
    
    
    //Get position of =,{,}
    int eq_pos, open_pos, close_pos;
    eq_pos = vector_string.find("=");
    open_pos = vector_string.find("{");
    close_pos = vector_string.find("}",open_pos+1);
    
    vector_name = vector_string.substr(0,eq_pos);
   
    
    //Extracting the list of elements of the vector
    std::string vector_elems_string = vector_string.substr(open_pos+1,close_pos-open_pos-1);
    
    
    std::vector<string> vector_elems_strs = explode(vector_elems_string,',');
   
    
    //Create the parser object that is need to parse the mathematical expression that are allowed in SyMoRo+ par files
    //The only argument true means that all the unknown variable will be threated as zero
    Parser prs(true);
    
    //Add the user defined variable (E/e and Pi/pi/PI are already defined in the Parser)
    for(int i=0; i < variables.size(); i++ ) {
        bool check = prs.user_var.add(variables[i].c_str(),(double)var_values[i]);
        if( !check ) return false;
    }  
    
    //Then we parse the expression in the vector
    vec.resize(vector_elems_strs.size());
    
    for(int i=0; i < vector_elems_strs.size(); i++ ) {
        char * res;
        //std::cout << "Called parser with " << vector_elems_strs[i] << std::endl;
        res = prs.parse(vector_elems_strs[i].c_str());
        //printf("\t%s\t",res);
        //the +6 is to avoid the "Ans = " par of the result
        std::string res_cpp(res+6);
        //std::cout << "returning " << res_cpp << std::endl;
        vec[i] = (T)str2double(res_cpp);
        //std::cout << "Set in vector " << vec[i] << " ( " << str2double(res_cpp) << " ) " << std::endl;
    }
    
    return true;
}

/**
 * 
 * \todo add formula parsing
 */
bool parModelFromString(const string& _parfile_content, symoro_par_model & model)
{
   std::string parfile_content = _parfile_content;
    
    //Custom parser just to avoid adding another depencency on some fancy string manipulator/parser
    
    //Remove spaces and tabs (and CR) as they are not relevant to the contents of the file
    parfile_content.erase(std::remove(parfile_content.begin(), parfile_content.end(), ' '), parfile_content.end());
    parfile_content.erase(std::remove(parfile_content.begin(), parfile_content.end(), '\t'), parfile_content.end());
    parfile_content.erase(std::remove(parfile_content.begin(), parfile_content.end(), '\r'), parfile_content.end());
    
    //Create a string stream object to iterate line after line
    istringstream iss(parfile_content);
    
    std::string line;    
    while (std::getline(iss, line)) {
        //Condition for which ignore the line
        if( line.size() == 0 ) { continue; } //empty line
        if( line.substr(2) == "(*" ) { continue; } //comment line
        
        //In all other case, the line (and possibile the following lines) contain a variable definition
        //In Symoro+ the parameters can be specified as a numerical value (or a formula of numerical values)
        //or as a symbolic name. This parse support only the import of numerical values or simple formulas, and
        //only the importation of geometric parameters of chain or tree structures
        if( contains(line,"=") ) {
            if( begins_with(line,"NL") ) {
                stringstream ss(line.substr(3,line.size()-3));
                ss >> model.NL;
                continue;
            }
            if( begins_with(line,"NJ") ) {
                stringstream ss(line.substr(3,line.size()-3));
                ss >> model.NJ;
                continue;
            }
            if( begins_with(line,"NF") ) {
                stringstream ss(line.substr(3,line.size()-3));
                ss >> model.NF;
                continue;
            }
            if( begins_with(line,"Type") ) {
                stringstream ss(line.substr(5,1));
                ss >> model.Type;
                continue;
            }
        }
        
        //Vector definition
        std::string all_vector = "";
        std::string vector_new_line;
        std::vector<int> int_vec(0);
        std::vector<double> double_vec(0);
        if( contains(line,"={") ) {
            if( contains(line,"}") ) {
                //This if the vector is in the same line of the declaration
                all_vector = line;
            } else {
                all_vector = line;
                //Otherwise get all the lines over which the vector spans
                while (std::getline(iss, vector_new_line)) {
                    all_vector = all_vector+vector_new_line;
                    if( contains(vector_new_line,"}") ) break;
                }
            }
            
            std::string dummy_name;
            //At this point, all_vector contains a string that inside contains the vector
            //Depending on the param, we expect a vector of double or integers
            if( begins_with(all_vector,"Ant") ) {
                if( !extract_vector<int>(all_vector,dummy_name,int_vec) ) return false;
                model.Ant = int_vec;
                continue;
            }
            
            if( begins_with(all_vector,"Sigma") ) {
                if( !extract_vector<int>(all_vector,dummy_name,int_vec) ) return false;
                model.Sigma = int_vec;
                continue;
            }
            
            if( begins_with(all_vector,"Mu") ) {
                if( !extract_vector<int>(all_vector,dummy_name,int_vec) ) return false;
                model.Mu = int_vec;
                continue;
            }
            
            if( begins_with(all_vector,"B") ) {
                if( !extract_vector<double>(all_vector,dummy_name,double_vec) ) return false;
                model.B = double_vec;
                continue;
            }
            
            if( begins_with(all_vector,"d") ) {
                if( !extract_vector<double>(all_vector,dummy_name,double_vec) ) return false;
                model.d = double_vec;
                continue;
            }
            
            if( begins_with(all_vector,"R") ) {
                if( !extract_vector<double>(all_vector,dummy_name,double_vec) ) return false;
                model.R = double_vec;
                continue;
            }
            
            if( begins_with(all_vector,"gamma") ) {
                if( !extract_vector<double>(all_vector,dummy_name,double_vec) ) return false;
                model.gamma = double_vec;
                continue;
            }
            
            if( begins_with(all_vector,"Alpha") ) {
                if( !extract_vector<double>(all_vector,dummy_name,double_vec) ) return false;
                model.Alpha = double_vec;
                continue;
            }
            
            //In SyMoRo par file, for Theta is reported all the expression, so 
            //if the offset is 0.2 for the frist joint, it will report t1+0.2
            //We are interested only on the offset so we put all this variable to 0
            //to obtain only the offset
            std::vector<std::string> Theta_var(model.NJ);
            std::vector<double> Theta_var_values(model.NJ);
            for(int j=1; j <= model.NJ; j++ ) {
                std::string j_str = int2string(j);
                Theta_var[j-1] = "t" + j_str;
                Theta_var_values[j-1] = 0.0;
            }
            
            if( begins_with(all_vector,"Theta") ) {
                if( !extract_vector<double>(all_vector,dummy_name,double_vec,Theta_var,Theta_var_values) ) return false;
                model.Theta = double_vec;
                continue;
            }
        
        }
    }
    
    return true;
}

bool treeFromParModelTree(const symoro_par_model& par_model, Tree& tree, const bool consider_first_link_inertia)
{
    const std::string base_name = "Link0";
    tree = Tree(base_name);
    //As the SYMORO .par doesn't support names for link and joints, the links will be named Link1, Link2, and joints Joint1, Joint2
    const std::string link_common_name = "Link";
    const std::string joint_common_name = "Joint";
    
    std::vector<std::string> link_names(par_model.NL+1);
    std::vector<std::string> joint_names(par_model.NJ+1);
    
    link_names[0] = base_name;
    joint_names[0] = "joint_not_existing";
    
    for(int l=0; l < par_model.NL; l++ ) {
        std::string link_name = link_common_name + int2string(l+1);
        std::string joint_name = joint_common_name + int2string(l+1);
        link_names[l+1] = link_name;
        joint_names[l+1] = joint_name;
        std::string precessor_link_name = link_names[par_model.Ant[l]];
        if( par_model.Sigma[l] != 0 ) { std::cerr << "Warning: only rotational joint are currently tested" << std::endl; }
        
        //The parameters use the convention explained in Khalil 1986
        Frame f_parent_child = DH_Khalil1986_Tree(par_model.d[l],
                                                  par_model.Alpha[l],
                                                  par_model.R[l],
                                                  par_model.Theta[l],
                                                  par_model.gamma[l],
                                                  par_model.B[l]);
        
        
                                                 
        switch( par_model.Sigma[l] ) {
            case 0:
            {
                //Rotational joint 
                //The parameters use the convention explained in Khalil 1986
                //The transformation matrix used in symoro is T_parent_child = Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)
                //That can be factorized to fit in Segment RotAxis model (theta is the joint variable)  as: 
                //T_parent_child = Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(x,-d)*Rot(x,-alpha)*Trans(z,-b)*Rot(z,-gamma)*Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)*Trans(z,r)
                Frame new_old_axis_ref_frame = Frame(Rotation::RotZ(par_model.gamma[l]))*Frame(Vector(0,0,par_model.B[l]))*
                                               Frame(Rotation::RotX(par_model.Alpha[l]))*Frame(Vector(par_model.d[l],0,0));
                Vector jnt_axis_parent = new_old_axis_ref_frame.M*Vector(0,0,1);
                Vector jnt_origin_parent = new_old_axis_ref_frame*Vector(0,0,0);
                tree.addSegment(Segment(link_name,Joint(joint_name,jnt_origin_parent,jnt_axis_parent,Joint::RotAxis),f_parent_child),precessor_link_name);
            }
            break;
            case 1:
            {
                //Prismatic joint
               //The parameters use the convention explained in Khalil 1986
                //The transformation matrix used in symoro is T_parent_child = Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)
                //That can be factorized to fit in Segment RotAxis model (theta is the joint variable)  as: 
                //T_parent_child = Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)*Rot(z,-theta)*Trans(x,-d)*Rot(x,-alpha)*Trans(z,-b)*Rot(z,-gamma)*Rot(z,gamma)*Trans(z,b)*Rot(x,alpha)*Trans(x,d)               
                
                Frame new_old_axis_ref_frame = Frame(Rotation::RotZ(par_model.gamma[l]))*Frame(Vector(0,0,par_model.B[l]))*
                                               Frame(Rotation::RotX(par_model.Alpha[l]))*Frame(Vector(par_model.d[l],0,0))*Frame(Rotation::RotZ(par_model.Theta[l]));                
                Vector jnt_axis_parent = new_old_axis_ref_frame.M*Vector(0,0,1);
                Vector jnt_origin_parent = new_old_axis_ref_frame*Vector(0,0,0);
                tree.addSegment(Segment(link_name,Joint(joint_name,jnt_origin_parent,jnt_axis_parent,Joint::TransAxis),f_parent_child),precessor_link_name);
            }
            break;
            case 2: 
                //Fixed joint
                tree.addSegment(Segment(link_name,Joint(joint_name,Joint::None),f_parent_child),precessor_link_name);
            break;
            default:
            std::cerr << "Error: Sigma value not expected"<< std::endl; return false;
            break;
        }
        
       
    }
    return true;
    
}

bool treeFromParModelChain(const symoro_par_model& par_model, Tree& tree, const bool consider_first_link_inertia)
{

    //As the SYMORO .par doesn't support names for link and joints, the links will be named Link1, Link2, and joints Joint1, Joint2
    const std::string link_common_name = "Link";
    const std::string joint_common_name = "Joint";
    
    const std::string base_name = link_common_name + int2string(0);
    if( !consider_first_link_inertia ) {
        tree = Tree(base_name);
    } else {
        const std::string fake_base_name = "FakeBase_introduced_by_symoro_par_import";
        tree = Tree(fake_base_name);
        tree.addSegment(Segment(base_name,Joint("FakeFixedJoint_introduced_by_symoro_par_import",Joint::None)),fake_base_name);
    }
    
    std::vector<std::string> link_names(par_model.NL+1);
    std::vector<std::string> joint_names(par_model.NJ+1);
    
    link_names[0] = base_name;
    joint_names[0] = "joint_not_existing";
    
    for(int l=0; l < par_model.NL; l++ ) {
        std::string link_name = link_common_name + int2string(l+1);
        std::string joint_name = joint_common_name + int2string(l);
        link_names[l+1] = link_name;
        joint_names[l+1] = joint_name;
        std::string precessor_link_name = link_names[par_model.Ant[l]];
        if( par_model.Ant[l] != l ) { std::cerr << "Error in the structure of par chain" << std::endl; return false; }        
        Frame f_parent_child = DH_Khalil1986_Tree(par_model.d[l],
                                                  par_model.Alpha[l],
                                                  par_model.R[l],
                                                  par_model.Theta[l],
                                                  0,
                                                  0);
        switch( par_model.Sigma[l] ) {
            case 0:
            {
                //Rotational joint 
                //The parameters use the convention explained in Khalil 1986
                //The transformation matrix used in symoro is T_parent_child = Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)
                //That can be factorized to fit in Segment RotAxis Joint model (theta is the joint variable) as: 
                //T_parent_child = Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(x,-d)*Rot(x,-alpha)*Rot(x,alpha)*Trans(x,d)*Trans(z,r)
                Frame new_old_axis_ref_frame = Frame(Rotation::RotX(par_model.Alpha[l]))*Frame(Vector(par_model.d[l],0,0));
                Vector jnt_axis_parent = new_old_axis_ref_frame.M*Vector(0,0,1);
                Vector jnt_origin_parent = new_old_axis_ref_frame*Vector(0,0,0);
                tree.addSegment(Segment(link_name,Joint(joint_name,jnt_origin_parent,jnt_axis_parent,Joint::RotAxis),f_parent_child),precessor_link_name);
            }
            break;
            case 1:
            {
                //Prismatic joint
                //The parameters use the convention explained in Khalil 1986
                //The transformation matrix used in symoro is T_parent_child = Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)
                //That can be factorized to fit in Segment TransAxis Joint model (r is the joint variable) as: 
                //T_parent_child = Rot(x,alpha)*Trans(x,d)*Rot(z,theta)*Trans(z,r)*Rot(z,-theta)*Trans(x,-d)*Rot(x,-alpha)*Rot(x,alpha)*Trans(x,d)*Rot(z,theta)
                Frame new_old_axis_ref_frame = Frame(Rotation::RotX(par_model.Alpha[l]))*Frame(Vector(par_model.d[l],0,0))*Frame(Rotation::RotZ(par_model.Theta[l]));
                Vector jnt_axis_parent = new_old_axis_ref_frame.M*Vector(0,0,1);
                Vector jnt_origin_parent = new_old_axis_ref_frame*Vector(0,0,0);
                tree.addSegment(Segment(link_name,Joint(joint_name,jnt_origin_parent,jnt_axis_parent,Joint::TransAxis),f_parent_child),precessor_link_name);
            }
            break;
            case 2: 
                //Fixed joint
                tree.addSegment(Segment(link_name,Joint(joint_name,Joint::None),f_parent_child),precessor_link_name);
            break;
            default:
            std::cerr << "Error: Sigma value not expected"<< std::endl; return false;
            break;
        }
            
    }
    return true;
    
}

bool treeFromParModel(const symoro_par_model& par_model, Tree& tree, const bool consider_first_link_inertia)
{
    if( par_model.Type != 1 && par_model.Type != 0 ) { 
        std::cerr << "Error: currently are only supported SYMORO+ .par files of Type Tree (1) and Simple Chain (0)" << std::endl;
        return false;
    }
    
    if( !par_model.isConsistent() ) {
        std::cerr << "Error: the SYMORO par model is not consistent" << std::endl;
        return false;
    }
    
    if( par_model.Type == 1 ) return treeFromParModelTree(par_model,tree,consider_first_link_inertia);
    if( par_model.Type == 0 ) return treeFromParModelChain(par_model,tree,consider_first_link_inertia);
   
    std::cerr << "Error: currently are only supported SYMORO+ .par files of Type Tree (1) and Simple Chain (0)" << std::endl;
    return false;
}


}
