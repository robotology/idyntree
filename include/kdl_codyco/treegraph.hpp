// 	Copyright	 (C) 2013 Italian Institute of Technology
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef KDL_CODYCO_TREE_GRAPH_HPP
#define KDL_CODYCO_TREE_GRAPH_HPP

#include <string>
#include <map>

#include "kdl_codyco/treeserialization.hpp"

namespace KDL
{
namespace CoDyCo 
{
    //Forward declaration
    class TreeGraphLink;
    class TreeGraphJoint;
    
    /**
     * \todo substitute this map based structure with vector base one
     *       (originally implemented in this way for exploiting similarity
     *          with KDL::Tree 
     */ 
    typedef std::map<std::string,TreeGraphLink> LinkMap;
    typedef std::map<std::string,TreeGraphJoint> JointMap;
    
    struct Traversal {
        std::vector< LinkMap::const_iterator > order;
        std::vector< LinkMap::const_iterator > parent;
    };
    
    //Only supporting 1 dof joints
    typedef Twist TwistSubspace;

    class TreeGraphLink
    {
    private:
        TreeGraphLink(const std::string& name): link_name(name), link_nr(0) {};
        int globalIterator2localIndex(LinkMap::const_iterator link_iterator) const;
        
    public:
        std::string link_name;
        RigidBodyInertia I;
        unsigned int link_nr;
        unsigned int body_part_nr;
        std::vector< JointMap::const_iterator >  adjacent_joint;
        std::vector< LinkMap::const_iterator > adjacent_link;
        std::vector< bool > is_this_parent;
        
        
        TreeGraphLink(const std::string& name, const RigidBodyInertia & Inertia, const int nr): link_name(name), I(Inertia), link_nr(nr), adjacent_joint(0), adjacent_link(0), is_this_parent(0) { };
        ~TreeGraphLink() {};
        
        /**
         * Get the number of other links connected to this link
         * 
         * @return the number of adjacent links
         */
        unsigned int getNrOfAdjacentLinks() const;
        
        /**
         * Check if this link is adjacent to a given link
         */    
        bool is_adjacent_to(LinkMap::const_iterator link_iterator) const;

        
        /**
         * Get the pose of this link with respect to an adjacent link
         * 
         * @param adjancent_index the local index of the selected adjacent link
         * @param q the position of the joint connecting *this link to the adjacent link
         * @return  \f$ {}^a X_t\f$ the pose of this link (\f$t\f$) with respect to the adjacent link (\f$a\f$)
         */
        Frame pose(int adjacent_index, const double& q) const;
        
        /**
         * Get the pose of this link with respect to an adjacent link
         * 
         * @param adjancent_iterator the const_iterator to the selected adjacent link
         * @param q the position of the joint connecting *this link to the adjacent link
         * @return  \f$ {}^a X_t\f$ the pose of this link (\f$t\f$) with respect to the adjacent link (\f$a\f$)
         */
        Frame pose(LinkMap::const_iterator adjacent_iterator, const double& q) const;
        
        /**
         * Get the motion subspace S of the joint connecting this link to an adjacent link
         * expressed in the adjacent link reference frame
         * 
         * @param adjancent_index the local index of the selected adjacent link
         * @param q the position of the joint connecting this link to the adjacent link
         * @return  \f$ {}^a S_{t,a}\f$ the motion subspace of the joint connecting 
         *          this link (\f$t\f$) with the adjacent link (\f$a\f$), 
         *          expressed in (\f$a\f$). 
         *          It is defined such that:
         *          \f$ {}^a v_a = {}^a v_t + {}^a S_{t,a} \dot{q}\f$ 
         */
        TwistSubspace S(int adjacent_index, const double& q) const;
        
        /**
         * Get the motion subspace S of the joint connecting this link to an adjacent link
         * expressed in the adjacent link reference frame
         * 
         * @param adjancent_iterator the const_iterator to the selected adjacent link
         * @param q the position of the joint connecting this link to the adjacent link
         * @return  \f$ {}^a S_{t,a}\f$ the motion subspace of the joint connecting 
         *          this link (\f$t\f$) with the adjacent link (\f$a\f$), 
         *          expressed in (\f$a\f$). 
         *          It is defined such that:
         *          \f$ {}^a v_a = {}^a v_t + {}^a S_{t,a} \dot{q}\f$ 
         */
        TwistSubspace S(LinkMap::const_iterator adjacent_iterator, const double& q) const;
        
        /**
         * Joint velocity, expressed in the adjacent link reference frame
         * @return \f$ {}^a vj_{t,a} = {}^a S_{t,a} \dot{q}\f$
         */
        Twist vj(int adjacent_index, const double& q,const double& qdot) const;
        
        /**
         * Joint velocity, expressed in the adjacent link reference frame
         * @return \f$ {}^a vj_{t,a} = {}^a S_{t,a} \dot{q}\f$
         */
        Twist vj(LinkMap::const_iterator adjacent_iterator, const double& q,const double& qdot) const;
        //cj for the type of implemented joint is always 0
        
        JointMap::const_iterator getAdjacentJoint(int adjacent_index) const;
        JointMap::const_iterator getAdjacentJoint(LinkMap::const_iterator adjacent_iterator) const;
        
        std::string toString() const;


    };
    
    /**
     * Class for representing a joint connecting two links (parent and child) 
     * in the TreeGraph 
     * 
     */
    class TreeGraphJoint
    {
    private:
        TreeGraphJoint(const std::string& name):  joint_name(name), q_nr(0) { q_previous=-1.0; update_buffers(0.0);};
    
        mutable Frame relative_pose_parent_child; //\f$ {}^p X_c \f$
        mutable Twist S_child_parent; //\f$ {}^c S_{p,c} \f$
        mutable Twist S_parent_child; //\f$ {}^p S_{c,p} \f$
        mutable double q_previous;
        
        void update_buffers(const double & q) const;
        
    public:
        std::string joint_name;
        Joint joint;
        Frame f_tip;
        unsigned int q_nr;
        unsigned int body_part;
        
        LinkMap::const_iterator parent;
        LinkMap::const_iterator child;
        
        TreeGraphJoint(const std::string & name, const Joint & joint_in, const Frame & f_tip_in, const unsigned int q_nr_in = 0): 
                               joint_name(name), joint(joint_in), f_tip(f_tip_in), q_nr(q_nr_in) { q_previous=-1.0; update_buffers(0.0);};
        
        ~TreeGraphJoint() {}; 
        
        /**
         * Get the pose of one link with respect to the other link
         * 
         * @param q the position of the joint 
         * @return  \f$ {}^p X_c\f$ the pose of the child link (\f$c\f$) with respect to the parent link (\f$p\f$) if inverse is false
         *          \f$ {}^c X_p\f$ the pose of the parent link (\f$p\f$) with respect to the child link (\f$c\f$) if inverse is true
         */
        Frame pose(const double& q, const bool inverse) const;
        
        /**
         * Get the motion subspace of the joint
         * @param q the position of the joint
         * @return \f$ {}^p S_{c,p} \f$ the motion subspace of the joint such that \f$ {}^p v_p = {}^p v_c + {}^p S_{c,p} \dot{q} \f$ if inverse is false
         *         \f$ {}^c S_{p,c} \f$ the motion subspace of the joint such that \f$ {}^c v_c = {}^c v_p + {}^c S_{p,c} \dot{q} \f$ if inverse is true
         */
        Twist S(const double& q, bool inverse) const;
        
        /**
         * Joint velocity, expressed in the adjacent link reference frame
         * @return \f$ {}^p vj_{c,p} = {}^p S_{c,p} \dot{q} \f$  if inverse is false
         * @return \f$ {}^c vj_{c,p} = {}^c S_{p,c} \dot{q} \f$
         */
        Twist vj(const double& q, const double &dq, bool inverse) const;
        
        std::string toString() const;
    };

    /**
     * \brief  This class encapsulates a <strong>tree</strong>
     * kinematic interconnection structure. It is build out of link and joints.
     *
     * @ingroup KinematicFamily
     */
    class TreeGraph
    {
    private:
        LinkMap links;
        JointMap joints;
        int nrOfDOFs;
        int nrOfLinks;
        std::string tree_name;
        std::string original_root;
        
        /**
         * Private version of getLink, returning non-const iterator
         */
        LinkMap::iterator getLink(const std::string& name, bool dummy);
        
        /**
         * Private version of getJoint, returning non-const iterator
         */
        JointMap::iterator getJoint(const std::string& name, bool dummy);
        

    public:
        LinkMap::const_iterator getInvalidLinkIterator() const{ return links.end(); }
        JointMap::const_iterator getInvalidJointIterator() const { return joints.end(); }
        
        /**
         * 
         * \todo Added just for debug, remove it
         * 
         */
        TreeGraph() {};
        /**
         * The constructor of a TreeGraph, from a classic KDL::Tree
         * 
         * \note As TreeGraph must represent all and only the links complete 
         *       with inertia, the "root" as usually represented by KDL::Tree 
         *       is not added as a link.
         *       For this reason, it is required that the first joint of the 
         *       KDL::Tree used to create a TreeGraph is a fixed joint. 
         *       To avoid problems related to the diffent frame of reference
         *       of the virtual root and the actual root, it is recommended
         *       to mantain an identity transformation between the virtual root
         *       and the real one.
         * 
         * \todo solve issue related to non-const TreeGraph
         */
        TreeGraph(const Tree & tree,const TreeSerialization & serialization=TreeSerialization()); 

        /**
         * Request the total number of joints in the tree.\n
         *
         * @return total nr of joints
         */
        unsigned int getNrOfJoints() const { return nrOfLinks-1; };

        /**
         * Request the total number of degrees of freedom in the tree.\n
         *
         * @return total nr of DOFs
         */
        unsigned int getNrOfDOFs() const { return nrOfDOFs; };

        /**
         * Request the total number of links in the tree.
         * @return total number of links
         */
        unsigned int getNrOfLinks()const {return nrOfLinks;};
        
        LinkMap::const_iterator getLink(const std::string& name) const;
        //LinkMap::const_iterator getLink(const int index);
        
        JointMap::const_iterator getJoint(const std::string& name) const;
        //JointMap::const_iterator getJoint(const int index); 

        /**
         * Visit the TreeGraph with a Depth-first traversal
         * 
         * @param order the link visit order
         * @param parent the vector of parent for each link, for the base link 
         *        it contains a LinkMap::end() const_iterator
         * @param base_link the starting point of the traversal
         * @return 0 if all went well, another integer otherwise
         * 
         * \note not a real time operation
         * 
         * \todo add real time version, by specified base_link as index
         */
        int compute_traversal(Traversal & traversal, const std::string& base_link="", const bool bf_traversal=false) const;

        TreeSerialization getSerialization() const;
        
        int check_consistency() const;
        
        int check_consistency(const Traversal traversal) const;

        std::string toString() const;
        
        ~TreeGraph(){};

    };
}
}
#endif





