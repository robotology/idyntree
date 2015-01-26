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

#ifndef KDL_CODYCO_UNDIRECTED_TREE_HPP
#define KDL_CODYCO_UNDIRECTED_TREE_HPP

#include <string>
#include <map>

#include "kdl_codyco/treeserialization.hpp"

#define COMPUTE_TRAVERSAL_BASE_LINK_DEFAULT_VALUE -10000

namespace KDL
{
namespace CoDyCo
{
    //Forward declaration
    class UndirectedTreeLink;
    class UndirectedTreeJunction;
    class UndirectedTree;

    /**
     *
     * \todo
     *
     */
    //typedef std::map<std::string,UndirectedTreeLink> LinkMap;
    //typedef std::map<std::string,UndirectedTreeJunction> JunctionMap;
    typedef std::vector<UndirectedTreeLink> LinkMap; /**< Actually a vector, named this way for backward compatibility */
    typedef std::vector<UndirectedTreeJunction> JunctionMap;  /**< Actually a vector, named this way for backward compatibility */

    typedef std::map<std::string,LinkMap::iterator> LinkNameMap;
    typedef std::map<std::string,JunctionMap::iterator> JunctionNameMap;

    /**
     * This class represent a traversal, given a root, of a KDL::CoDyCo::UndirectedTree
     * It containts:
     *     * a traversal order vector (the index is the visit order number) starting at the given root, that respects the property that a link is visited after his parent
     *     * a parent vector (the index is the link id) such that for each link it returns its parent under the given root
     */
    class Traversal {

    friend class UndirectedTree;

    private:
        std::vector< LinkMap::const_iterator > order;
        std::vector< LinkMap::const_iterator > parent;

    public:
        Traversal(): order(0), parent(0) {};
        ~Traversal() {};

        /**
         * Get the number of visited links.
         * This is the nrOfLinks of the relative KDL::CoDyCo::UndirectedTree if the traversal is a FullTreeTraversal,
         * or a number between 0 and nrOfLinks it this is a SubTreeTraversal.
         *
         * @return the number of links in the Traversal
         */
        int getNrOfVisitedLinks() const;

        /**
         * Get a link visited by the traversal given its order in the traversal (a number between 0 and getNrOfVisitedLinks()-1
         *
         * @param order_number a number between 0 and getNrOfVisitedLinks()-1 specifyng the order position in the Traversal
         *        of the requested link
         *
         * @return a LinkMap::const_iterator of the requested element
         */
        LinkMap::const_iterator getOrderedLink(int order_number) const;

        /**
         * Get the base link (i.e. the first link) of the traversal
         *
         * @return the same thing as getOrderedLink(0)
         */
        LinkMap::const_iterator getBaseLink() const;

        /**
         * Get the parent of a link in the considered traversal
         */
        LinkMap::const_iterator getParentLink(int link_number) const;

        /**
         * Get the parent of a link in the considered traversal
         */
        LinkMap::const_iterator getParentLink(LinkMap::const_iterator) const;
    };

    class FullTreeTraversal : public Traversal
    {
    public:
        FullTreeTraversal(const KDL::CoDyCo::UndirectedTree & undirected_tree, std::string base_link );
        FullTreeTraversal(const KDL::CoDyCo::UndirectedTree & undirected_tree, int base_link_index );
        ~FullTreeTraversal() {};
    };

    class SubTreeTraversal : public Traversal
    {
        SubTreeTraversal(const KDL::CoDyCo::UndirectedTree & undirected_tree, std::string base_link, std::vector<std::string> cut_junctions );
        SubTreeTraversal(const KDL::CoDyCo::UndirectedTree & undirected_tree, int base_link_index, std::vector<int> cut_junctions_indeces );
        ~SubTreeTraversal() {};
    };

    //Only supporting 1 dof joints
    typedef Twist TwistSubspace;

    class UndirectedTreeLink
    {

    friend class UndirectedTree;
    friend class UndirectedTreeJunction;

    private:
        UndirectedTreeLink(const std::string& name): link_name(name), link_nr(-1) {};
        int globalIterator2localIndex(LinkMap::const_iterator link_iterator) const;

        std::string link_name;
        RigidBodyInertia I;
        int link_nr;
        std::vector< JunctionMap::const_iterator >  adjacent_joint;
        std::vector< LinkMap::const_iterator > adjacent_link;
        std::vector< bool > is_this_parent; /**< true if this link is the actual parent (for the KDL::Joint) of the junction, false otherwise */

    public:

        UndirectedTreeLink() {};
        UndirectedTreeLink(const std::string& name, const RigidBodyInertia & Inertia, const int nr): link_name(name), I(Inertia), link_nr(nr), adjacent_joint(0), adjacent_link(0), is_this_parent(0) {   };
        ~UndirectedTreeLink() {};

        UndirectedTreeLink & operator=(UndirectedTreeLink const &x) { if ( this != &x ) {
            link_name = x.link_name;
            I = x.I;
            link_nr = x.link_nr;

            adjacent_joint = x.adjacent_joint;
            adjacent_link = x.adjacent_link;
            is_this_parent = x.is_this_parent;} return *this; }

        std::string getName() const {return link_name;}

        int getLinkIndex() const {return link_nr;}

        RigidBodyInertia getInertia() const {return I;}

        void setInertia(const RigidBodyInertia & Inertia) { this->I = Inertia; }


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
        Frame & pose(int adjacent_index, const double& q) const;

        /**
         * Get the pose of this link with respect to an adjacent link
         *
         * @param adjancent_iterator the const_iterator to the selected adjacent link
         * @param q the position of the joint connecting *this link to the adjacent link
         * @return  \f$ {}^a X_t\f$ the pose of this link (\f$t\f$) with respect to the adjacent link (\f$a\f$)
         */
        Frame & pose(LinkMap::const_iterator adjacent_iterator, const double& q) const;

        /**
         * Get the motion subspace S of the joint connecting this link to an adjacent link
         * expressed in the adjacent link reference frame
         *
         * \todo Change the semantic of adjacent_index, it is confusing?
         *
         * @param adjancent_index the local index of the selected adjacent link
         * @param q the position of the joint connecting this link to the adjacent link
         * @return  \f$ {}^a S_{t,a}\f$ the motion subspace of the joint connecting
         *          this link (\f$t\f$) with the adjacent link (\f$a\f$),
         *          expressed in (\f$a\f$).
         *          It is defined such that:
         *          \f$ {}^a v_a = {}^a v_t + {}^a S_{t,a} \dot{q}\f$
         */
        TwistSubspace & S(int adjacent_index, const double& q) const;

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
        TwistSubspace & S(LinkMap::const_iterator adjacent_iterator, const double& q) const;

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

        /**
         * \note adjacent index is an index local to each UndirectedTreeLink
         *
         * \todo Change the semantic of adjacent_index, it is confusing?
         *
         */
        JunctionMap::const_iterator getAdjacentJoint(int adjacent_index) const;
        LinkMap::const_iterator getAdjacentLink(int adjacent_index) const;

        JunctionMap::const_iterator getAdjacentJoint(LinkMap::const_iterator adjacent_iterator) const;

        std::string toString() const;


    };

    /**
     * Class for representing a joint connecting two links (parent and child)
     * in the UndirectedTree
     *
     */
    class UndirectedTreeJunction
    {

    friend class UndirectedTree;
    friend class UndirectedTreeLink;

    private:
        UndirectedTreeJunction(const std::string& name):  joint_name(name), q_nr(-1) { q_previous=-1.0; update_buffers(0.0);};


        mutable Frame relative_pose_parent_child; /**< \f$ {}^p X_c \f$ */
        mutable Frame relative_pose_child_parent; /**< \f$ {}^c X_p \f$ */
        mutable Twist S_child_parent; /**< \f$ {}^c S_{p,c} \f$ */
        mutable Twist S_parent_child; /**< \f$ {}^p S_{c,p} \f$ */
        mutable double q_previous;

        void update_buffers(const double & q) const;

        std::string joint_name;
        Joint joint;
        Frame f_tip;
        int q_nr;

        LinkMap::const_iterator parent;
        LinkMap::const_iterator child;

    public:

        UndirectedTreeJunction() {};
        UndirectedTreeJunction(const std::string & name, const Joint & joint_in, const Frame & f_tip_in, const int q_nr_in = -1, const int body_part_in = -1, const int body_part_q_nr_in=-1):
                               joint_name(name), joint(joint_in), f_tip(f_tip_in), q_nr(q_nr_in) { q_previous=-1.0; update_buffers(0.0);};


        ~UndirectedTreeJunction() {};

        std::string getName() const { return joint_name; }

        int getJunctionIndex() const { return q_nr; }

        int getDOFIndex() const { return q_nr; }

        const Joint & getJoint() const { return joint; }

        LinkMap::const_iterator getParentLink() const { return parent; }

        LinkMap::const_iterator getChildLink() const { return child; }




        /**
         * Get the number of degrees of freedom of the junction
         *
         * \note the library currently supports only 0 (Joint::None) or 1 degree of freedom
         *
         */
        int getNrOfDOFs() const { if( joint.getType() == Joint::None ) { return 0; } else { return 1; } }

        /**
         * Get the pose of one link with respect to the other link
         *
         * @param q the position of the joint
         * @return  \f$ {}^p X_c\f$ the pose of the child link (\f$c\f$) with respect to the parent link (\f$p\f$) if inverse is false
         *          \f$ {}^c X_p\f$ the pose of the parent link (\f$p\f$) with respect to the child link (\f$c\f$) if inverse is true
         */
        Frame & pose(const double& q, const bool inverse) const;

        /**
         * Get the motion subspace of the joint
         * @param q the position of the joint
         * @return \f$ {}^p S_{c,p} \f$ the motion subspace of the joint such that \f$ {}^p v_p = {}^p v_c + {}^p S_{c,p} \dot{q} \f$ if inverse is false
         *         \f$ {}^c S_{p,c} \f$ the motion subspace of the joint such that \f$ {}^c v_c = {}^c v_p + {}^c S_{p,c} \dot{q} \f$ if inverse is true
         */
        Twist & S(const double& q, bool inverse) const;

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
    class UndirectedTree
    {
    private:
        LinkMap links;
        JunctionMap junctions;

        LinkNameMap links_names;
        JunctionNameMap junctions_names;

        int nrOfDOFs;
        //int nrOfLinks;

        std::string tree_name;
        std::string original_root;





       /**
        * Private helper function for constructor
        *
        */
        void constructor(const Tree & tree, const TreeSerialization & serialization);

    public:
        LinkMap::const_iterator getInvalidLinkIterator() const{ return links.end(); }
        JunctionMap::const_iterator getInvalidJunctionIterator() const { return junctions.end(); }

        /**
         * Private version of getLink, returning non-const iterator
         */
        LinkMap::iterator getLink(const std::string& name, bool dummy);

        /**
         * Private version of getJoint, returning non-const iterator
         */
        JunctionMap::iterator getJunction(const std::string& name, bool dummy);

        /**
         * Reserved version of getLink, returning non-const iterator
         */
        LinkMap::iterator getLink(const int index, bool dummy);

        /**
         * Reserved version of getJoint, returning non-const iterator
         */
        JunctionMap::iterator getJunction(const int index, bool dummy);



        /**
         *
         * \todo Added just for debug, remove it
         *
         */
        UndirectedTree() {};
        /**
         * The constructor of a UndirectedTree, from a classic KDL::Tree
         *
         * \note As UndirectedTree must represent all and only the links complete
         *       with inertia, the "root" as usually represented by KDL::Tree
         *       is not added as a link.
         *       For this reason, it is required that the first joint of the
         *       KDL::Tree used to create a UndirectedTree is a fixed joint.
         *       To avoid problems related to the diffent frame of reference
         *       of the virtual root and the actual root, it is recommended
         *       to mantain an identity transformation between the virtual root
         *       and the real one.
         *
         * \todo solve issue related to non-const UndirectedTree
         */
        UndirectedTree(const Tree & tree,const TreeSerialization & serialization=TreeSerialization());


        /**
         * Copy constructor
         */
        UndirectedTree(const UndirectedTree& in);

        UndirectedTree& operator=(const UndirectedTree& in);


        /**
         * Request the total number of joints in the tree.\n
         *
         * @return total nr of joints
         */
        unsigned int getNrOfJunctions() const { return getNrOfLinks()-1; };

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
        unsigned int getNrOfLinks() const {return links.size();};

        /**
         * Get a link of the graph by specifyng its name
         */
        LinkMap::const_iterator getLink(const std::string& name) const;

        /**
         * Get a link of the graph by specifyng its index
         */
        LinkMap::const_iterator getLink(const int index) const;

        /**
         * Get a junction of the graph by specifyng its name
         */
        JunctionMap::const_iterator getJunction(const std::string& name) const;

        /**
         * Get a junction of the graph by specifyng its index
         */
        JunctionMap::const_iterator getJunction(const int index) const;

        /**
         * Visit the UndirectedTree with a Depth-first traversal
         *
         * @param order the link visit order
         * @param parent the vector of parent for each link, for the base link
         *        it contains a LinkMap::end() const_iterator
         * @param base_link the name of the link used as the starting point of the traversal
         * @return 0 if all went well, another integer otherwise
         *
         * \note not a real time operation
         *
         * \todo add real time version, by specifyng base_link as index (and by dealing with the deque)
         */
        int compute_traversal(Traversal & traversal, const std::string& base_link, const bool bf_traversal=false) const;

         /**
         * Visit the UndirectedTree with a  traversal
         *
         * @param base_link_index the id of the link used as the starting point of the traversal
         * @return 0 if all went well, another integer otherwise
         *
         * \note not a real time operation
         *
         * \todo add real time version, by specifyng base_link as index (and by dealing with the deque)
         */
        int compute_traversal(Traversal & traversal, const int base_link_index=COMPUTE_TRAVERSAL_BASE_LINK_DEFAULT_VALUE, const bool bf_traversal=false) const;

        /**
         * Compute the traversal for a subtree of the undirected tree.
         * For specifyng the subtree, it is necessary to specify the base link and
         * the junctions that separate the subtree for the rest of the tree
         *
         * @param traversal the result traversal
         * @param base_link the name of the base link
         * @param cut_junctions a vector of the names of the junctions that separated the subtree from the rest of the tree
         * @return 0 if all went well, another integer if there where errors
         */
        int compute_traversal_subtree(Traversal & traversal, const std::string& base_link, const std::vector<std::string> & cut_junctions );

        /**
         * Compute the traversal for a subtree of the undirected tree.
         * For specifyng the subtree, it is necessary to specify the base link and
         * the junctions that separate the subtree for the rest of the tree
         *
         * @param traversal the result traversal
         * @param base_link the index of the base link
         * @param cut_junctions a vector of the indeces of the junctions that separated the subtree from the rest of the tree
         * @return 0 if all went well, another integer if there where errors
         */
        int compute_traversal_subtree(Traversal & traversal, const int base_link_index, const std::vector<int> & cut_junctions_indeces);


        Tree getTree(std::string base="") const;

        TreeSerialization getSerialization() const;


        int check_consistency() const;

        int check_consistency(const Traversal traversal) const;

        std::string toString() const;

        ~UndirectedTree(){};

    };
}
}
#endif





