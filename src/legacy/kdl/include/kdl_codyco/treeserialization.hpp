/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef KDL_TREE_SERIALIZATION_HPP
#define KDL_TREE_SERIALIZATION_HPP

#ifdef __DEPRECATED
  #warning <treeserialization.hpp> is deprecated.
#endif

#include <string>
#include <kdl/tree.hpp>

namespace KDL{
namespace CoDyCo{

    /**
     * Class for describing a Tree serialization
     * (i.e. a mapping between:
     *      * Links and 0,1,...,nrOfLinks-1
     *      * DOFs and 0,1,...,nrOfJoints-1
     *      * Junction and 0,1,...,nrOfLinks-2 )
     *
     * Please note that currently for simplicity is assumed that the Junctions with a DOF
     * have the same id of their DOF (then all the fixed junctions have higher IDs)
     *
     * Assuming (as currently in KDL) that joints can have only 0 (fixed) or 1 dof.
     *
     * A serialization can be considered valid also if the id of an segment
     * is not always lower then the id of any of its children
     *
     * \todo : in solver inizialization, getLinkName is used for all the links
     *         should be the case to replace the vector with a more efficient (in query)
     *          data structure?
     *
     */
    class TreeSerialization {
    private:
        void addDFSrecursive(SegmentMap::const_iterator current_el,int & link_cnt, int & fixed_joint_cnt);

        void addDFSrecursive_only_fixed(SegmentMap::const_iterator current_el, int & fixed_joint_cnt);

        void addDFSrecursive_only_links(SegmentMap::const_iterator current_el,int & link_cnt);

        std::vector<std::string> links;
        std::vector<std::string> junctions;
        std::vector<std::string> dofs;

    public:


        TreeSerialization();

        /**
         * Constructor that builds the default serialization for KDL::Tree
         * For DOFs the one used to build the q_nr attribute in KDL::TreeElement
         * For links a DFS visit of the KDL tree is done
         * For joints the 1DOF joints have the same ID of their DOF, otherwise they are assigned a new ID higher then getNrOfDOFs
         */
        TreeSerialization(const Tree & tree);

        /**
         * Constructor that build the serialization for KDL::Tree
         * The link serialization is defined by the passed links_in vector
         *
         * The DOF and joints serialization is defined by the joints_in vector:
         * depending on its length it is considered as the vector of DOF or the vector of Junctions
         * In either cases it remains the constraint that the fixed junction have the higher IDs
         */
        TreeSerialization(const Tree & tree, std::vector<std::string> & links_in, std::vector<std::string> & joints_in);

        /**
         * Constructor that build the serialization for KDL::Tree

         * Using this constructor, the link serialization will be arbitrary.
         *
         * The DOF and junctions serialization is defined by the joints_in vector:
         * depending on its length it is considered as the vector of DOF or the vector of Junctions
         * In either cases it remains the constraint that the fixed junction have the higher IDs
         */
        TreeSerialization(const Tree & tree, std::vector<std::string> & joints_in);

        ~TreeSerialization();

        TreeSerialization(const TreeSerialization& x);

        /**
         * Returns the ID of a given junction in the serialization
         *
         * @param[in] joint_name the name of the junction for which the ID is requested
         * @return the requested ID of the junction
         *
         * \note Not efficient, performs a search
         */
        int getJunctionID(const std::string joint_name) const;

        bool setJunctionNameID(const std::string junction_name, const int new_ID);

        /**
         * Returns the ID of a given degree of freedom in the serialization
         *
         * @param[in] dof_name the name of the degree of freedom for which the ID is requested
         * @return the requested ID of the degree of freedom
         *
         * \note Not efficient, performs a search
         */
        int getDOFID(const std::string dof_name) const;

        bool setDOFNameID(const std::string dof_name, const int new_ID);


        /**
         * Returns the ID of a given link in the serialization
         *
         * @param[in] link_name the name of the link for which the ID is requested
         * @return the requested ID of the link
         *
         * \note Not efficient, performs a search
         */
        int getLinkID(const std::string link_name) const;

        bool setLinkNameID(const std::string link_name, const int new_ID);

        std::string getJunctionName(int joint_id) const;

        std::string getDOFName(int dof_id) const;

        std::string getLinkName(int link_id) const;

        /**
         * Set the number of Links
         */
        size_t setNrOfLinks(const size_t new_size);

        /**
         * Get the number of Links
         *
         */
        size_t getNrOfLinks() const;

        /**
         * Get the number of internal degrees of freedom
         */
        size_t setNrOfDOFs(const size_t new_size);

        /**
         * Get the number of internal degrees of freedom
         */
        size_t getNrOfDOFs() const;

        /**
         * Set the number of junctions
         */
        size_t setNrOfJunctions(const size_t new_size);


        /**
         * Get the number of joints of any DOF (not called getNrOfJoints to
         *   avoid confusion with the function of KDL::Tree/KDL::Chain)
         */
        size_t getNrOfJunctions() const;

        /**
         * Check if the TreeSerialization is a valid serialization for the
         * given Tree (checking also if the names in the serialization are
         *  the same of those in the tree)
         *
         * \note
         */
        bool is_consistent(const Tree & tree) const;

        /**
         * Load the serialization of the links from a vector of strings
         * @param[in] links_serialization a vector of string of getNrOfLinks() size
         *
         * @return true if the loading was ok, false if something went wrong (name of alink not recognized, error in vector size)
         *
         * \note this method does not affect the Junctions/DOFs serialization
         */
        bool loadLinksFromStringVector(const std::vector<std::string> & links_serialization);

        /**
         * Load the serialization of the links from a file
         * @param[in] file_name the name of the file that contains getNrOfLinks() lines, each one is the name of a link
         *
         * @return true if the loading was ok, false if something went wrong (name of alink not recognized, error in vector size)
         *
         * \note this method does not affect the Junctions/DOFs serialization
         */
        bool loadLinksFromFile(const std::string file_name);

        /**
         * Load the serialization of the junctions (and consequently of the DOFs) from a vector of strings
         * @param[in] junctions_serialization a vector of string of getNrOfJunctions() size, where the first getNrOfDOFs() junction are
         *  the non-fixed junctions
         *
         * @return true if the loading was ok, false if something went wrong (name of alink not recognized, error in vector size)
         *
         * \note this method does not affect the links serialization
         */
        bool loadJunctionsDOFsFromStringVector(const std::vector<std::string> & junctions_serialization);

        /**
         * Load the serialization of the links from a file
         * @param[in] file_name the name of the file that contains getNrOfJunctions() lines, each one is the name of a junction,
         *  where the first getNrOfDOFs() junction are  the non-fixed junctions
         *
         * @return true if the loading was ok, false if something went wrong (name of alink not recognized, error in vector size)
         *
         * \note this method does not affect the link serialization
         */
        bool loadJunctionsDOFsFromFile(const std::string file_name);

        /**
         * Convert the content of the serialization to a string.
         */
        std::string toString();
    };


}
}


#endif
