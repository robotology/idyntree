/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SUB_MODEL_H
#define IDYNTREE_SUB_MODEL_H

#include <iDynTree/Model/Indeces.h>

#include <vector>
#include <string>
#include <cstddef>

namespace iDynTree
{
    class Model;
    class Traversal;

    /**
     * Class representing the decomposition in one model in several submodels.
     *
     * This class is used in algorithms, such as estimation of external wrenches,
     * where a complete model is decomposed in several submodels.
     *
     * For each submodel a iDynTree::Traversal is provided, so that algorithms
     * that are explicitly designed to run on both FullModel Traversal and SubModel
     * traversal can be executed on the SubModel.
     *
     */
    class SubModelDecomposition
    {
    private:
        /**
         * Vector of size nrOfSubModels traversal, one for each submodel.
         */
        std::vector<Traversal *> subModelTraversals;

        /**
         * Vector mapping link index to sub model index.
         */
        std::vector<size_t> link2subModelIndex;

        /**
         * Copy constructor is forbidden
         */
        SubModelDecomposition(const SubModelDecomposition & other);

        /**
         * Copy operator is forbidden
         */
        SubModelDecomposition& operator=(const SubModelDecomposition &other);

    public:
        /**
         * Constructor
         */
        SubModelDecomposition();

        /**
         * Destructor
         */
        ~SubModelDecomposition();

        /**
         * Create a submodel decomposition
         * of a given model with a given full tree traversal,
         * by separating the model across the given joints.
         *
         * @param[in] model the model to split
         * @param[in] traversal the full model traversal to split
         * @param[in] splitJoints the model will be split along
         *                        the joints whose names are in this vector.
         * @return true if all went fine, false otherwise
         */
        bool splitModelAlongJoints(const Model & model,
                                   const Traversal & traversal,
                                   const std::vector<std::string>& splitJoints);

        /**
         * Set the number of the submodels in the decomposition.
         */
        void setNrOfSubModels(const size_t nrOfSubModels);

        /**
         * Get the number of submodels in the decompistion.
         */
        size_t getNrOfSubModels() const;

        /**
         * Get the traversal of a given submodel
         */
        Traversal & getTraversal(const size_t subModelIndex);

        /**
         * Get the traversal of a given submodel (const version)
         */
        const Traversal & getTraversal(const size_t subModelIndex) const;

        /**
         * Get the subModel to which a given links belongs.
         */
        size_t getSubModelOfLink(const LinkIndex & link) const;
    };


}

#endif /* IDYNTREE_LINK_H */