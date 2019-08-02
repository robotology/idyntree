/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Visualizer.h>

#include <iDynTree/Core/TestUtils.h>

#include "testModels.h"

void checkVizLoading(const iDynTree::Model & model)
{
    // Open visualizer
    iDynTree::Visualizer viz;

    bool ok = viz.addModel(model,"model");
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 5; i++)
    {
        viz.draw();
    }

    viz.close();
}

void threeLinksReducedTest()
{
    // Check visualizer of simple model
    iDynTree::ModelLoader mdlLoader, mdlLoaderReduced;

    // Load full model
    bool ok = mdlLoader.loadModelFromFile(getAbsModelPath("threeLinks.urdf"));
    ASSERT_IS_TRUE(ok);

    // Check visualization of full model
    checkVizLoading(mdlLoader.model());

    // Load reduced model
    std::vector<std::string> consideredJoints;
    consideredJoints.push_back("joint_2_3");
    ok = mdlLoaderReduced.loadReducedModelFromFullModel(mdlLoader.model(),consideredJoints);
    ASSERT_IS_TRUE(ok);

    // Check vizualization for reduced model
    checkVizLoading(mdlLoaderReduced.model());
}

void checkArrowsVisualization() {
    iDynTree::Visualizer viz;

    iDynTree::IVectorsVisualization& vectors = viz.vectors();

    size_t index = vectors.addVector(iDynTree::Position(0.1, 0.1, 0.0), iDynTree::Direction(0.0, -1.0, 0.0), 0.5);
    ASSERT_IS_TRUE(index >= 0);

    iDynTree::Vector3 components;
    components(0) = 0.2;
    components(1) = -0.5;
    components(2) = 0.1;

    index = vectors.addVector(iDynTree::Position(0.2, 0.1, 0.1), components);
    ASSERT_IS_TRUE(index >= 0);
    bool ok = vectors.setVectorColor(index, iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0));
    ASSERT_IS_TRUE(ok);
    components(0) = 0.5;
    components(1) = 0.0;
    components(2) = 0.1;
    ok = vectors.updateVector(index, iDynTree::Position(0.2, 0.1, 0.1), components);
    ASSERT_IS_TRUE(ok);



    for(int i=0; i < 5; i++)
    {
        viz.draw();
    }

    viz.close();
}

int main()
{
    threeLinksReducedTest();
    checkArrowsVisualization();

    return EXIT_SUCCESS;
}
