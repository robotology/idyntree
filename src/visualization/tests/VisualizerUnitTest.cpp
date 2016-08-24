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

int main()
{
    threeLinksReducedTest();

    return EXIT_SUCCESS;
}
