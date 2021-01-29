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

void checkAdditionalTexture() {
    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions mainWindowOptions, textureOptions;

    textureOptions.winWidth = 200;
    textureOptions.winHeight = 200;

    bool ok = viz.init(mainWindowOptions);
    ASSERT_IS_TRUE(ok);

    iDynTree::ITexture* texture = viz.textures().add("testTexture", textureOptions);
    iDynTree::ColorViz backGroundColor(0.1, 0.2, 0.3, 0.4);

    texture->environment().setBackgroundColor(backGroundColor);

    viz.draw();

    std::vector<iDynTree::PixelViz> pixels;
    ok = texture->getPixels(pixels);
    ASSERT_IS_TRUE(ok);

    ASSERT_IS_TRUE(pixels.size() == static_cast<size_t>(textureOptions.winWidth * textureOptions.winHeight));
    ASSERT_IS_TRUE(pixels[0].a = backGroundColor.a);
    ASSERT_IS_TRUE(pixels[0].r = backGroundColor.r);
    ASSERT_IS_TRUE(pixels[0].g = backGroundColor.g);
    ASSERT_IS_TRUE(pixels[0].b = backGroundColor.b);

}

void checkFrameVisualization() {
    iDynTree::Visualizer viz;

    viz.init();

    viz.camera().setPosition(iDynTree::Position(2.0, 0.0, 2.0));
    ASSERT_EQUAL_VECTOR(viz.camera().getPosition(), iDynTree::Position(2.0, 0.0, 2.0));
    viz.camera().setTarget(iDynTree::Position(0.0, 0.0, 0.0));
    ASSERT_EQUAL_VECTOR(viz.camera().getTarget(), iDynTree::Position(0.0, 0.0, 0.0));

    iDynTree::IFrameVisualization& frames = viz.frames();

    iDynTree::Transform firstTransform;
    firstTransform.setRotation(iDynTree::Rotation::RPY(1.57, 0,0));
    firstTransform.setPosition(iDynTree::Position(0.0, -1.0, 0.0));

    size_t index = frames.addFrame(firstTransform);
    ASSERT_IS_TRUE(index == 0);

    iDynTree::Transform secondTransform;
    secondTransform.setRotation(iDynTree::Rotation::RPY(-1.57, 0,0));
    secondTransform.setPosition(iDynTree::Position(0.0, +1.0, 0.0));

    index = frames.addFrame(secondTransform);
    ASSERT_IS_TRUE(index == 1);

    for(int i=0; i < 5; i++)
    {
        viz.draw();
    }

    bool ok = frames.updateFrame(index, firstTransform);
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 5; i++)
    {
        viz.draw();
    }

    iDynTree::Transform firstRead, secondRead;
    ok = frames.getFrameTransform(0, firstRead);
    ASSERT_IS_TRUE(ok);

    ok = frames.getFrameTransform(1, secondRead);
    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_TRANSFORM_TOL(firstRead, secondRead, 1e-5);
    ASSERT_EQUAL_TRANSFORM_TOL(firstRead, firstTransform, 1e-5);

    ok = frames.setVisible(0, false);
    ASSERT_IS_TRUE(ok);
    ok = frames.setVisible(1, false);
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 5; i++)
    {
        viz.draw();
    }

}

int main()
{
    threeLinksReducedTest();
    checkArrowsVisualization();
    checkAdditionalTexture();
    checkFrameVisualization();

    return EXIT_SUCCESS;
}
