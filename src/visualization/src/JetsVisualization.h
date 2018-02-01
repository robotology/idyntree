/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_JETS_VISUALIZATION_H
#define IDYNTREE_JETS_VISUALIZATION_H

#include <iDynTree/Visualizer.h>

#include <irrlicht.h>

namespace iDynTree
{

class ModelVisualization;

class JetsVisualization: public IJetsVisualization
{
private:
    JetsVisualization(const JetsVisualization& other);
    JetsVisualization& operator=(const JetsVisualization& other);

    void drawJet(size_t i, double intensity);

    irr::scene::ISceneManager* m_smgr;
    ModelVisualization * m_modelViz;
    std::vector<irr::scene::ISceneNode *> * m_framesNodes;

    std::vector<FrameIndex> m_jetsFrameIndices;
    std::vector<irr::scene::ISceneNode *> m_jetsNodes;
    std::vector<Direction> m_jetsDirections;
    std::vector<ColorViz>  m_jetsColors;

    double m_maxRadius;
    double m_minRadius;
    double m_maxLength;

    void reset();

public:
    JetsVisualization();
    void init(irr::scene::ISceneManager* smgr, ModelVisualization * modelViz, std::vector<irr::scene::ISceneNode *> * framesNodes);
    ~JetsVisualization();

    virtual bool setJetsFrames(const std::vector< std::string > & jetsFrames);
    virtual size_t getNrOfJets() const;
    virtual Direction getJetDirection(const int jetIndex) const;
    virtual bool setJetDirection(const int jetIndex, const Direction & jetDirection);
    virtual bool setJetColor(const int jetIndex, const ColorViz & jetColor);
    virtual bool setJetsDimensions(const double & minRadius, const double & maxRadius, const double & maxLenght);
    virtual bool setJetsIntensity(const VectorDynSize & jetsIntensity);
};

}

#endif
