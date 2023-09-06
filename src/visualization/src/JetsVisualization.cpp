// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include "JetsVisualization.h"
#include "ModelVisualization.h"

#include "IrrlichtUtils.h"

#include <iDynTree/Model.h>

#include <cassert>

namespace iDynTree
{

JetsVisualization::JetsVisualization(): m_smgr(0),
                                        m_modelViz(0),
                                        m_framesNodes(0),
                                        m_jetsFrameIndices(0),
                                        m_jetsNodes(0),
                                        m_jetsDirections(0),
                                        m_jetsColors(0),
                                        m_maxRadius(0.2),
                                        m_minRadius(0.1),
                                        m_maxLength(0.4)
{

}

void JetsVisualization::init(irr::scene::ISceneManager* smgr, ModelVisualization *modelViz, std::vector<irr::scene::ISceneNode *> * framesNodes)
{
    m_smgr = smgr;
    assert(m_modelViz == 0);
    m_modelViz = modelViz;
    m_framesNodes = framesNodes;
}

void JetsVisualization::reset()
{
    m_jetsFrameIndices.resize(0);
    m_jetsNodes.resize(0);
    m_jetsDirections.resize(0);
    m_jetsColors.resize(0);
}

JetsVisualization::JetsVisualization(const JetsVisualization& /*other*/)
{
    assert(false);
}


JetsVisualization::~JetsVisualization()
{

}


JetsVisualization& JetsVisualization::operator=(const JetsVisualization& /*other*/)
{
    assert(false);
    return *this;
}

size_t JetsVisualization::getNrOfJets() const
{
    return m_jetsNodes.size();
}

Direction JetsVisualization::getJetDirection(const int jetIndex) const
{
    if( jetIndex < 0 || (size_t)jetIndex >= getNrOfJets() ) return iDynTree::Direction::Default();

    return m_jetsDirections[jetIndex];
}

bool JetsVisualization::setJetDirection(const int jetIndex, const Direction &jetDirection)
{
    if( jetIndex < 0 || (size_t)jetIndex >= getNrOfJets() ) return false;

    m_jetsDirections[jetIndex] = jetDirection;

    return true;
}

bool JetsVisualization::setJetColor(const int jetIndex, const ColorViz &jetColor)
{
    if( jetIndex < 0 || (size_t)jetIndex >= getNrOfJets() ) return false;

    m_jetsColors[jetIndex] = jetColor;

    return true;
}


bool JetsVisualization::setJetsDimensions(const double &minRadius,
                                          const double &maxRadius,
                                          const double &maxLenght)
{
    m_maxLength = maxLenght;
    m_minRadius = minRadius;
    m_maxRadius = maxRadius;
    return true;
}

bool JetsVisualization::setJetsFrames(const std::vector <std::string> &jetsFrames)
{
    // Clean previous jets created
    reset();

    m_jetsFrameIndices.resize(jetsFrames.size(),FRAME_INVALID_INDEX);
    m_jetsNodes.resize(jetsFrames.size(),0);
    m_jetsDirections.resize(jetsFrames.size(),Direction(0.0,0.0,1.0));
    m_jetsColors.resize(jetsFrames.size(),ColorViz(1.0,1.0,1.0,0.0));

    for(size_t i=0; i < jetsFrames.size(); i++)
    {
        FrameIndex jetFrameIdx = m_modelViz->model().getFrameIndex(jetsFrames[i]);
        if( jetFrameIdx == FRAME_INVALID_INDEX )
        {
            std::stringstream ss;
            ss << "Impossible to find frame " << jetsFrames[i] << " in the model.";
            reportError("JetsVisualization","setJetsFrames",ss.str().c_str());
            reset();
            return false;
        }

        m_jetsFrameIndices[i] = jetFrameIdx;
    }

    // Set the initial jets values to 0
    iDynTree::VectorDynSize zeroIntensities;
    zeroIntensities.resize(jetsFrames.size());

    zeroIntensities.zero();

    setJetsIntensity(zeroIntensities);

    return true;
}

void JetsVisualization::drawJet(size_t i, double intensity)
{
    // Delete existing node
    if( this->m_jetsNodes[i] )
    {
        this->m_jetsNodes[i]->remove();
        this->m_jetsNodes[i] = 0;
    }

    // Create new node
    irr::scene::IMesh* coneMesh = createFrustumMesh(intensity*m_maxRadius,intensity*m_minRadius,intensity*m_maxLength);
    this->m_jetsNodes[i] = m_smgr->addMeshSceneNode(coneMesh,(*(this->m_framesNodes))[this->m_jetsFrameIndices[i]]);
    coneMesh->drop();
    coneMesh = 0;

    // Account for direction
    this->m_jetsNodes[i]->setRotation(idyntree2irr_rot(RotationWithPrescribedZColumn(this->m_jetsDirections[i])));

    // Set wireframe in the new node
    for( size_t mat = 0; mat < this->m_jetsNodes[i]->getMaterialCount(); mat++)
    {
        this->m_jetsNodes[i]->getMaterial(mat) = idyntree2irr(this->m_jetsColors[i]);
        this->m_jetsNodes[i]->getMaterial(mat).setFlag(irr::video::EMF_WIREFRAME,true);
    }
}

bool JetsVisualization::setJetsIntensity(const VectorDynSize &jetsIntensity)
{
    if( jetsIntensity.size() != getNrOfJets() ) return false;

    // Inefficient implementation: create a new mesh whenever the intensity are updated
    for(size_t i=0; i < getNrOfJets(); i++)
    {
        drawJet(i,jetsIntensity(i));
    }

    return true;
}


}
