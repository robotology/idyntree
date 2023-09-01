// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
#ifndef IDYNTREE_VECTORS_VISUALIZATION_H
#define IDYNTREE_VECTORS_VISUALIZATION_H

#include <iDynTree/Visualizer.h>
#include "Label.h"

#include <vector>
#include <irrlicht.h>

namespace iDynTree {

    class VectorsVisualization : public IVectorsVisualization {

        typedef struct {
            Position origin;
            Direction direction;
            double modulus;
            ColorViz color = ColorViz(1.0, 0.0, 0.0, 1.0);
            Label label;
            irr::scene::IMeshSceneNode * visualizationNode = nullptr;
        } VectorsProperties;

        ColorViz m_vectorsDefaultColor{ColorViz(1.0, 0.0, 0.0, 1.0)};

        std::vector<VectorsProperties> m_vectors;

        irr::scene::ISceneManager* m_smgr;

        double m_radiusOffset, m_radiusMultiplier, m_heightScale;

        void drawVector(size_t vectorIndex);

        void drawAll();

    public:

        VectorsVisualization();

        VectorsVisualization(const VectorsVisualization& other) = delete;

        VectorsVisualization& operator=(const VectorsVisualization& other) = delete;

        void init(irr::scene::ISceneManager* smgr);

        void close();

        virtual ~VectorsVisualization() override;

        virtual size_t addVector(const Position & origin, const Direction & direction, double modulus) override;

        virtual size_t addVector(const Position & origin, const Vector3 & components) override;

        virtual size_t getNrOfVectors() const override;

        virtual bool getVector(size_t vectorIndex, Position & currentOrigin,
                               Direction & currentDirection, double & currentModulus) const override;

        virtual bool getVector(size_t vectorIndex, Position & currentOrigin, Vector3 & components) const override;

        virtual bool updateVector(size_t vectorIndex, const Position & origin, const Direction & direction, double modulus) override;

        virtual bool updateVector(size_t vectorIndex, const Position & origin, const Vector3& components) override;

        virtual bool setVectorColor(size_t vectorIndex, const ColorViz & vectorColor) override;

        virtual void setVectorsDefaultColor(const ColorViz &vectorColor) override;

        virtual void setVectorsColor(const ColorViz &vectorColor) override;

        virtual bool setVectorsAspect(double zeroModulusRadius, double modulusMultiplier, double heightScale) override;

        virtual bool setVisible(size_t vectorIndex, bool visible) override;

        virtual ILabel* getVectorLabel(size_t vectorIndex) override;

    };

}

#endif // IDYNTREE_VECTORS_VISUALIZATION_H
