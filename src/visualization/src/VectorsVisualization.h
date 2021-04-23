/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */
#ifndef IDYNTREE_VECTORS_VISUALIZATION_H
#define IDYNTREE_VECTORS_VISUALIZATION_H

#include <iDynTree/Visualizer.h>

#include <vector>
#include <irrlicht.h>

namespace iDynTree {

    class VectorsVisualization : public IVectorsVisualization {

        typedef struct {
            Position origin;
            Direction direction;
            double modulus;
            ColorViz color = ColorViz(1.0, 0.0, 0.0, 1.0);
            irr::scene::ISceneNode * visualizationNode = nullptr;
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

    };

}

#endif // IDYNTREE_VECTORS_VISUALIZATION_H
