// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OCSolvers/MultipleShootingSolver.h>

#include <iDynTree/SparsityStructure.h>
#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Integrator.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/OptimizationProblem.h>

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/Utils.h>

#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>

#include <cassert>
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>
#include <map>

namespace iDynTree {
    namespace optimalcontrol
    {

        enum class MeshPointType{
            Control,
            State
        };

        class MeshPointOrigin{
            std::string m_name;
            std::string m_description;
            int m_priority;
        public:
            MeshPointOrigin()
            :m_name("ERROR")
            ,m_description("A meshPointOrigin not initialized")
            ,m_priority(-1)
            {}

            MeshPointOrigin(const std::string& name, int priority, const std::string& description) : m_name(name), m_description(description), m_priority(priority){}
            static MeshPointOrigin FirstPoint() {return MeshPointOrigin("FirstPoint", 9, "The first point");}
            static MeshPointOrigin LastPoint() {return MeshPointOrigin("LastPoint", 8, "The last point");}
            static MeshPointOrigin Control() {return MeshPointOrigin("Control", 7, "A control mesh");}
            static MeshPointOrigin CompensateLongPeriod() {return MeshPointOrigin("CompensateLongPeriod", 6, "A mesh filling a long period");}
            static MeshPointOrigin UserControl() {return MeshPointOrigin("UserControl", 5, "An user defined control mesh");}
            static MeshPointOrigin UserState() {return MeshPointOrigin("UserState", 4, "An user defined state mesh");}
            static MeshPointOrigin Instant() {return MeshPointOrigin("Instant", 3, "A mesh corresponding to a single instant cost/constraint");}
            static MeshPointOrigin FillVariables() {return MeshPointOrigin("FillVariables", 2, "A mesh added to have a constant number of varialbes");}
            static MeshPointOrigin TimeRange() {return MeshPointOrigin("TimeRange", 1, "A mesh corresponding to the begin/end of a cost/constraint");}
            static MeshPointOrigin Ignored() {return MeshPointOrigin("Ignored", 0, "An ignored mesh");}

            int priority() const {return m_priority;}
            std::string name() const {return m_name;}
            std::string description() const {return m_description;}

            bool operator<(const MeshPointOrigin &rhs) const {return m_priority < rhs.priority();}
            bool operator==(const MeshPointOrigin &rhs) const {return m_priority == rhs.priority();}
            bool operator!=(const MeshPointOrigin &rhs) const {return m_priority != rhs.priority();}
        };

        typedef  struct{
            double time;
            MeshPointType type;
            MeshPointOrigin origin;
            size_t controlIndex, previousControlIndex, stateIndex;
            //std::vector<size_t> integratorAuxiliariesOffsets;
        } MeshPoint;

        //MARK: Transcription implementation
        class HessianBlocksMap {
            std::map<CollocationHessianIndex, bool> m_map;
            std::map<CollocationHessianIndex, SparsityStructure> m_sparsity;

        public:

            HessianBlocksMap () { }

            bool& operator()(size_t row, size_t col) {
                CollocationHessianIndex indices(row, col);
                std::map<CollocationHessianIndex, bool>::iterator it = m_map.find(indices);

                if (it == m_map.end()) {
                    std::pair<std::map<CollocationHessianIndex, bool>::iterator, bool> result = m_map.insert(std::make_pair(indices, false));
                    it = result.first;
                }
                return it->second;
            }

            SparsityStructure& blockSparsity(size_t row, size_t col) {
                CollocationHessianIndex indices(row, col);
                std::map<CollocationHessianIndex, SparsityStructure>::iterator it = m_sparsity.find(indices);

                if (it == m_sparsity.end()) {
                    std::pair<std::map<CollocationHessianIndex, SparsityStructure>::iterator, bool> result = m_sparsity.insert(std::make_pair(indices, SparsityStructure()));
                    it = result.first;
                }
                return it->second;
            }

            void getFullSparsity(SparsityStructure& sparsity) {
                sparsity.clear();

                for (auto& block : m_sparsity) {
                    sparsity.addBlock(block.first.first(), block.first.second(), block.second);
                }
            }

            void clear() {
                m_map.clear();
                m_sparsity.clear();
            }

            void reset() {
                for (auto& it : m_map) {
                    it.second = false;
                }
            }

        };


        class MultipleShootingSolver::MultipleShootingTranscription : public optimization::OptimizationProblem {

            std::shared_ptr<OptimalControlProblem> m_ocproblem;
            std::shared_ptr<Integrator> m_integrator;
            size_t m_totalMeshes, m_controlMeshes;
            bool m_prepared;
            std::vector<double> m_userStateMeshes, m_userControlMeshes;
            std::vector<MeshPoint> m_meshPoints;
            std::vector<MeshPoint>::iterator m_meshPointsEnd;
            double m_minStepSize, m_maxStepSize, m_controlPeriod;
            size_t m_nx, m_nu, m_numberOfVariables, m_constraintsPerInstant, m_numberOfConstraints;
            std::vector<size_t> m_jacobianNZRows, m_jacobianNZCols/*, m_hessianNZRows, m_hessianNZCols*/;
            SparsityStructure m_ocStateJacobianSparsity, m_ocControlJacobianSparsity;
            std::vector<SparsityStructure> m_collocationStateJacobianNZ, m_collocationControlJacobianNZ;
            SparsityStructure m_mergedCollocationControlJacobianNZ;
            size_t m_jacobianNonZeros/*, m_hessianNonZeros*/;
            double m_plusInfinity, m_minusInfinity;
            VectorDynSize m_constraintsLowerBound, m_constraintsUpperBound;
            VectorDynSize m_constraintsBuffer, m_stateBuffer, m_controlBuffer, m_variablesBuffer, m_guessBuffer, m_costStateGradientBuffer, m_costControlGradientBuffer;
            MatrixDynSize m_costHessianStateBuffer, m_costHessianControlBuffer, m_costHessianStateControlBuffer, m_costHessianControlStateBuffer;
            std::vector<VectorDynSize> m_collocationStateBuffer, m_collocationControlBuffer;
            std::vector<MatrixDynSize> m_collocationStateJacBuffer, m_collocationControlJacBuffer;
            MatrixDynSize m_constraintsStateJacBuffer, m_constraintsControlJacBuffer;
            VectorDynSize m_lambdaConstraints, m_lambdaCollocation;
            MatrixDynSize m_stateHessianBuffer, m_controlHessianBuffer, m_stateControlHessianBuffer;
            CollocationHessianMap m_stateCollocationHessians, m_controlCollocationHessians, m_stateControCollocationlHessians;
            HessianBlocksMap m_hessianBlocks;
            VectorDynSize m_solution;
            std::shared_ptr<TimeVaryingVector> m_stateGuesses, m_controlGuesses;
            bool m_solved;

            SparsityStructure m_constraintsStateHessianSparsity, m_constraintsControlHessianSparsity, m_constraintsMixedHessianSparsity;
            SparsityStructure m_costsStateHessianSparsity, m_costsControlHessianSparsity, m_costsMixedHessianSparsity;
            CollocationHessianSparsityMap m_systemStateHessianSparsity, m_systemControlHessianSparsity, m_systemMixedHessianSparsity;
            SparsityStructure m_fullHessianSparsity;

            bool m_useCostRegularization, m_useConstraintsRegularization;
            double m_constraintsRegularization, m_costsRegularization;


            friend class MultipleShootingSolver;

            void resetMeshPoints(){
                m_meshPointsEnd = m_meshPoints.begin();
            }

            void addMeshPoint(MeshPoint& newMeshPoint){
                if (m_meshPointsEnd == m_meshPoints.end()){
                    m_meshPoints.push_back(newMeshPoint);
                    m_meshPointsEnd = m_meshPoints.end();
                } else {
                    *m_meshPointsEnd = newMeshPoint;
                    ++m_meshPointsEnd;
                }
            }

            std::vector<MeshPoint>::iterator findNextMeshPoint(const std::vector<MeshPoint>::iterator& start){
                std::vector<MeshPoint>::iterator nextMesh = start;
                MeshPointOrigin ignored = MeshPointOrigin::Ignored();
                assert(nextMesh != m_meshPoints.end());
                assert(nextMesh->origin.name().size() > 0);
                do {
                    ++nextMesh;
                    assert(nextMesh->origin.name().size() > 0);
                    assert(nextMesh != m_meshPoints.end());
                } while (nextMesh->origin == ignored); //find next valid mesh

                return nextMesh;
            }

            void setIgnoredMesh(std::vector<MeshPoint>::iterator& toBeIgnored){
                assert(toBeIgnored != m_meshPoints.end());
                toBeIgnored->time = m_ocproblem->finalTime() + m_maxStepSize;
                toBeIgnored->origin = MeshPointOrigin::Ignored();
            }

            void cleanLeftoverMeshes(){
                for (auto toBeIgnored = m_meshPointsEnd; toBeIgnored != m_meshPoints.end(); toBeIgnored++){
                    setIgnoredMesh(toBeIgnored);
                }
            }

            void priorityWarning(std::vector<MeshPoint>::iterator& meshToBeRemoved, MeshPointOrigin& noWarningLevel){
                if (noWarningLevel < meshToBeRemoved->origin){ //check if priority was high
                    std::ostringstream errorMsg;
                    errorMsg << meshToBeRemoved->origin.description() << " had to be removed due to limits on the minimum step size. "
                             << "Its time was " << meshToBeRemoved->time;
                    reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
                }
            }

            void resetNonZerosCount(){
                m_jacobianNonZeros = 0;
                m_hessianBlocks.clear();
            }

            void addNonZero(std::vector<size_t>& input, size_t position, size_t toBeAdded){
                assert(input.size() >= position);
                if (position >= input.size()){
                    input.push_back(toBeAdded);
                } else {
                    input[position] = toBeAdded;
                }
            }

            void addJacobianBlock(size_t initRow, size_t rows, size_t initCol, size_t cols){
                for (size_t i = 0; i < rows; ++i){
                    for (size_t j = 0; j < cols; ++j){
                        addNonZero(m_jacobianNZRows, m_jacobianNonZeros, initRow + i);
                        addNonZero(m_jacobianNZCols, m_jacobianNonZeros, initCol + j);
                        m_jacobianNonZeros++;
                    }
                }
            }

            void addJacobianBlock(size_t initRow, size_t initCol, const SparsityStructure& sparsity){
                for (size_t i = 0; i < sparsity.size(); ++i) {
                    addNonZero(m_jacobianNZRows, m_jacobianNonZeros, initRow + sparsity[i].row);
                    addNonZero(m_jacobianNZCols, m_jacobianNonZeros, initCol + sparsity[i].col);
                    m_jacobianNonZeros++;
                }
            }

            void addIdentityJacobianBlock(size_t initRow, size_t initCol, size_t dimension){
                for (size_t i = 0; i < dimension; ++i){
                    addNonZero(m_jacobianNZRows, m_jacobianNonZeros, initRow + i);
                    addNonZero(m_jacobianNZCols, m_jacobianNonZeros, initCol + i);
                    m_jacobianNonZeros++;
                }
            }

            void addHessianBlock(size_t initRow, size_t rows, size_t initCol, size_t cols){

                m_hessianBlocks.blockSparsity(initRow, initCol).addDenseBlock(0, 0, rows, cols);

                //add also the transpose

                m_hessianBlocks.blockSparsity(initCol, initRow).addDenseBlock(0, 0, cols, rows);

            }

            void addHessianBlock(size_t initRow, size_t initCol, const SparsityStructure& sparsity){

                m_hessianBlocks.blockSparsity(initRow, initCol).merge(sparsity);

                //add also the transpose

                SparsityStructure& transposeSparsity = m_hessianBlocks.blockSparsity(initCol, initRow);

                for (size_t i = 0; i < sparsity.size(); ++i){
                    transposeSparsity.add(sparsity[i].col, sparsity[i].row);
                }
            }

            void setHessianBlock(MatrixDynSize& hessian, const MatrixDynSize& block, size_t startRow, size_t startCol) {
                if (m_hessianBlocks(startRow, startCol)) {
                    toEigen(hessian).block(static_cast<Eigen::Index>(startRow), static_cast<unsigned int>(startCol), block.rows(), block.cols()) += toEigen(block);
                } else {
                    toEigen(hessian).block(static_cast<Eigen::Index>(startRow), static_cast<unsigned int>(startCol), block.rows(), block.cols()) = toEigen(block);
                    m_hessianBlocks(startRow, startCol) = true;
                }
            }

            void setHessianBlockAndItsTranspose(MatrixDynSize& hessian, const MatrixDynSize& block, size_t startRow, size_t startCol) {
                setHessianBlock(hessian, block, startRow, startCol);

                if (m_hessianBlocks(startCol, startRow)) {
                    toEigen(hessian).block(static_cast<Eigen::Index>(startCol), static_cast<unsigned int>(startRow), block.cols(), block.rows()) += toEigen(block).transpose();
                } else {
                    toEigen(hessian).block(static_cast<Eigen::Index>(startCol), static_cast<unsigned int>(startRow), block.cols(), block.rows()) = toEigen(block).transpose();
                    m_hessianBlocks(startCol, startRow) = true;
                }
            }

            void addHessianSparsityBlock(bool isSparse, size_t row, size_t numberOfRows, size_t col, size_t numberOfCols, const SparsityStructure& sparsity) {
                if (isSparse) {
                    this->addHessianBlock(row, col, sparsity);
                } else {
                    this->addHessianBlock(row, numberOfRows, col, numberOfCols);
                }
            }

            void mergeSparsityVectors(const std::vector<SparsityStructure>& original, SparsityStructure& merged) {
                merged.clear();
                merged = original[0];
                merged.merge(original[1]);
            }

            void allocateBuffers(){
                if (m_stateBuffer.size() != m_nx) {
                    m_stateBuffer.resize(static_cast<unsigned int>(m_nx));
                }

                if (m_costStateGradientBuffer.size() != m_nx) {
                    m_costStateGradientBuffer.resize(static_cast<unsigned int>(m_nx));
                }

                if ((m_costHessianStateBuffer.rows() != m_nx) || (m_costHessianStateBuffer.cols() != m_nx)) {
                    m_costHessianStateBuffer.resize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nx));
                }

                if (m_controlBuffer.size() != m_nu) {
                    m_controlBuffer.resize(static_cast<unsigned int>(m_nu));
                }

                if (m_costControlGradientBuffer.size() != m_nu) {
                    m_costControlGradientBuffer.resize(static_cast<unsigned int>(m_nu));
                }

                if ((m_costHessianControlBuffer.rows() != m_nu) || (m_costHessianControlBuffer.cols() != m_nu)) {
                    m_costHessianControlBuffer.resize(static_cast<unsigned int>(m_nu), static_cast<unsigned int>(m_nu));
                }

                if ((m_costHessianStateControlBuffer.rows() != m_nx) || (m_costHessianStateControlBuffer.cols() != m_nu)) {
                    m_costHessianStateControlBuffer.resize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));
                }

                if ((m_costHessianControlStateBuffer.rows() != m_nu) || (m_costHessianControlStateBuffer.cols() != m_nx)) {
                    m_costHessianControlStateBuffer.resize(static_cast<unsigned int>(m_nu), static_cast<unsigned int>(m_nx));
                }


                //TODO: I should consider also the possibility to have auxiliary variables in the integrator
                if (m_variablesBuffer.size() != m_numberOfVariables) {
                    m_variablesBuffer.resize(static_cast<unsigned int>(m_numberOfVariables));
                }

                m_guessBuffer.resize(static_cast<unsigned int>(m_numberOfVariables));

                if (m_constraintsBuffer.size() != m_constraintsPerInstant) {
                    m_constraintsBuffer.resize(static_cast<unsigned int>(m_constraintsPerInstant));
                }

                if (m_constraintsLowerBound.size() != m_numberOfConstraints) {
                    m_constraintsLowerBound.resize(static_cast<unsigned int>(m_numberOfConstraints));
                }

                if (m_constraintsUpperBound.size() != m_numberOfConstraints) {
                    m_constraintsUpperBound.resize(static_cast<unsigned int>(m_numberOfConstraints));
                }

                //TODO: I should consider also the possibility to have auxiliary variables in the integrator
                if (m_collocationStateBuffer.size() != 2) {
                    m_collocationStateBuffer.resize(2);
                }
                for (size_t i = 0; i < 2; ++i) {
                    if (m_collocationStateBuffer[i].size() != m_nx) {
                        m_collocationStateBuffer[i].resize(static_cast<unsigned int>(m_nx));
                    }
                }

                if (m_collocationControlBuffer.size() != 2) {
                    m_collocationControlBuffer.resize(2);
                }
                for (size_t i = 0; i < 2; ++i) {
                    if (m_collocationControlBuffer[i].size() != m_nu) {
                        m_collocationControlBuffer[i].resize(static_cast<unsigned int>(m_nu));
                    }
                }

                if (m_collocationStateJacBuffer.size() != 2) {
                    m_collocationStateJacBuffer.resize(2);
                }
                for (size_t i = 0; i < 2; ++i) {
                    if ((m_collocationStateJacBuffer[i].rows() != m_nx) || (m_collocationStateJacBuffer[i].cols() != m_nx)) {
                        m_collocationStateJacBuffer[i].resize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nx));
                    }
                }

                if (m_collocationControlJacBuffer.size() != 2) {
                    m_collocationControlJacBuffer.resize(2);
                }
                for (size_t i = 0; i < 2; ++i) {
                    if ((m_collocationControlJacBuffer[i].rows() != m_nx) || (m_collocationControlJacBuffer[i].cols() != m_nu)) {
                        m_collocationControlJacBuffer[i].resize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));
                    }
                }

                if ((m_constraintsStateJacBuffer.rows() != m_constraintsPerInstant) || (m_constraintsStateJacBuffer.cols() != m_nx)) {
                    m_constraintsStateJacBuffer.resize(static_cast<unsigned int>(m_constraintsPerInstant), static_cast<unsigned int>(m_nx));
                }

                if ((m_constraintsControlJacBuffer.rows() != m_constraintsPerInstant) || (m_constraintsControlJacBuffer.cols() != m_nu)) {
                    m_constraintsStateJacBuffer.resize(static_cast<unsigned int>(m_constraintsPerInstant), static_cast<unsigned int>(m_nu));
                }

                m_lambdaCollocation.resize(static_cast<unsigned int>(m_nx));
                m_lambdaConstraints.resize(static_cast<unsigned int>(m_constraintsPerInstant));
                m_stateHessianBuffer.resize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nx));
                m_controlHessianBuffer.resize(static_cast<unsigned int>(m_nu), static_cast<unsigned int>(m_nu));
                m_stateControlHessianBuffer.resize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));
                m_stateCollocationHessians[CollocationHessianIndex(0,0)] = m_stateHessianBuffer;
                m_stateCollocationHessians[CollocationHessianIndex(0,1)] = m_stateHessianBuffer;
                m_stateCollocationHessians[CollocationHessianIndex(1,1)] = m_stateHessianBuffer;
                m_controlCollocationHessians[CollocationHessianIndex(0,0)] = m_controlHessianBuffer;
                m_controlCollocationHessians[CollocationHessianIndex(0,1)] = m_controlHessianBuffer;
                m_controlCollocationHessians[CollocationHessianIndex(1,1)] = m_controlHessianBuffer;
                m_stateControCollocationlHessians[CollocationHessianIndex(0,0)] = MatrixDynSize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));
                m_stateControCollocationlHessians[CollocationHessianIndex(0,1)] = MatrixDynSize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));
                m_stateControCollocationlHessians[CollocationHessianIndex(1,0)] = MatrixDynSize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));
                m_stateControCollocationlHessians[CollocationHessianIndex(1,1)] = MatrixDynSize(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_nu));

            }


            size_t setControlMeshPoints() {
                double endTime = m_ocproblem->finalTime(), initTime = m_ocproblem->initialTime();
                size_t controlMeshes = 0;
                double time = initTime;
                MeshPoint newMeshPoint;

                newMeshPoint.type = MeshPointType::Control;

                while (time <= endTime){
                    if (checkDoublesAreEqual(time,initTime))//(time == initTime)
                        newMeshPoint.origin = MeshPointOrigin::FirstPoint();
                    else if (checkDoublesAreEqual(time,endTime))//(time == endTime)
                        newMeshPoint.origin = MeshPointOrigin::LastPoint();
                    else
                        newMeshPoint.origin = MeshPointOrigin::Control();
                    newMeshPoint.time = time;

                    addMeshPoint(newMeshPoint);
                    controlMeshes++;

                    time = initTime + controlMeshes * m_controlPeriod;
                }

                std::vector<MeshPoint>::iterator lastControlMeshIterator = (m_meshPointsEnd)-1;

                if ((lastControlMeshIterator->origin == MeshPointOrigin::LastPoint()) && (m_integrator->info().isExplicit())) {//the last control input would have no effect
                    controlMeshes--;
                    setIgnoredMesh(lastControlMeshIterator);
                    m_meshPointsEnd = lastControlMeshIterator;
                    --lastControlMeshIterator;
                }

                newMeshPoint.origin = MeshPointOrigin::LastPoint();
                newMeshPoint.type = MeshPointType::State;
                newMeshPoint.time = endTime;

                if (lastControlMeshIterator->time < endTime){
                    if ((lastControlMeshIterator->time + m_minStepSize) > endTime){ //if last control mesh point is too close to the end, remove it and place a state mesh point at the end
                        setIgnoredMesh(lastControlMeshIterator);
                        controlMeshes--;
                        m_meshPointsEnd = lastControlMeshIterator;
                    }
                    addMeshPoint(newMeshPoint);  //add a mesh point at the end;
                }
                return controlMeshes;
            }

            bool preliminaryChecks() {
                if (!(m_ocproblem)){
                    reportError("MultipleShootingTranscription", "prepare",
                                "Optimal control problem not set.");
                    return false;
                }

                if (!(m_integrator)) {
                    reportError("MultipleShootingTranscription", "prepare",
                                "Integrator not set.");
                    return false;
                }

                if ((m_ocproblem->finalTime() - m_ocproblem->initialTime()) < m_minStepSize){
                    reportError("MultipleShootingTranscription", "prepare",
                                "The time horizon defined in the OptimalControlProblem is smaller than the minimum step size.");
                    return false;
                }

                if (m_controlPeriod < m_minStepSize){
                    reportError("MultipleShootingTranscription", "prepare", "The control period cannot be lower than the minimum step size.");
                    return false;
                }

                if ((m_controlPeriod >= m_maxStepSize) && (m_controlPeriod <= (2 * m_minStepSize))){
                    reportError("MultipleShootingTranscription", "prepare",
                                "Cannot add control mesh points when the controller period is in the range [dTMax, 2*dTmin]."); // the first mesh point is a control mesh. The second mesh than would be too far from the first but not far enough to put a state mesh point in the middle.
                    return false;
                }

                if (!(m_prepared)){
                    if (m_ocproblem->dynamicalSystem().expired()){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "No dynamical system set to the OptimalControlProblem.");
                        return false;
                    } else {
                        if (!(m_integrator->dynamicalSystem().expired()) &&
                                (m_integrator->dynamicalSystem().lock() != m_ocproblem->dynamicalSystem().lock())){
                            reportError("MultipleShootingTranscription", "prepare",
                                        "The selected OptimalControlProblem and the Integrator point to two different dynamical systems.");
                            return false;
                        } else if (m_integrator->dynamicalSystem().expired()) {
                            if (!(m_integrator->setDynamicalSystem(m_ocproblem->dynamicalSystem().lock()))){
                                reportError("MultipleShootingTranscription", "prepare",
                                            "Error while setting the dynamicalSystem to the Integrator using the one pointer provided by the OptimalControlProblem object.");
                                return false;
                            }
                        }
                    }
                }

                if (!(m_integrator->setMaximumStepSize(m_maxStepSize))){
                    reportError("MultipleShootingTranscription", "prepare","Error while setting the maximum step size to the integrator.");
                    return false;
                }

                return true;
            }

            bool setMeshPoints() {
                double endTime = m_ocproblem->finalTime(), initTime = m_ocproblem->initialTime();
                resetMeshPoints();

                if (m_controlPeriod < 2*(m_minStepSize)){ //no other mesh point could be inserted otherwise
                    size_t newControlMeshes = setControlMeshPoints();
                    size_t newTotalMeshes = static_cast<size_t>(m_meshPointsEnd - m_meshPoints.begin());

                    if (m_prepared){
                        if ((m_totalMeshes != newTotalMeshes) || (m_controlMeshes != newControlMeshes)){
                            reportWarning("MultipleShootingSolver", "setMeshPoints", "Unable to keep number of variables constant.");
                        }
                    }

                    m_controlMeshes = newControlMeshes;
                    m_totalMeshes = newTotalMeshes;
                    return true;
                }

                setControlMeshPoints();

                MeshPoint newMeshPoint;
                //Adding user defined control mesh points
                newMeshPoint.origin = MeshPointOrigin::UserControl();
                newMeshPoint.type = MeshPointType::Control;
                for (size_t i = 0; i < m_userControlMeshes.size(); ++i){
                   if ((m_userControlMeshes[i] > initTime) && (m_userControlMeshes[i] < endTime)){
                        newMeshPoint.time = m_userControlMeshes[i];
                        addMeshPoint(newMeshPoint);
                   } else {
                       std::ostringstream errorMsg;
                       errorMsg << "Ignored user defined control mesh point at time " << m_userControlMeshes[i] << "Out of time range.";
                       reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
                   }
                }

                //Adding user defined state mesh points
                newMeshPoint.origin = MeshPointOrigin::UserState();
                newMeshPoint.type = MeshPointType::State;
                for (size_t i = 0; i < m_userStateMeshes.size(); ++i){
                    if ((m_userStateMeshes[i] > initTime) && (m_userStateMeshes[i] < endTime)){
                        newMeshPoint.time = m_userStateMeshes[i];
                        addMeshPoint(newMeshPoint);
                    } else {
                        std::ostringstream errorMsg;
                        errorMsg << "Ignored user defined state mesh point at time " << m_userStateMeshes[i] << "Out of time range.";
                        reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
                    }
                }

                std::vector<TimeRange> &constraintsTRs = m_ocproblem->getConstraintsTimeRanges();
                std::vector<TimeRange> &costsTRs = m_ocproblem->getCostsTimeRanges();

                //adding mesh points for the constraint time ranges
                newMeshPoint.type = MeshPointType::State;
                for (size_t i = 0; i < constraintsTRs.size(); ++i){
                    if (constraintsTRs[i].isInstant()) {
                        if (!(constraintsTRs[i].isInRange(initTime)) && !(constraintsTRs[i].isInRange(endTime)) && (constraintsTRs[i].initTime() > initTime)){ //we will have a mesh there in any case
                            newMeshPoint.origin = MeshPointOrigin::Instant();
                            newMeshPoint.time = constraintsTRs[i].initTime();
                            addMeshPoint(newMeshPoint);
                        }
                    }
                    else {
                        newMeshPoint.origin = MeshPointOrigin::TimeRange();
                        if ((constraintsTRs[i].initTime() > initTime) && (constraintsTRs[i].initTime() < endTime)){
                            newMeshPoint.time = constraintsTRs[i].initTime();
                            addMeshPoint(newMeshPoint);
                        }
                        if ((constraintsTRs[i].endTime() > initTime) && (constraintsTRs[i].endTime() < endTime)){
                            newMeshPoint.time = constraintsTRs[i].endTime();
                            addMeshPoint(newMeshPoint);
                        }
                    }
                }


                //adding mesh points for the costs time ranges
                for (size_t i = 0; i < costsTRs.size(); ++i){
                    if (costsTRs[i].isInstant()){
                        if (!(costsTRs[i].isInRange(initTime)) && !(costsTRs[i].isInRange(endTime)) && (costsTRs[i].initTime() > initTime)) { //we will have a mesh there in any case
                            newMeshPoint.origin = MeshPointOrigin::Instant();
                            newMeshPoint.time = costsTRs[i].initTime();
                            addMeshPoint(newMeshPoint);
                        }
                    }
                    else {
                        newMeshPoint.origin = MeshPointOrigin::TimeRange();
                        if ((costsTRs[i].initTime() > initTime) && (costsTRs[i].initTime() < endTime)){
                            newMeshPoint.time = costsTRs[i].initTime();
                            addMeshPoint(newMeshPoint);
                        }
                        if ((costsTRs[i].endTime() > initTime) && (costsTRs[i].endTime() < endTime)){
                            newMeshPoint.time = costsTRs[i].endTime();
                            addMeshPoint(newMeshPoint);
                        }
                    }
                }

                cleanLeftoverMeshes(); //set to ignored the leftover meshes, i.e. those that were set in a previous call

                double tMin = m_minStepSize;
                std::sort(m_meshPoints.begin(), m_meshPointsEnd,
                          [tMin](const MeshPoint&a, const MeshPoint&b) {
                                if (std::abs(b.time - a.time) < tMin)
                                    return a.origin.priority() < b.origin.priority();
                                else return a.time < b.time;}); //reorder the vector. Using this ordering, by scrolling the vector, in case of two narrow points, I will get the more important first

                //Now I need to prune the vector

                std::vector<MeshPoint>::iterator mesh = m_meshPoints.begin();
                std::vector<MeshPoint>::iterator nextMesh = mesh;
                MeshPointOrigin last = MeshPointOrigin::LastPoint();
                MeshPointOrigin ignored = MeshPointOrigin::Ignored();
                MeshPointOrigin timeRangeOrigin = MeshPointOrigin::TimeRange();

                newMeshPoint.origin = MeshPointOrigin::CompensateLongPeriod();
                newMeshPoint.type = MeshPointType::State;

                double timeDistance;
                while (mesh->origin != last){

                    nextMesh = findNextMeshPoint(mesh); //find next valid mesh
                    timeDistance = std::abs(nextMesh->time - mesh->time); //for the way I have ordered the vector, it can be negative
                    assert(timeDistance < (endTime - initTime));

                    if (timeDistance > m_maxStepSize){ //two consecutive points are too distant
                        unsigned int additionalPoints = static_cast<unsigned int>(std::ceil(timeDistance/(m_maxStepSize))) - 1;
                        double dtAdd = timeDistance / (additionalPoints + 1); //additionalPoints + 1 is the number of segments.
                        long nextPosition = nextMesh - m_meshPoints.begin();
                        //since tmin < tmax/2, dtAdd > tmin. Infact, the worst case is when timeDistance is nearly equal to tmax.
                        double startTime = std::min(mesh->time, nextMesh->time);
                        for (unsigned int i = 0; i < additionalPoints; ++i) {
                            newMeshPoint.time = startTime + (i + 1)*dtAdd;
                            assert((newMeshPoint.time >= initTime) && (newMeshPoint.time <= endTime));
                            addMeshPoint(newMeshPoint);
                        }
                        nextMesh = m_meshPoints.begin() + nextPosition;
                        mesh = nextMesh;

                    } else if (timeDistance < m_minStepSize){ //too consecutive points are too close

                        if (nextMesh->origin < mesh->origin){ //check who has the higher priority
                            priorityWarning(nextMesh, timeRangeOrigin);
                            setIgnoredMesh(nextMesh);
                        } else { //actually, because of the ordering, this case should never be reached.
                            priorityWarning(mesh, timeRangeOrigin);
                            setIgnoredMesh(mesh);
                            do {
                                --mesh;
                            } while (mesh->origin == ignored);
                            nextMesh = mesh;
                        }
                    } else {
                        mesh = nextMesh;
                    }
                }

                std::sort(m_meshPoints.begin(), m_meshPointsEnd,
                          [](const MeshPoint&a, const MeshPoint&b) {return a.time < b.time;}); //reorder the vector

                size_t newTotalMeshes = 0, newControlMeshes = 0;
                mesh = m_meshPoints.begin();

                while (mesh->origin != last){ //count meshes
                    if (mesh->type == MeshPointType::Control){
                        newControlMeshes++;
                    }
                    newTotalMeshes++;
                    ++mesh;
                }
                if (mesh->type == MeshPointType::Control){
                    newControlMeshes++;
                }
                newTotalMeshes++; //the last mesh
                m_meshPointsEnd = m_meshPoints.begin() + static_cast<long>(newTotalMeshes);
                assert((m_meshPointsEnd - 1)->origin == last);

                if (m_prepared){
                    if (m_controlMeshes == newControlMeshes){

                        if (m_totalMeshes != newTotalMeshes){

                            if (m_totalMeshes < newTotalMeshes){ //here I need to remove meshes

                                size_t toBeRemoved = newTotalMeshes - m_totalMeshes;
                                mesh = m_meshPoints.begin();
                                nextMesh = mesh;
                                std::vector<MeshPoint>::iterator nextNextMesh = mesh;
                                while ((mesh->origin != last) && (toBeRemoved > 0)){
                                    nextMesh = findNextMeshPoint(mesh); //find next valid mesh

                                    if (nextMesh->origin != last) {
                                        nextNextMesh = findNextMeshPoint(nextMesh);
                                        timeDistance = std::abs(nextNextMesh->time - mesh->time);

                                        if ((timeDistance < m_maxStepSize) && !(timeRangeOrigin < nextMesh->origin)){ //I can remove nextMesh
                                            setIgnoredMesh(nextMesh);
                                            toBeRemoved--;
                                            newTotalMeshes--;
                                        } else {
                                            mesh = nextMesh;
                                        }
                                    } else {
                                        mesh = nextMesh;
                                    }
                                }
                                if (toBeRemoved > 0){
                                    reportWarning("MultipleShootingSolver", "setMeshPoints",
                                                  "Unable to keep a constant number of variables. Unable to remove enough mesh points.");
                                }

                            } else if (m_totalMeshes > newTotalMeshes){ //here I need to add meshes
                                unsigned int toBeAdded = static_cast<unsigned int>(m_totalMeshes - newTotalMeshes);
                                mesh = m_meshPoints.begin();
                                nextMesh = mesh;
                                newMeshPoint.origin = MeshPointOrigin::FillVariables();
                                newMeshPoint.type = MeshPointType::State;
                                while ((mesh->origin != last) && (toBeAdded > 0)){
                                    nextMesh = findNextMeshPoint(mesh); //find next valid mesh
                                    timeDistance = std::abs(nextMesh->time - mesh->time);

                                    if (timeDistance > (2*m_minStepSize)){
                                        unsigned int possibleMeshes = static_cast<unsigned int>(std::ceil(timeDistance/(m_minStepSize))) - 1;
                                        unsigned int meshToAddHere = std::min(toBeAdded, possibleMeshes);
                                        double dT = timeDistance/(meshToAddHere + 1);
                                        long nextPosition = nextMesh - m_meshPoints.begin();
                                        double startTime = std::min(mesh->time, nextMesh->time);
                                        for(unsigned int m = 1; m <= meshToAddHere; m++){
                                            newMeshPoint.time = startTime + m*dT;
                                            addMeshPoint(newMeshPoint);
                                            newTotalMeshes++;
                                            toBeAdded--;
                                        }
                                        nextMesh = m_meshPoints.begin() + nextPosition;
                                    }
                                    mesh = nextMesh;
                                }

                                if (toBeAdded > 0){
                                    reportWarning("MultipleShootingSolver", "setMeshPoints",
                                                  "Unable to keep a constant number of variables. Unable to add enough mesh points.");
                                }

                            }
                            std::sort(m_meshPoints.begin(), m_meshPointsEnd,
                                      [](const MeshPoint&a, const MeshPoint&b) {return a.time < b.time;}); //reorder the vector
                            m_meshPointsEnd = m_meshPoints.begin() + static_cast<long>(newTotalMeshes);
                            assert((m_meshPointsEnd - 1)->origin == last);
                        }
                    } else {
                        reportWarning("MultipleShootingSolver", "setMeshPoints",
                                      "Unable to keep a constant number of variables. The control meshes are different");
                    }
                }

                m_totalMeshes = newTotalMeshes;
                m_controlMeshes = newControlMeshes;

                return true;
            }

            bool setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem) {
                if (!problem) {
                    reportError("MultipleShootingSolver", "setOptimalControlProblem", "Empty pointer.");
                    return false;
                }

                if (m_ocproblem){
                    reportError("MultipleShootingSolver", "setOptimalControlProblem", "The OptimalControlProblem for this instance has already been set.");
                    return false;
                }

                m_ocproblem = problem;
                return true;
            }

            bool setIntegrator(const std::shared_ptr<Integrator> integrationMethod) {
                if (!(integrationMethod)) {
                    reportError("MultipleShootingSolver", "setIntegrator", "Empty pointer.");
                    return false;
                }

                if (m_integrator){
                    reportError("MultipleShootingSolver", "setIntegrator", "The integration method for this instance has already been set.");
                    return false;
                }

                m_integrator = integrationMethod;
                return true;
            }

            bool setStepSizeBounds(const double minStepSize, const double maxStepsize) {
                if (minStepSize <= 0){
                    reportError("MultipleShootingSolver", "setStepSizeBounds","The minimum step size is expected to be positive.");
                    return false;
                }

                if (maxStepsize <= (2 * minStepSize)){
                    reportError("MultipleShootingSolver", "setStepSizeBounds","The maximum step size is expected to be greater than twice the minimum."); //imagine to have a distance between two mesh points slightly greater than the max. It would be impossible to add a mesh point in the middle
                    return false;
                }

                m_minStepSize = minStepSize;
                m_maxStepSize = maxStepsize;

                return true;
            }

            bool setControlPeriod(double period) {
                if (period <= 0){
                    reportError("MultipleShootingSolver", "prepare", "The control period is supposed to be positive.");
                    return false;
                }
                m_controlPeriod = period;
                return true;
            }

            bool setAdditionalStateMeshPoints(const std::vector<double>& stateMeshes) {
                m_userStateMeshes = stateMeshes;
                return true;
            }

            bool setAdditionalControlMeshPoints(const std::vector<double>& controlMeshes) {
                m_userControlMeshes = controlMeshes;
                return true;
            }

            void setPlusInfinity(double plusInfinity) {
                assert(plusInfinity > 0);
                m_plusInfinity = plusInfinity;
            }

            void setMinusInfinity(double minusInfinity) {
                assert(minusInfinity < 0);
                m_minusInfinity = minusInfinity;
            }

            bool setInitialState(const VectorDynSize &initialState) {
                if (!(m_ocproblem)){
                    reportError("MultipleShootingTranscription", "setInitialState", "The optimal control problem pointer is empty.");
                    return false;
                }

                if (m_ocproblem->dynamicalSystem().expired()){
                    reportError("MultipleShootingTranscription", "setInitialState", "The optimal control problem does not point to any dynamical system.");
                    return false;
                }

                if (!(m_ocproblem->dynamicalSystem().lock()->setInitialState(initialState))){
                    reportError("MultipleShootingTranscription", "setInitialState", "Error while setting the initial state to the dynamical system.");
                    return false;
                }

                return true;
            }

            bool getTimings(std::vector<double>& stateEvaluations, std::vector<double>& controlEvaluations) {
                if (!(m_prepared)){
                    reportWarning("MultipleShootingTranscription", "getTimings", "The method solve was not called yet. Use the method getPossibleTimings instead.");

                    return false;
                }

                if (stateEvaluations.size() != (m_totalMeshes - 1)) {
                    stateEvaluations.resize(m_totalMeshes - 1);
                }

                if (controlEvaluations.size() != m_controlMeshes) {
                    controlEvaluations.resize(m_controlMeshes);
                }

                size_t stateIndex = 0, controlIndex = 0;

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                MeshPointOrigin ignored = MeshPointOrigin::Ignored();

                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    assert(mesh->origin != ignored);

                    if (mesh->origin != first){
                        stateEvaluations[stateIndex] = mesh->time;
                        stateIndex++;
                    }
                    if (mesh->type == MeshPointType::Control){
                        controlEvaluations[controlIndex] = mesh->time;
                        controlIndex++;
                    }
                }

                return true;
            }

            bool getPossibleTimings(std::vector<double>& stateEvaluations, std::vector<double>& controlEvaluations) {

                if (!preliminaryChecks()) {
                    return false;
                }

                if (!setMeshPoints()){
                    return false;
                }

                if (stateEvaluations.size() != (m_totalMeshes - 1)) {
                    stateEvaluations.resize(m_totalMeshes - 1);
                }

                if (controlEvaluations.size() != m_controlMeshes) {
                    controlEvaluations.resize(m_controlMeshes);
                }

                size_t stateIndex = 0, controlIndex = 0;

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                MeshPointOrigin ignored = MeshPointOrigin::Ignored();

                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    assert(mesh->origin != ignored);

                    if (mesh->origin != first){
                        stateEvaluations[stateIndex] = mesh->time;
                        stateIndex++;
                    }
                    if (mesh->type == MeshPointType::Control){
                        controlEvaluations[controlIndex] = mesh->time;
                        controlIndex++;
                    }
                }

                return true;
            }

            bool getSolution(std::vector<VectorDynSize>& states, std::vector<VectorDynSize>& controls) {
                if (!(m_solved)){
                    reportError("MultipleShootingTranscription", "getSolution", "First you need to solve the problem once.");
                    return false;
                }

                if (states.size() != (m_totalMeshes - 1)) {
                    states.resize((m_totalMeshes - 1));
                }

                if (controls.size() != m_controlMeshes) {
                    controls.resize(m_controlMeshes);
                }

                Eigen::Map<Eigen::VectorXd> solutionMap = toEigen(m_solution);

                size_t stateIndex = 0, controlIndex = 0;

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();

                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){
                    if (mesh->origin != first){
                        if (states[stateIndex].size() !=  m_nx) {
                            states[stateIndex].resize(static_cast<unsigned int>(m_nx));
                        }

                        toEigen(states[stateIndex]) = solutionMap.segment(static_cast<unsigned int>(mesh->stateIndex), static_cast<unsigned int>(m_nx));
                        stateIndex++;
                    }
                    if (mesh->type == MeshPointType::Control){
                        if (controls[controlIndex].size() !=  m_nu) {
                            controls[controlIndex].resize(static_cast<unsigned int>(m_nu));
                        }

                        toEigen(controls[controlIndex]) = solutionMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), static_cast<Eigen::Index>(m_nu));
                        controlIndex++;
                    }
                }

                return true;
            }

        public:

            MultipleShootingTranscription()
            : m_ocproblem(nullptr)
            , m_integrator(nullptr)
            , m_totalMeshes(0)
            , m_controlMeshes(0)
            , m_prepared(false)
            , m_meshPointsEnd(m_meshPoints.end())
            , m_minStepSize(0.001)
            , m_maxStepSize(0.01)
            , m_controlPeriod(0.01)
            , m_nx(0)
            , m_nu(0)
            , m_numberOfVariables(0)
            , m_constraintsPerInstant(0)
            , m_numberOfConstraints(0)
            , m_plusInfinity(1e19)
            , m_minusInfinity(-1e19)
            , m_stateGuesses(nullptr)
            , m_controlGuesses(nullptr)
            , m_solved(false)
            , m_useCostRegularization(false)
            , m_useConstraintsRegularization(false)
            , m_constraintsRegularization(0.0)
            , m_costsRegularization(0.0)
            { }

            MultipleShootingTranscription(const std::shared_ptr<OptimalControlProblem> problem, const std::shared_ptr<Integrator> integrationMethod)
            : m_ocproblem(problem)
            , m_integrator(integrationMethod)
            , m_totalMeshes(0)
            , m_controlMeshes(0)
            , m_prepared(false)
            , m_meshPointsEnd(m_meshPoints.end())
            , m_minStepSize(0.001)
            , m_maxStepSize(0.01)
            , m_controlPeriod(0.01)
            , m_nx(0)
            , m_nu(0)
            , m_numberOfVariables(0)
            , m_constraintsPerInstant(0)
            , m_numberOfConstraints(0)
            , m_plusInfinity(1e19)
            , m_minusInfinity(-1e19)
            , m_stateGuesses(nullptr)
            , m_controlGuesses(nullptr)
            , m_solved(false)
            , m_useCostRegularization(false)
            , m_useConstraintsRegularization(false)
            , m_constraintsRegularization(0.0)
            , m_costsRegularization(0.0)
            { }

            MultipleShootingTranscription(const MultipleShootingTranscription& other) = delete;

            virtual ~MultipleShootingTranscription() override;

            virtual bool prepare() override {

                if (!preliminaryChecks()){
                    return false;
                }

                if (!setMeshPoints()){
                    return false;
                }

                size_t nx = m_integrator->dynamicalSystem().lock()->stateSpaceSize();
                m_nx = nx;

                size_t nu = m_integrator->dynamicalSystem().lock()->controlSpaceSize();
                m_nu = nu;

                //TODO: I should consider also the possibility to have auxiliary variables in the integrator
                m_numberOfVariables = (m_totalMeshes) * nx + m_controlMeshes * nu; //also the initial state should be an optimization variable. This allows to use only the constraint jacobian in case of a linear problem

                m_constraintsPerInstant = m_ocproblem->getConstraintsDimension();
                size_t nc = m_constraintsPerInstant;
                m_numberOfConstraints = (m_totalMeshes - 1) * nx + (m_constraintsPerInstant) * (m_totalMeshes); //dynamical constraints (plus identity constraint for the initial state) and normal constraints

                //Determine problem type
                m_infoData->hasLinearConstraints = (m_ocproblem->countLinearConstraints() != 0) || m_ocproblem->systemIsLinear();
                m_infoData->hasNonLinearConstraints = (m_ocproblem->countLinearConstraints() != m_constraintsPerInstant) || !(m_ocproblem->systemIsLinear());
                m_infoData->costIsLinear = m_ocproblem->hasOnlyLinearCosts();
                m_infoData->costIsQuadratic = m_ocproblem->hasOnlyQuadraticCosts();
                m_infoData->costIsNonLinear = !(m_ocproblem->hasOnlyLinearCosts()) && !(m_ocproblem->hasOnlyQuadraticCosts());
                m_infoData->hasSparseHessian = true;
                m_infoData->hasSparseConstraintJacobian = true;
                m_infoData->hessianIsProvided = true;

                allocateBuffers();

                Eigen::Map<Eigen::VectorXd> lowerBoundMap = toEigen(m_constraintsLowerBound);
                Eigen::Map<Eigen::VectorXd> upperBoundMap = toEigen(m_constraintsUpperBound);

                bool ocHasStateJacobianSparsisty = m_ocproblem->constraintsJacobianWRTStateSparsity(m_ocStateJacobianSparsity);
                if (!ocHasStateJacobianSparsisty) {
                    reportWarning("MultipleShootingTranscription", "prepare", "Failed to retrieve state sparsity of optimal control problem constraints jacobian. Assuming dense matrix.");
                }

                bool ocHasControlJacobianSparsisty = m_ocproblem->constraintsJacobianWRTControlSparsity(m_ocControlJacobianSparsity);
                if (!ocHasControlJacobianSparsisty) {
                    reportWarning("MultipleShootingTranscription", "prepare", "Failed to retrieve control sparsity of optimal control problem constraints jacobian. Assuming dense matrix.");
                }

                bool systemHasStateJacobianSparsity = m_integrator->getCollocationConstraintJacobianStateSparsity(m_collocationStateJacobianNZ);

                bool systemHasControlJacobianSparsity = m_integrator->getCollocationConstraintJacobianControlSparsity(m_collocationControlJacobianNZ);

                if (systemHasControlJacobianSparsity) {
                    mergeSparsityVectors(m_collocationControlJacobianNZ, m_mergedCollocationControlJacobianNZ);
                }

                auto printError = [](bool ok, const std::string& hessianName, const std::string& variableType) {
                    if (!ok) {
                        reportWarning("MultipleShootingTranscription", "prepare", ("Failed to retrieve " + variableType+ " sparsity of optimal control problem " +
                                                                                   hessianName + " hessian. Assuming dense matrix.").c_str());
                    }
                };

                bool constraintsHaveStateHessianSparsity = m_ocproblem->constraintsSecondPartialDerivativeWRTStateSparsity(m_constraintsStateHessianSparsity);
                printError(constraintsHaveStateHessianSparsity, "constraints", "state");

                bool constraintsHaveControlHessianSparsity = m_ocproblem->constraintsSecondPartialDerivativeWRTControlSparsity(m_constraintsControlHessianSparsity);
                printError(constraintsHaveControlHessianSparsity, "constraints", "control");

                bool constraintsHaveMixedHessianSparsity = m_ocproblem->constraintsSecondPartialDerivativeWRTStateControlSparsity(m_constraintsMixedHessianSparsity);
                printError(constraintsHaveMixedHessianSparsity, "constraints", "state/control");


                bool costsHaveStateHessianSparsity = m_ocproblem->costsSecondPartialDerivativeWRTStateSparsity(m_costsStateHessianSparsity);
                printError(costsHaveStateHessianSparsity, "costs", "state");

                bool costsHaveControlHessianSparsity = m_ocproblem->costsSecondPartialDerivativeWRTControlSparsity(m_costsControlHessianSparsity);
                printError(costsHaveControlHessianSparsity, "costs", "control");

                bool costsHaveMixedHessianSparsity = m_ocproblem->costsSecondPartialDerivativeWRTStateControlSparsity(m_costsMixedHessianSparsity);
                printError(costsHaveMixedHessianSparsity, "costs", "state/control");

                bool systemHasStateHessianSparsity = m_integrator->getCollocationConstraintSecondDerivativeWRTStateSparsity(m_systemStateHessianSparsity);
                bool systemHasControlHessianSparsity = m_integrator->getCollocationConstraintSecondDerivativeWRTControlSparsity(m_systemControlHessianSparsity);
                bool systemHasMixedHessianSparsity = m_integrator->getCollocationConstraintSecondDerivativeWRTStateControlSparsity(m_systemMixedHessianSparsity);

                resetNonZerosCount();

                std::vector<MeshPoint>::iterator mesh = m_meshPoints.begin(), previousControlMesh = mesh;
                size_t index = 0, constraintIndex = 0;
                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                while (mesh != m_meshPointsEnd){
                    if (mesh->origin == first){
                        //setting up the indeces
                        mesh->stateIndex = index; //the initial state goes first
                        index += nx;
                        mesh->controlIndex = index;
                        mesh->previousControlIndex = index;
                        index += nu;
                        previousControlMesh = mesh;

                        //Saving constraints bounds
                        if (!(m_ocproblem->getConstraintsLowerBound(mesh->time, m_minusInfinity, m_constraintsBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating constraints lower bounds at time " << mesh->time << ".";
                            reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                        }
                        lowerBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nc)) = toEigen(m_constraintsBuffer);

                        if (!(m_ocproblem->getConstraintsUpperBound(mesh->time, m_plusInfinity, m_constraintsBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating constraints upper bounds at time " << mesh->time << ".";
                            reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                        }
                        upperBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nc)) = toEigen(m_constraintsBuffer);

                        for (unsigned int i = 0; i < nc; ++i) {
                            if ((m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i)) <= m_minusInfinity) &&
                                    (m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i)) >= m_plusInfinity)) { //unbounded case
                                m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i)) = m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i))/2; //avoid unboundness
                                m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i)) = m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i))/2; //avoid unboundness
                            }
                        }

                        //Saving the jacobian structure due to the constraints (state should not be constrained here)
                        if (ocHasControlJacobianSparsisty) {
                            addJacobianBlock(constraintIndex, mesh->controlIndex, m_ocControlJacobianSparsity);
                        } else {
                            addJacobianBlock(constraintIndex, nc, mesh->controlIndex, nu);
                        }
                        constraintIndex += nc;

                        //Saving the hessian structure
                        if (!m_info.costIsLinear()) {
                            addHessianSparsityBlock(costsHaveStateHessianSparsity, mesh->stateIndex, nx, mesh->stateIndex, nx, m_costsStateHessianSparsity);
                            addHessianSparsityBlock(costsHaveMixedHessianSparsity, mesh->stateIndex, nx, mesh->controlIndex, nu, m_costsMixedHessianSparsity);
                            addHessianSparsityBlock(costsHaveControlHessianSparsity, mesh->controlIndex, nu, mesh->controlIndex, nu, m_costsControlHessianSparsity);
                        }
                        if (m_info.hasNonLinearConstraints()){
                            addHessianSparsityBlock(constraintsHaveControlHessianSparsity, mesh->controlIndex, nu, mesh->controlIndex, nu, m_constraintsControlHessianSparsity);
                        }

                    } else if (mesh->type == MeshPointType::Control) {
                        mesh->previousControlIndex = previousControlMesh->controlIndex;
                        mesh->controlIndex = index;
                        index += nu;
                        mesh->stateIndex = index;
                        index += nx;
                        previousControlMesh = mesh;

                        //Saving dynamical constraints bounds
                        lowerBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nx)).setZero();
                        upperBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nx)).setZero();

                        //Saving the jacobian structure due to the dynamical constraints
                        if (systemHasControlJacobianSparsity) {
                            addJacobianBlock(constraintIndex, mesh->controlIndex, m_collocationControlJacobianNZ[1]);
                            addJacobianBlock(constraintIndex, mesh->previousControlIndex, m_collocationControlJacobianNZ[0]);
                        } else {
                            addJacobianBlock(constraintIndex, nx, mesh->controlIndex, nu);
                            addJacobianBlock(constraintIndex, nx, mesh->previousControlIndex, nu);
                        }

                        if (systemHasStateJacobianSparsity) {
                            addJacobianBlock(constraintIndex, mesh->stateIndex, m_collocationStateJacobianNZ[1]);
                            addJacobianBlock(constraintIndex, (mesh - 1)->stateIndex, m_collocationStateJacobianNZ[0]);
                        } else {
                            addJacobianBlock(constraintIndex, nx, mesh->stateIndex, nx);
                            addJacobianBlock(constraintIndex, nx, (mesh - 1)->stateIndex, nx);
                        }

                        constraintIndex += nx;

                        //Saving constraints bounds
                        if (!(m_ocproblem->getConstraintsLowerBound(mesh->time, m_minusInfinity, m_constraintsBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating constraints lower bounds at time " << mesh->time << ".";
                            reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                        }
                        lowerBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nc)) = toEigen(m_constraintsBuffer);

                        if (!(m_ocproblem->getConstraintsUpperBound(mesh->time, m_plusInfinity, m_constraintsBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating constraints upper bounds at time " << mesh->time << ".";
                            reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                        }
                        upperBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nc)) = toEigen(m_constraintsBuffer);

                        for (unsigned int i = 0; i < nc; ++i) {
                            if ((m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i)) <= m_minusInfinity) &&
                                    (m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i)) >= m_plusInfinity)) { //unbounded case
                                m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i)) = m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i))/2; //avoid unboundness
                                m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i)) = m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i))/2; //avoid unboundness
                            }
                        }

                        //Saving the jacobian structure due to the constraints
                        if (ocHasStateJacobianSparsisty) {
                            addJacobianBlock(constraintIndex, mesh->stateIndex, m_ocStateJacobianSparsity);
                        } else {
                            addJacobianBlock(constraintIndex, nc, mesh->stateIndex, nx);
                        }
                        if (ocHasControlJacobianSparsisty) {
                            addJacobianBlock(constraintIndex, mesh->controlIndex, m_ocControlJacobianSparsity);
                        } else {
                            addJacobianBlock(constraintIndex, nc, mesh->controlIndex, nu);
                        }
                        constraintIndex += nc;

                        //Saving the hessian structure
                        if (!m_info.costIsLinear()) {
                            addHessianSparsityBlock(costsHaveStateHessianSparsity, mesh->stateIndex, nx, mesh->stateIndex, nx, m_costsStateHessianSparsity);
                            addHessianSparsityBlock(costsHaveMixedHessianSparsity, mesh->stateIndex, nx, mesh->controlIndex, nu, m_costsMixedHessianSparsity);
                            addHessianSparsityBlock(costsHaveControlHessianSparsity, mesh->controlIndex, nu, mesh->controlIndex, nu, m_costsControlHessianSparsity);
                        }
                        if (m_info.hasNonLinearConstraints()){
                            addHessianSparsityBlock(constraintsHaveStateHessianSparsity, mesh->stateIndex, nx, mesh->stateIndex, nx, m_constraintsStateHessianSparsity);
                            addHessianSparsityBlock(constraintsHaveMixedHessianSparsity, mesh->stateIndex, nx, mesh->controlIndex, nu, m_constraintsMixedHessianSparsity);
                            addHessianSparsityBlock(constraintsHaveControlHessianSparsity, mesh->controlIndex, nu, mesh->controlIndex, nu, m_constraintsControlHessianSparsity);
                        }

                        if (!(m_ocproblem->systemIsLinear())) {

                            addHessianSparsityBlock(systemHasStateHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    (mesh-1)->stateIndex, nx, m_systemStateHessianSparsity[CollocationHessianIndex(0,0)]);
                            addHessianSparsityBlock(systemHasStateHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    mesh->stateIndex, nx, m_systemStateHessianSparsity[CollocationHessianIndex(0,1)]);
                            addHessianSparsityBlock(systemHasStateHessianSparsity, mesh->stateIndex, nx,
                                                    mesh->stateIndex, nx, m_systemStateHessianSparsity[CollocationHessianIndex(1,1)]);

                            addHessianSparsityBlock(systemHasMixedHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    mesh->previousControlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(0,0)]);
                            addHessianSparsityBlock(systemHasMixedHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    mesh->controlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(0,1)]);
                            addHessianSparsityBlock(systemHasMixedHessianSparsity, mesh->stateIndex, nx,
                                                    mesh->previousControlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(1,0)]);
                            addHessianSparsityBlock(systemHasMixedHessianSparsity, mesh->stateIndex, nx,
                                                    mesh->controlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(1,1)]);

                            addHessianSparsityBlock(systemHasControlHessianSparsity, mesh->previousControlIndex, nu,
                                                    mesh->previousControlIndex, nu, m_systemControlHessianSparsity[CollocationHessianIndex(0,0)]);
                            addHessianSparsityBlock(systemHasControlHessianSparsity, mesh->previousControlIndex, nu,
                                                    mesh->controlIndex, nu, m_systemControlHessianSparsity[CollocationHessianIndex(0,1)]);
                            addHessianSparsityBlock(systemHasControlHessianSparsity, mesh->controlIndex, nu,
                                                    mesh->controlIndex, nu, m_systemControlHessianSparsity[CollocationHessianIndex(1,1)]);
                        }


                    } else if (mesh->type == MeshPointType::State) {
                        mesh->controlIndex = previousControlMesh->controlIndex;
                        mesh->previousControlIndex = previousControlMesh->controlIndex;
                        mesh->stateIndex = index;
                        index += nx;

                        //Saving dynamical constraints bounds
                        lowerBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nx)).setZero();
                        upperBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nx)).setZero();

                        //Saving the jacobian structure due to the dynamical constraints
                        if (systemHasControlJacobianSparsity) {
                            addJacobianBlock(constraintIndex, mesh->controlIndex, m_mergedCollocationControlJacobianNZ);
                        } else {
                            addJacobianBlock(constraintIndex, nx, mesh->controlIndex, nu);
                        }

                        if (systemHasStateJacobianSparsity) {
                            addJacobianBlock(constraintIndex, mesh->stateIndex, m_collocationStateJacobianNZ[1]);
                            addJacobianBlock(constraintIndex, (mesh - 1)->stateIndex, m_collocationStateJacobianNZ[0]);
                        } else {
                            addJacobianBlock(constraintIndex, nx, mesh->stateIndex, nx);
                            addJacobianBlock(constraintIndex, nx, (mesh - 1)->stateIndex, nx);
                        }
                        constraintIndex += nx;

                        //Saving constraints bounds
                        if (!(m_ocproblem->getConstraintsLowerBound(mesh->time, m_minusInfinity, m_constraintsBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating constraints lower bounds at time " << mesh->time << ".";
                            reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                        }
                        lowerBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nc)) = toEigen(m_constraintsBuffer);

                        if (!(m_ocproblem->getConstraintsUpperBound(mesh->time, m_plusInfinity, m_constraintsBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating constraints upper bounds at time " << mesh->time << ".";
                            reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                        }
                        upperBoundMap.segment(static_cast<Eigen::Index>(constraintIndex), static_cast<Eigen::Index>(nc)) = toEigen(m_constraintsBuffer);

                        for (unsigned int i = 0; i < nc; ++i) {
                            if ((m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i)) <= m_minusInfinity) &&
                                    (m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i)) >= m_plusInfinity)) { //unbounded case
                                m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i)) = m_constraintsLowerBound(static_cast<unsigned int>(constraintIndex + i))/2; //avoid unboundness
                                m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i)) = m_constraintsUpperBound(static_cast<unsigned int>(constraintIndex + i))/2; //avoid unboundness
                            }
                        }

                        if (ocHasStateJacobianSparsisty) {
                            addJacobianBlock(constraintIndex, mesh->stateIndex, m_ocStateJacobianSparsity);
                        } else {
                            addJacobianBlock(constraintIndex, nc, mesh->stateIndex, nx);
                        }
                        if (ocHasControlJacobianSparsisty) {
                            addJacobianBlock(constraintIndex, mesh->controlIndex, m_ocControlJacobianSparsity);
                        } else {
                            addJacobianBlock(constraintIndex, nc, mesh->controlIndex, nu);
                        }
                        constraintIndex += nc;

                        //Saving the hessian structure
                        if (!m_info.costIsLinear()) {
                            addHessianSparsityBlock(costsHaveStateHessianSparsity, mesh->stateIndex, nx, mesh->stateIndex, nx, m_costsStateHessianSparsity);
                            addHessianSparsityBlock(costsHaveMixedHessianSparsity, mesh->stateIndex, nx, mesh->controlIndex, nu, m_costsMixedHessianSparsity);
//                            addHessianSparsityBlock(costsHaveControlHessianSparsity, mesh->controlIndex, nu, mesh->controlIndex, nu, m_costsControlHessianSparsity); // This was already inserted before
                        }
                        if (m_info.hasNonLinearConstraints()){
                            addHessianSparsityBlock(constraintsHaveStateHessianSparsity, mesh->stateIndex, nx, mesh->stateIndex, nx, m_constraintsStateHessianSparsity);
                            addHessianSparsityBlock(constraintsHaveMixedHessianSparsity, mesh->stateIndex, nx, mesh->controlIndex, nu, m_constraintsMixedHessianSparsity);
//                            addHessianSparsityBlock(constraintsHaveControlHessianSparsity, mesh->controlIndex, nu, mesh->controlIndex, nu, m_constraintsControlHessianSparsity); // This was already inserted before
                        }

                        if (!(m_ocproblem->systemIsLinear())) {

                            addHessianSparsityBlock(systemHasStateHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    (mesh-1)->stateIndex, nx, m_systemStateHessianSparsity[CollocationHessianIndex(0,0)]);
                            addHessianSparsityBlock(systemHasStateHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    mesh->stateIndex, nx, m_systemStateHessianSparsity[CollocationHessianIndex(0,1)]);
                            addHessianSparsityBlock(systemHasStateHessianSparsity, mesh->stateIndex, nx,
                                                    mesh->stateIndex, nx, m_systemStateHessianSparsity[CollocationHessianIndex(1,1)]);

                            addHessianSparsityBlock(systemHasMixedHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    mesh->previousControlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(0,0)]);
                            addHessianSparsityBlock(systemHasMixedHessianSparsity, (mesh-1)->stateIndex, nx,
                                                    mesh->controlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(0,1)]);
                            addHessianSparsityBlock(systemHasMixedHessianSparsity, mesh->stateIndex, nx,
                                                    mesh->previousControlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(1,0)]);
                            addHessianSparsityBlock(systemHasMixedHessianSparsity, mesh->stateIndex, nx,
                                                    mesh->controlIndex, nu, m_systemMixedHessianSparsity[CollocationHessianIndex(1,1)]);

//                            addHessianSparsityBlock(systemHasControlHessianSparsity, mesh->previousControlIndex, nu,
//                                                    mesh->previousControlIndex, nu, m_systemControlHessianSparsity[CollocationHessianIndex(0,0)]); // This was already inserted before
//                            addHessianSparsityBlock(systemHasControlHessianSparsity, mesh->previousControlIndex, nu,
//                                                    mesh->controlIndex, nu, m_systemControlHessianSparsity[CollocationHessianIndex(0,1)]); // This was already inserted before
//                            addHessianSparsityBlock(systemHasControlHessianSparsity, mesh->controlIndex, nu,
//                                                    mesh->controlIndex, nu, m_systemControlHessianSparsity[CollocationHessianIndex(1,1)]); // This was already inserted before
                        }

                    }
                    ++mesh;
                }
                assert(index == m_numberOfVariables);
                assert(constraintIndex == m_numberOfConstraints);

                m_hessianBlocks.getFullSparsity(m_fullHessianSparsity);

                if (m_useCostRegularization || m_useConstraintsRegularization) {
                    m_fullHessianSparsity.addIdentityBlock(0, 0, m_numberOfVariables);
                }

                m_prepared = true;
                return true;
            }

            virtual void reset() override {
                m_prepared = false;
                m_totalMeshes = 0;
                m_controlMeshes = 0;
                m_numberOfVariables = 0;
                resetNonZerosCount();
                resetMeshPoints();
            }

            virtual unsigned int numberOfVariables() override {
                return static_cast<unsigned int>(m_numberOfVariables);
            }

            virtual unsigned int numberOfConstraints() override {
                return static_cast<unsigned int>(m_numberOfConstraints);
            }

            virtual bool getConstraintsBounds(VectorDynSize& constraintsLowerBounds, VectorDynSize& constraintsUpperBounds) override {
                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "getConstraintsInfo", "First you need to call the prepare method");
                    return false;
                }

                if (constraintsLowerBounds.size() != numberOfConstraints()) {
                    constraintsLowerBounds.resize(numberOfConstraints());
                }

                if (constraintsUpperBounds.size() != numberOfConstraints()) {
                    constraintsUpperBounds.resize(numberOfConstraints());
                }

                constraintsLowerBounds = m_constraintsLowerBound;
                constraintsUpperBounds = m_constraintsUpperBound;

                return true;
            }

            virtual bool getVariablesUpperBound(VectorDynSize& variablesUpperBound) override {

                Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_stateBuffer);
                Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_controlBuffer);


                if (variablesUpperBound.size() != m_numberOfVariables) {
                    variablesUpperBound.resize(numberOfVariables());
                }
                Eigen::Map<Eigen::VectorXd> upperBoundMap = toEigen(variablesUpperBound);

                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                bool isBounded;
                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){
                    if (mesh->origin == first){
                        upperBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = toEigen(m_ocproblem->dynamicalSystem().lock()->initialState());

                        isBounded = m_ocproblem->getControlUpperBound(mesh->time, m_controlBuffer);
                        if (isBounded) {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = controlBufferMap;
                        } else {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu).setConstant(m_plusInfinity);
                        }

                    } else if (mesh->type == MeshPointType::Control) {

                        isBounded = m_ocproblem->getControlUpperBound(mesh->time, m_controlBuffer);
                        if (isBounded) {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = controlBufferMap;
                        } else {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu).setConstant(m_plusInfinity);
                        }

                        isBounded = m_ocproblem->getStateUpperBound(mesh->time, m_stateBuffer);
                        if (isBounded) {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = stateBufferMap;
                        } else {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx).setConstant(m_plusInfinity);
                        }

                    } else if (mesh->type == MeshPointType::State) {

                        isBounded = m_ocproblem->getStateUpperBound(mesh->time, m_stateBuffer);
                        if (isBounded) {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = stateBufferMap;
                        } else {
                            upperBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx).setConstant(m_plusInfinity);
                        }

                    }
                }
                return true;
            }

            virtual bool getVariablesLowerBound(VectorDynSize& variablesLowerBound) override {

                Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_stateBuffer);
                Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_controlBuffer);


                if (variablesLowerBound.size() != m_numberOfVariables) {
                    variablesLowerBound.resize(numberOfVariables());
                }
                Eigen::Map<Eigen::VectorXd> lowerBoundMap = toEigen(variablesLowerBound);

                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                bool isBounded;
                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){
                    if (mesh->origin == first){
                        lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = toEigen(m_ocproblem->dynamicalSystem().lock()->initialState());
                        isBounded = m_ocproblem->getControlLowerBound(mesh->time, m_controlBuffer);
                        if (isBounded) {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = controlBufferMap;
                        } else {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu).setConstant(m_minusInfinity);
                        }

                    } else if (mesh->type == MeshPointType::Control) {

                        isBounded = m_ocproblem->getControlLowerBound(mesh->time, m_controlBuffer);
                        if (isBounded) {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = controlBufferMap;
                        } else {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu).setConstant(m_minusInfinity);
                        }

                        isBounded = m_ocproblem->getStateLowerBound(mesh->time, m_stateBuffer);
                        if (isBounded) {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = stateBufferMap;
                        } else {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx).setConstant(m_minusInfinity);
                        }

                    } else if (mesh->type == MeshPointType::State) {

                        isBounded = m_ocproblem->getStateLowerBound(mesh->time, m_stateBuffer);
                        if (isBounded) {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = stateBufferMap;
                        } else {
                            lowerBoundMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx).setConstant(m_minusInfinity);
                        }

                    }
                }
                return true;
            }

            virtual bool getConstraintsJacobianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns) override {

                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "getConstraintsInfo", "First you need to call the prepare method");
                    return false;
                }

                if (nonZeroElementRows.size() != m_jacobianNonZeros) {
                    nonZeroElementRows.resize(static_cast<unsigned int>(m_jacobianNonZeros));
                }

                if (nonZeroElementColumns.size() != m_jacobianNonZeros) {
                    nonZeroElementColumns.resize(static_cast<unsigned int>(m_jacobianNonZeros));
                }


                for (unsigned int i = 0; i < m_jacobianNonZeros; ++i){ //not the full vector of nonZeros should be read
                    nonZeroElementRows[i] = m_jacobianNZRows[i];
                    nonZeroElementColumns[i] = m_jacobianNZCols[i];
                }

                return true;
            }

            virtual bool getHessianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns) override {
                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "getHessianInfo", "First you need to call the prepare method");
                    return false;
                }

                nonZeroElementRows = m_fullHessianSparsity.nonZeroElementRows();
                nonZeroElementColumns = m_fullHessianSparsity.nonZeroElementColumns();

                return true;
            }

            bool getGuess(VectorDynSize &guess) override {
                if (!m_stateGuesses || !m_controlGuesses) { //no guess is available (this is not an error)
                    return false;
                }

                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "getGuess", "First you need to call the prepare method");
                    return false;
                }

                iDynTreeEigenVector guessMap = toEigen(m_guessBuffer);
                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                bool isValid = false;
                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){
                    if (mesh->origin == first){

                        const iDynTree::VectorDynSize &controlGuess = m_controlGuesses->get(mesh->time, isValid);

                        if (!isValid || (controlGuess.size() != nu)) {
                            std::ostringstream errorMsg;
                            errorMsg << "Unable to get a valid control guess at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "getGuess", errorMsg.str().c_str());
                            return false;
                        }

                        guessMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = toEigen(controlGuess);

                        guessMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) =
                                toEigen(m_ocproblem->dynamicalSystem().lock()->initialState());

                    } else if (mesh->type == MeshPointType::Control) {

                        const iDynTree::VectorDynSize &controlGuess = m_controlGuesses->get(mesh->time, isValid);

                        if (!isValid || (controlGuess.size() != nu)) {
                            std::ostringstream errorMsg;
                            errorMsg << "Unable to get a valid control guess at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "getGuess", errorMsg.str().c_str());
                            return false;
                        }


                        const iDynTree::VectorDynSize &stateGuess = m_stateGuesses->get(mesh->time, isValid);

                        if (!isValid || (stateGuess.size() != nx)) {
                            std::ostringstream errorMsg;
                            errorMsg << "Unable to get a valid state guess at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "getGuess", errorMsg.str().c_str());
                            return false;
                        }

                        guessMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = toEigen(controlGuess);
                        guessMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = toEigen(stateGuess);
                    } else if (mesh->type == MeshPointType::State) {

                        const iDynTree::VectorDynSize &stateGuess = m_stateGuesses->get(mesh->time, isValid);

                        if (!isValid || (stateGuess.size() != nx)) {
                            std::ostringstream errorMsg;
                            errorMsg << "Unable to get a valid state guess at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "getGuess", errorMsg.str().c_str());
                            return false;
                        }

                        guessMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = toEigen(stateGuess);
                    }
                }

                guess = m_guessBuffer;

                return true;
            }

            virtual bool setVariables(const VectorDynSize& variables) override {
                if (variables.size() != m_variablesBuffer.size()){
                    reportError("MultipleShootingTranscription", "setVariables", "The input variables have a size different from the expected one.");
                    return false;
                }

                m_variablesBuffer = variables;
                return true;
            }

            virtual bool evaluateCostFunction(double& costValue) override {
                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "evaluateCostFunction", "First you need to call the prepare method");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_stateBuffer);
                Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_controlBuffer);
                Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_variablesBuffer);

                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);

                costValue = 0;
                double singleCost;

                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    stateBufferMap = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx);
                    controlBufferMap = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu);

                    if (!(m_ocproblem->costsEvaluation(mesh->time, m_stateBuffer, m_controlBuffer, singleCost))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostFunction", errorMsg.str().c_str());
                        return false;
                    }

                    costValue += singleCost;

                }
                return true;
            }

            virtual bool evaluateCostGradient(VectorDynSize& gradient) override {

                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "evaluateCostGradient", "First you need to call the prepare method");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_stateBuffer);
                Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_controlBuffer);
                Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_variablesBuffer);
                Eigen::Map<Eigen::VectorXd> costStateGradient = toEigen(m_costStateGradientBuffer);
                Eigen::Map<Eigen::VectorXd> costControlGradient = toEigen(m_costControlGradientBuffer);

                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);

                if (gradient.size() != numberOfVariables()) {
                    gradient.resize(static_cast<unsigned int>(numberOfVariables()));
                }

                Eigen::Map<Eigen::VectorXd> gradientMap = toEigen(gradient);

                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    stateBufferMap = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx);
                    controlBufferMap = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu);

                    if (!(m_ocproblem->costsFirstPartialDerivativeWRTState(mesh->time, m_stateBuffer, m_controlBuffer, m_costStateGradientBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost state gradient at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostGradient", errorMsg.str().c_str());
                        return false;
                    }

                    gradientMap.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx) = costStateGradient;


                    if (!(m_ocproblem->costsFirstPartialDerivativeWRTControl(mesh->time, m_stateBuffer, m_controlBuffer, m_costControlGradientBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost control gradient at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostGradient", errorMsg.str().c_str());
                        return false;
                    }

                    if (mesh->type == MeshPointType::Control){
                        gradientMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) = costControlGradient;
                    } else if (mesh->type == MeshPointType::State) {
                        gradientMap.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu) += costControlGradient;
                    }

                }

                return true;
            }

            virtual bool evaluateCostHessian(MatrixDynSize& hessian) override {

                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "evaluateCostHessian", "First you need to call the prepare method");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_stateBuffer);
                Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_controlBuffer);
                Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_variablesBuffer);
                iDynTreeEigenMatrixMap costStateHessian = toEigen(m_costHessianStateBuffer);
                iDynTreeEigenMatrixMap costControlHessian = toEigen(m_costHessianControlBuffer);
                iDynTreeEigenMatrixMap costStateControlHessian = toEigen(m_costHessianStateControlBuffer);
                iDynTreeEigenMatrixMap costControlStateHessian = toEigen(m_costHessianControlStateBuffer);


                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);

                if ((hessian.rows() != numberOfVariables()) || (hessian.cols() != numberOfVariables())) {
                    hessian.resize(static_cast<unsigned int>(numberOfVariables()), static_cast<unsigned int>(numberOfVariables()));
                }

                iDynTreeEigenMatrixMap hessianMap = toEigen(hessian);

                if (m_useCostRegularization) {
                    for (unsigned int i = 0; i < hessian.rows(); ++i) {
                        hessian(i, i) = 0.0;
                    }
                }

                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    stateBufferMap = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx);
                    controlBufferMap = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu);

                    if (!(m_ocproblem->costsSecondPartialDerivativeWRTState(mesh->time, m_stateBuffer, m_controlBuffer, m_costHessianStateBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost state hessian at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostHessian", errorMsg.str().c_str());
                        return false;
                    }

                    hessianMap.block(static_cast<Eigen::Index>(mesh->stateIndex), static_cast<Eigen::Index>(mesh->stateIndex), nx, nx) = costStateHessian;

                    if (!(m_ocproblem->costsSecondPartialDerivativeWRTStateControl(mesh->time, m_stateBuffer, m_controlBuffer, m_costHessianStateControlBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost state-control hessian at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostHessian", errorMsg.str().c_str());
                        return false;
                    }

                    hessianMap.block(static_cast<Eigen::Index>(mesh->stateIndex), static_cast<Eigen::Index>(mesh->controlIndex), nx, nu) = costStateControlHessian;
                    costControlStateHessian = costStateControlHessian.transpose();
                    hessianMap.block(static_cast<Eigen::Index>(mesh->controlIndex), static_cast<Eigen::Index>(mesh->stateIndex), nu, nx) = costControlStateHessian;

                    if (!(m_ocproblem->costsSecondPartialDerivativeWRTControl(mesh->time, m_stateBuffer, m_controlBuffer, m_costHessianControlBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost control hessian at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostHessian", errorMsg.str().c_str());
                        return false;
                    }

                    if (mesh->type == MeshPointType::Control){
                        hessianMap.block(static_cast<Eigen::Index>(mesh->controlIndex), static_cast<Eigen::Index>(mesh->controlIndex), nu, nu) = costControlHessian;
                    } else if (mesh->type == MeshPointType::State) {
                        hessianMap.block(static_cast<Eigen::Index>(mesh->controlIndex), static_cast<Eigen::Index>(mesh->controlIndex), nu, nu) += costControlHessian;
                    }
                }

                if (m_useCostRegularization) {
                    for (unsigned int i = 0; i < hessian.rows(); ++i) {
                        hessian(i, i) += m_costsRegularization;
                    }
                }

                return true;
            }

            virtual bool evaluateConstraints(VectorDynSize& constraints) override {
                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "evaluateConstraints", "First you need to call the prepare method");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_stateBuffer);
                Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_variablesBuffer);
                Eigen::Map<Eigen::VectorXd> constraintsBufferMap = toEigen(m_constraintsBuffer);
                Eigen::Map<Eigen::VectorXd> currentState = toEigen(m_collocationStateBuffer[1]);
                Eigen::Map<Eigen::VectorXd> previousState = toEigen(m_collocationStateBuffer[0]);
                Eigen::Map<Eigen::VectorXd> currentControl = toEigen(m_collocationControlBuffer[1]);
                Eigen::Map<Eigen::VectorXd> previousControl = toEigen(m_collocationControlBuffer[0]);


                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);
                Eigen::Index nc = static_cast<Eigen::Index>(m_constraintsPerInstant);

                if (constraints.size() != numberOfConstraints()) {
                    constraints.resize(static_cast<unsigned int>(numberOfConstraints()));
                }

                Eigen::Map<Eigen::VectorXd> constraintsMap = toEigen(constraints);


                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                Eigen::Index constraintIndex = 0;
                double dT = 0;
                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    currentState = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx);

                    currentControl  = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu);
                    previousControl = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->previousControlIndex), nu);

                    if (mesh->origin != first) {
                        previousState = variablesBuffer.segment(static_cast<Eigen::Index>((mesh - 1)->stateIndex), nx);
                        dT = mesh->time - (mesh - 1)->time;
                        if (!(m_integrator->evaluateCollocationConstraint(mesh->time, m_collocationStateBuffer, m_collocationControlBuffer, dT, m_stateBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating the collocation constraint at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "evaluateConstraints", errorMsg.str().c_str());
                            return false;
                        }
                        constraintsMap.segment(constraintIndex, nx) = stateBufferMap;
                        constraintIndex += nx;
                    }

                    bool okConstraint = true;
                    if (mesh->origin == first) {
                        okConstraint = m_ocproblem->constraintsEvaluation(mesh->time, m_integrator->dynamicalSystem().lock()->initialState(), m_collocationControlBuffer[1], m_constraintsBuffer);

                    } else {
                        okConstraint = m_ocproblem->constraintsEvaluation(mesh->time, m_collocationStateBuffer[1], m_collocationControlBuffer[1], m_constraintsBuffer);
                    }
                    if (!okConstraint){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating the constraints at time " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateConstraints", errorMsg.str().c_str());
                        return false;
                    }
                    constraintsMap.segment(constraintIndex, nc) = constraintsBufferMap;
                    constraintIndex += nc;
                }
                assert(static_cast<size_t>(constraintIndex) == numberOfConstraints());
                return true;
            }

            virtual bool evaluateConstraintsJacobian(MatrixDynSize& jacobian) override {

                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "evaluateConstraints", "First you need to call the prepare method");
                    return false;
                }

                Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_variablesBuffer);
                Eigen::Map<Eigen::VectorXd> currentState = toEigen(m_collocationStateBuffer[1]);
                Eigen::Map<Eigen::VectorXd> previousState = toEigen(m_collocationStateBuffer[0]);
                Eigen::Map<Eigen::VectorXd> currentControl = toEigen(m_collocationControlBuffer[1]);
                Eigen::Map<Eigen::VectorXd> previousControl = toEigen(m_collocationControlBuffer[0]);


                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);
                Eigen::Index nc = static_cast<Eigen::Index>(m_constraintsPerInstant);

                if (jacobian.rows() != numberOfConstraints() || jacobian.cols() != numberOfVariables()) {
                    jacobian.resize(numberOfConstraints(), numberOfVariables());
                }

                iDynTreeEigenMatrixMap jacobianMap = toEigen(jacobian);

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                Eigen::Index constraintIndex = 0;
                double dT = 0;
                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    currentState = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx);
                    currentControl  = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu);
                    previousControl = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->previousControlIndex), nu);

                    if (mesh->origin != first) {
                        previousState = variablesBuffer.segment(static_cast<Eigen::Index>((mesh - 1)->stateIndex), nx);
                        dT = mesh->time - (mesh - 1)->time;
                        if (!(m_integrator->evaluateCollocationConstraintJacobian(mesh->time, m_collocationStateBuffer, m_collocationControlBuffer, dT, m_collocationStateJacBuffer, m_collocationControlJacBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating the collocation constraint jacobian at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                            return false;
                        }


                        jacobianMap.block(constraintIndex, static_cast<Eigen::Index>((mesh-1)->stateIndex), nx, nx) = toEigen(m_collocationStateJacBuffer[0]);

                        jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->stateIndex), nx, nx) = toEigen(m_collocationStateJacBuffer[1]);

                        jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->controlIndex), nx, nu) = toEigen(m_collocationControlJacBuffer[1]);

                        if (mesh->type == MeshPointType::Control) {
                            jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->previousControlIndex), nx, nu) = toEigen(m_collocationControlJacBuffer[0]);
                        } else if (mesh->type == MeshPointType::State) {
                            jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->previousControlIndex), nx, nu) += toEigen(m_collocationControlJacBuffer[0]); //the previous and the current control coincides
                        }
                        constraintIndex += nx;
                    }


                    if (mesh->origin == first) {

                        if (!(m_ocproblem->constraintsJacobianWRTControl(mesh->time, m_integrator->dynamicalSystem().lock()->initialState(), m_collocationControlBuffer[1], m_constraintsControlJacBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating the constraints control jacobian at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                            return false;
                        }

                        jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->controlIndex), nc, nu) = toEigen(m_constraintsControlJacBuffer);

                    } else {

                        if (!(m_ocproblem->constraintsJacobianWRTState(mesh->time, m_collocationStateBuffer[1], m_collocationControlBuffer[1], m_constraintsStateJacBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating the constraints state jacobian at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                            return false;
                        }

                        jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->stateIndex), nc, nx) = toEigen(m_constraintsStateJacBuffer);

                        if (!(m_ocproblem->constraintsJacobianWRTControl(mesh->time, m_collocationStateBuffer[1], m_collocationControlBuffer[1], m_constraintsControlJacBuffer))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating the constraints control jacobian at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                            return false;
                        }

                        jacobianMap.block(constraintIndex, static_cast<Eigen::Index>(mesh->controlIndex), nc, nu) = toEigen(m_constraintsControlJacBuffer);
                    }
                    constraintIndex += nc;
                }
                assert(static_cast<size_t>(constraintIndex) == m_numberOfConstraints);
                return true;
            }

            virtual bool evaluateConstraintsHessian(const VectorDynSize& constraintsMultipliers, MatrixDynSize& hessian) override {

                if (!(m_prepared)){
                    reportError("MultipleShootingTranscription", "evaluateConstraints", "First you need to call the prepare method");
                    return false;
                }

                if (!m_info.hasNonLinearConstraints()) {
                    hessian.resize(static_cast<unsigned int>(m_numberOfVariables), static_cast<unsigned int>(m_numberOfVariables));
                    hessian.zero();
                    return true;
                }

                Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_variablesBuffer);
                Eigen::Map<Eigen::VectorXd> currentState = toEigen(m_collocationStateBuffer[1]);
                Eigen::Map<Eigen::VectorXd> previousState = toEigen(m_collocationStateBuffer[0]);
                Eigen::Map<Eigen::VectorXd> currentControl = toEigen(m_collocationControlBuffer[1]);
                Eigen::Map<Eigen::VectorXd> previousControl = toEigen(m_collocationControlBuffer[0]);
                Eigen::Map<Eigen::VectorXd> lambdaCollocation = toEigen(m_lambdaCollocation);
                Eigen::Map<Eigen::VectorXd> lambdaConstraints = toEigen(m_lambdaConstraints);
                Eigen::Map<const Eigen::VectorXd> fullLambda = toEigen(constraintsMultipliers);


                Eigen::Index nx = static_cast<Eigen::Index>(m_nx);
                Eigen::Index nu = static_cast<Eigen::Index>(m_nu);
                Eigen::Index nc = static_cast<Eigen::Index>(m_constraintsPerInstant);

                if ((hessian.rows() != numberOfVariables()) || (hessian.cols() != numberOfVariables())) {
                    hessian.resize(static_cast<unsigned int>(numberOfVariables()), static_cast<unsigned int>(numberOfVariables()));
                }

                if (m_useConstraintsRegularization) {
                    for (unsigned int i = 0; i < hessian.rows(); ++i) {
                        hessian(i, i) = 0.0;
                    }
                }

                m_hessianBlocks.reset();

                MeshPointOrigin first = MeshPointOrigin::FirstPoint();
                Eigen::Index constraintIndex = 0;
                double dT = 0;
                for (auto mesh = m_meshPoints.begin(); mesh != m_meshPointsEnd; ++mesh){

                    currentState = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->stateIndex), nx);
                    currentControl  = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->controlIndex), nu);
                    previousControl = variablesBuffer.segment(static_cast<Eigen::Index>(mesh->previousControlIndex), nu);

                    if (mesh->origin != first) {
                        previousState = variablesBuffer.segment(static_cast<Eigen::Index>((mesh - 1)->stateIndex), nx);
                        lambdaCollocation = fullLambda.segment(constraintIndex, nx);
                        dT = mesh->time - (mesh - 1)->time;

                        if (!(m_integrator->evaluateCollocationConstraintSecondDerivatives(mesh->time, m_collocationStateBuffer, m_collocationControlBuffer, dT,
                                                                                           m_lambdaCollocation, m_stateCollocationHessians,
                                                                                           m_controlCollocationHessians, m_stateControCollocationlHessians))){
                            std::ostringstream errorMsg;
                            errorMsg << "Error while evaluating the collocation constraint hessian at time " << mesh->time << ".";
                            reportError("MultipleShootingTranscription", "evaluateConstraintsHessian", errorMsg.str().c_str());
                            return false;
                        }

                        setHessianBlock(hessian, m_stateCollocationHessians[CollocationHessianIndex(0, 0)], (mesh-1)->stateIndex, (mesh-1)->stateIndex);
                        setHessianBlockAndItsTranspose(hessian, m_stateCollocationHessians[CollocationHessianIndex(0, 1)], (mesh-1)->stateIndex, mesh->stateIndex);
                        setHessianBlock(hessian, m_stateCollocationHessians[CollocationHessianIndex(1, 1)], mesh->stateIndex, mesh->stateIndex);

                        setHessianBlock(hessian, m_controlCollocationHessians[CollocationHessianIndex(0,0)], mesh->previousControlIndex, mesh->previousControlIndex);
                        setHessianBlockAndItsTranspose(hessian, m_controlCollocationHessians[CollocationHessianIndex(0,1)], mesh->previousControlIndex, mesh->controlIndex);
                        setHessianBlock(hessian, m_controlCollocationHessians[CollocationHessianIndex(1,1)], mesh->controlIndex, mesh->controlIndex);

                        setHessianBlockAndItsTranspose(hessian, m_stateControCollocationlHessians[CollocationHessianIndex(0,0)], (mesh-1)->stateIndex, mesh->previousControlIndex);
                        setHessianBlockAndItsTranspose(hessian, m_stateControCollocationlHessians[CollocationHessianIndex(0,1)], (mesh-1)->stateIndex, mesh->controlIndex);
                        setHessianBlockAndItsTranspose(hessian, m_stateControCollocationlHessians[CollocationHessianIndex(1,0)], mesh->stateIndex, mesh->previousControlIndex);
                        setHessianBlockAndItsTranspose(hessian, m_stateControCollocationlHessians[CollocationHessianIndex(1,1)], mesh->stateIndex, mesh->controlIndex);

                        constraintIndex += nx;
                    }


                    if (nc != 0) {
                        if (mesh->origin == first) {

                            lambdaConstraints = fullLambda.segment(constraintIndex, nc);

                            if (!(m_ocproblem->constraintsSecondPartialDerivativeWRTControl(mesh->time, m_integrator->dynamicalSystem().lock()->initialState(), m_collocationControlBuffer[1], m_lambdaConstraints, m_controlHessianBuffer))){
                                std::ostringstream errorMsg;
                                errorMsg << "Error while evaluating the constraints hessian wrt control at time " << mesh->time << ".";
                                reportError("MultipleShootingTranscription", "evaluateConstraintsHessian", errorMsg.str().c_str());
                                return false;
                            }

                            setHessianBlock(hessian, m_controlHessianBuffer, mesh->controlIndex, mesh->controlIndex);

                        } else {

                            lambdaConstraints = fullLambda.segment(constraintIndex, nc);

                            if (!(m_ocproblem->constraintsSecondPartialDerivativeWRTState(mesh->time, m_collocationStateBuffer[1], m_collocationControlBuffer[1], m_lambdaConstraints, m_stateHessianBuffer))){
                                std::ostringstream errorMsg;
                                errorMsg << "Error while evaluating the constraints hessian wrt state at time " << mesh->time << ".";
                                reportError("MultipleShootingTranscription", "evaluateConstraintsHessian", errorMsg.str().c_str());
                                return false;
                            }

                            setHessianBlock(hessian, m_stateHessianBuffer, mesh->stateIndex, mesh->stateIndex);

                            if (!(m_ocproblem->constraintsSecondPartialDerivativeWRTStateControl(mesh->time, m_collocationStateBuffer[1], m_collocationControlBuffer[1], m_lambdaConstraints, m_stateControlHessianBuffer))){
                                std::ostringstream errorMsg;
                                errorMsg << "Error while evaluating the constraints hessian wrt state and control at time " << mesh->time << ".";
                                reportError("MultipleShootingTranscription", "evaluateConstraintsHessian", errorMsg.str().c_str());
                                return false;
                            }

                            setHessianBlockAndItsTranspose(hessian, m_stateControlHessianBuffer, mesh->stateIndex, mesh->controlIndex);

                            if (!(m_ocproblem->constraintsSecondPartialDerivativeWRTControl(mesh->time, m_collocationStateBuffer[1], m_collocationControlBuffer[1], m_lambdaConstraints, m_controlHessianBuffer))){
                                std::ostringstream errorMsg;
                                errorMsg << "Error while evaluating the constraints hessian wrt control at time " << mesh->time << ".";
                                reportError("MultipleShootingTranscription", "evaluateConstraintsHessian", errorMsg.str().c_str());
                                return false;
                            }

                            setHessianBlock(hessian, m_controlHessianBuffer, mesh->controlIndex, mesh->controlIndex);

                        }
                        constraintIndex += nc;
                    }

                }
                assert(static_cast<size_t>(constraintIndex) == m_numberOfConstraints);

                if (m_useConstraintsRegularization) {
                    for (unsigned int i = 0; i < hessian.rows(); ++i) {
                        hessian(i, i) += m_constraintsRegularization;
                    }
                }

                return true;
            }
        };
        MultipleShootingSolver::MultipleShootingTranscription::~MultipleShootingTranscription() {}

        // MARK: Class implementation

        MultipleShootingSolver::MultipleShootingSolver(const std::shared_ptr<OptimalControlProblem> &ocProblem)
        : OptimalControlSolver(ocProblem)
        {
            m_transcription.reset(new MultipleShootingTranscription());
            assert(m_transcription);
            m_transcription->setOptimalControlProblem(ocProblem);
        }

        bool MultipleShootingSolver::setStepSizeBounds(double minStepSize, double maxStepsize)
        {
            return m_transcription->setStepSizeBounds(minStepSize, maxStepsize);
        }

        bool MultipleShootingSolver::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            return m_transcription->setIntegrator(integrationMethod);
        }

        bool MultipleShootingSolver::setControlPeriod(double period)
        {
            return m_transcription->setControlPeriod(period);
        }

        bool MultipleShootingSolver::setAdditionalStateMeshPoints(const std::vector<double> &stateMeshes)
        {
            return m_transcription->setAdditionalStateMeshPoints(stateMeshes);
        }

        bool MultipleShootingSolver::setAdditionalControlMeshPoints(const std::vector<double> &controlMeshes)
        {
            return m_transcription->setAdditionalControlMeshPoints(controlMeshes);
        }

        bool MultipleShootingSolver::setOptimizer(std::shared_ptr<optimization::Optimizer> optimizer)
        {
            if (!optimizer) {
                reportError("MultipleShootingSolver", "setOptimizer", "Empty optimizer pointer");
                return false;
            }

            if (!(optimizer->setProblem(m_transcription))){
                reportError("MultipleShootingSolver", "setOptimizer", "Cannot use the selected optimizer to solve the specified optimal control problem.");
                return false;
            }

            m_optimizer = optimizer;

            m_transcription->setPlusInfinity(m_optimizer->plusInfinity());

            m_transcription->setMinusInfinity(m_optimizer->minusInfinity());

            return true;
        }

        bool MultipleShootingSolver::setInitialState(const VectorDynSize &initialState)
        {
            return m_transcription->setInitialState(initialState);
        }

        bool MultipleShootingSolver::setGuesses(std::shared_ptr<TimeVaryingVector> stateGuesses,
                                                std::shared_ptr<TimeVaryingVector> controlGuesses)
        {
            if (!stateGuesses) {
                reportError("MultipleShootingSolver", "setGuesses", "Empty state guess pointer.");
                return false;
            }

            if (!controlGuesses) {
                reportError("MultipleShootingSolver", "setGuesses", "Empty control guess pointer.");
                return false;
            }

            m_transcription->m_stateGuesses = stateGuesses;
            m_transcription->m_controlGuesses = controlGuesses;

            return true;

        }

        bool MultipleShootingSolver::getTimings(std::vector<double> &stateEvaluations, std::vector<double> &controlEvaluations)
        {
            return m_transcription->getTimings(stateEvaluations, controlEvaluations);
        }

        bool MultipleShootingSolver::getPossibleTimings(std::vector<double> &stateEvaluations, std::vector<double> &controlEvaluations)
        {
            return m_transcription->getPossibleTimings(stateEvaluations, controlEvaluations);
        }


        bool MultipleShootingSolver::solve()
        {
            if (!m_optimizer){
                reportError("MultipleShootingSolver", "solve", "No optimizer selected.");
                return false;
            }
            if (m_optimizer->solve()){
                if (!(m_optimizer->getPrimalVariables(m_transcription->m_solution))){
                    reportError("MultipleShootingSolver", "solve", "Error while retrieving the primal variables from the optimizer.");
                    return false;
                }
            } else {
                reportError("MultipleShootingSolver", "solve", "Error when calling the optimizer solve method.");
                return false;
            }
            m_transcription->m_solved = true;

            m_transcription->m_stateGuesses = nullptr; //Guesses are used only once
            m_transcription->m_controlGuesses = nullptr;

            return true;
        }

        bool MultipleShootingSolver::getSolution(std::vector<VectorDynSize> &states, std::vector<VectorDynSize> &controls)
        {
            return m_transcription->getSolution(states, controls);
        }

        void MultipleShootingSolver::resetTranscription()
        {
            m_transcription->reset();
        }

        void iDynTree::optimalcontrol::MultipleShootingSolver::disableCostsHessianRegularization()
        {
            m_transcription->m_useCostRegularization = false;
        }

        void iDynTree::optimalcontrol::MultipleShootingSolver::disableConstraintsHessianRegularization()
        {
            m_transcription->m_useConstraintsRegularization = false;
        }

        void iDynTree::optimalcontrol::MultipleShootingSolver::addCostsHessianRegularization(double regularization)
        {
            m_transcription->m_useCostRegularization = true;
            m_transcription->m_costsRegularization = regularization;
        }

        void iDynTree::optimalcontrol::MultipleShootingSolver::addConstraintsHessianRegularization(double regularization)
        {
            m_transcription->m_useConstraintsRegularization = true;
            m_transcription->m_constraintsRegularization = regularization;
        }
    }
}
