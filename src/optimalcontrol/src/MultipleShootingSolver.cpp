/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OCSolvers/MultipleShootingSolver.h>

#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Integrator.h>
#include <iDynTree/TimeRange.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Utils.h>

#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>

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

        class MultipleShootingTranscription::MultipleShootingTranscriptionPimpl{
        public:
            std::shared_ptr<OptimalControlProblem> ocproblem;
            std::shared_ptr<Integrator> integrator;
            size_t totalMeshes, controlMeshes;
            bool prepared;
            std::vector<double> userStateMeshes, userControlMeshes;
            std::vector<MeshPoint> meshPoints;
            std::vector<MeshPoint>::iterator meshPointsEnd;
            double minStepSize, maxStepSize, controlPeriod;
            size_t nx, nu, numberOfVariables, constraintsPerInstant, numberOfConstraints;
            std::vector<size_t> jacobianNZRows, jacobianNZCols, hessianNZRows, hessianNZCols;
            size_t jacobianNonZeros, hessianNonZeros;
            double plusInfinity, minusInfinity;
            VectorDynSize constraintsLowerBound, constraintsUpperBound;
            VectorDynSize constraintsBuffer, stateBuffer, controlBuffer, variablesBuffer, costStateGradientBuffer, costControlGradientBuffer;
            MatrixDynSize costHessianStateBuffer, costHessianControlBuffer, costHessianStateControlBuffer, costHessianControlStateBuffer;
            std::vector<VectorDynSize> collocationStateBuffer, collocationControlBuffer;
            std::vector<MatrixDynSize> collocationStateJacBuffer, collocationControlJacBuffer;
            MatrixDynSize constraintsStateJacBuffer, constraintsControlJacBuffer;
            VectorDynSize solution;
            bool solved;

            void resetMeshPoints(){
                meshPointsEnd = meshPoints.begin();
            }

            void addMeshPoint(MeshPoint& newMeshPoint){
                if (meshPointsEnd == meshPoints.end()){
                    meshPoints.push_back(newMeshPoint);
                    meshPointsEnd = meshPoints.end();
                } else {
                    *meshPointsEnd = newMeshPoint;
                    ++meshPointsEnd;
                }
            }

            std::vector<MeshPoint>::iterator findNextMeshPoint(std::vector<MeshPoint>::iterator& start){
                std::vector<MeshPoint>::iterator nextMesh = start;
                MeshPointOrigin ignored = MeshPointOrigin::Ignored();
                assert(nextMesh != meshPoints.end());
                assert(nextMesh->origin.name().size() > 0);
                do {
                    ++nextMesh;
                    assert(nextMesh->origin.name().size() > 0);
                    assert(nextMesh != meshPoints.end());
                } while (nextMesh->origin == ignored); //find next valid mesh

                return nextMesh;
            }

            void setIgnoredMesh(std::vector<MeshPoint>::iterator& toBeIgnored){
                assert(toBeIgnored != meshPoints.end());
                toBeIgnored->time = ocproblem->finalTime() + maxStepSize;
                toBeIgnored->origin = MeshPointOrigin::Ignored();
            }

            void cleanLeftoverMeshes(){
                for (auto toBeIgnored = meshPointsEnd; toBeIgnored != meshPoints.end(); toBeIgnored++){
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
                jacobianNonZeros = 0;
                hessianNonZeros = 0;
            }

            void addNonZero(std::vector<size_t>& input, size_t position, size_t toBeAdded){
                assert(input.size() >= position);
                if (position >= input.size())
                    input.push_back(toBeAdded);
                else input[position] = toBeAdded;
            }

            void addJacobianBlock(size_t initRow, size_t rows, size_t initCol, size_t cols){
                for (size_t i = 0; i < rows; ++i){
                    for (size_t j = 0; j < cols; ++j){
                        addNonZero(jacobianNZRows, jacobianNonZeros, initRow + i);
                        addNonZero(jacobianNZCols, jacobianNonZeros, initCol + j);
                        jacobianNonZeros++;
                    }
                }
            }

            void addHessianBlock(size_t initRow, size_t rows, size_t initCol, size_t cols){
                for (size_t i = 0; i < rows; ++i){
                    for (size_t j = 0; j < cols; ++j){
                        addNonZero(hessianNZRows, hessianNonZeros, initRow + i);
                        addNonZero(hessianNZCols, hessianNonZeros, initCol + j);
                        hessianNonZeros++;
                    }
                }
            }

            void allocateBuffers(){
                if (stateBuffer.size() != nx)
                    stateBuffer.resize(static_cast<unsigned int>(nx));

                if (costStateGradientBuffer.size() != nx)
                    costStateGradientBuffer.resize(static_cast<unsigned int>(nx));

                if ((costHessianStateBuffer.rows() != nx) || (costHessianStateBuffer.cols() != nx))
                    costHessianStateBuffer.resize(static_cast<unsigned int>(nx), static_cast<unsigned int>(nx));

                if (controlBuffer.size() != nu)
                    controlBuffer.resize(static_cast<unsigned int>(nu));

                if (costControlGradientBuffer.size() != nu)
                    costControlGradientBuffer.resize(static_cast<unsigned int>(nu));

                if ((costHessianControlBuffer.rows() != nu) || (costHessianControlBuffer.cols() != nu))
                    costHessianControlBuffer.resize(static_cast<unsigned int>(nu), static_cast<unsigned int>(nu));

                if ((costHessianStateControlBuffer.rows() != nx) || (costHessianStateControlBuffer.cols() != nu))
                    costHessianStateControlBuffer.resize(static_cast<unsigned int>(nx), static_cast<unsigned int>(nu));

                if ((costHessianControlStateBuffer.rows() != nu) || (costHessianControlStateBuffer.cols() != nx))
                    costHessianControlStateBuffer.resize(static_cast<unsigned int>(nu), static_cast<unsigned int>(nx));


                //TODO: I should consider also the possibility to have auxiliary variables in the integrator
                if (variablesBuffer.size() != numberOfVariables)
                    variablesBuffer.resize(static_cast<unsigned int>(numberOfVariables));

                if (constraintsBuffer.size() != constraintsPerInstant)
                    constraintsBuffer.resize(static_cast<unsigned int>(constraintsPerInstant));

                if (constraintsLowerBound.size() != numberOfConstraints)
                    constraintsLowerBound.resize(static_cast<unsigned int>(numberOfConstraints));

                if (constraintsUpperBound.size() != numberOfConstraints)
                    constraintsUpperBound.resize(static_cast<unsigned int>(numberOfConstraints));

                //TODO: I should consider also the possibility to have auxiliary variables in the integrator
                if (collocationStateBuffer.size() != 2)
                    collocationStateBuffer.resize(2);
                for (size_t i = 0; i < 2; ++i)
                    if (collocationStateBuffer[i].size() != nx)
                        collocationStateBuffer[i].resize(static_cast<unsigned int>(nx));

                if (collocationControlBuffer.size() != 2)
                    collocationControlBuffer.resize(2);
                for (size_t i = 0; i < 2; ++i)
                    if (collocationControlBuffer[i].size() != nu)
                        collocationControlBuffer[i].resize(static_cast<unsigned int>(nu));

                if (collocationStateJacBuffer.size() != 2)
                    collocationStateJacBuffer.resize(2);
                for (size_t i = 0; i < 2; ++i)
                    if ((collocationStateJacBuffer[i].rows() != nx) || (collocationStateJacBuffer[i].cols() != nx))
                        collocationStateJacBuffer[i].resize(static_cast<unsigned int>(nx), static_cast<unsigned int>(nx));

                if (collocationControlJacBuffer.size() != 2)
                    collocationControlJacBuffer.resize(2);
                for (size_t i = 0; i < 2; ++i)
                    if ((collocationControlJacBuffer[i].rows() != nx) || (collocationControlJacBuffer[i].cols() != nu))
                        collocationControlJacBuffer[i].resize(static_cast<unsigned int>(nx), static_cast<unsigned int>(nu));

                if ((constraintsStateJacBuffer.rows() != constraintsPerInstant) || (constraintsStateJacBuffer.cols() != nx))
                    constraintsStateJacBuffer.resize(static_cast<unsigned int>(constraintsPerInstant), static_cast<unsigned int>(nx));

                if ((constraintsControlJacBuffer.rows() != constraintsPerInstant) || (constraintsControlJacBuffer.cols() != nu))
                    constraintsStateJacBuffer.resize(static_cast<unsigned int>(constraintsPerInstant), static_cast<unsigned int>(nu));
            }



            MultipleShootingTranscriptionPimpl()
            : ocproblem(nullptr)
            ,integrator(nullptr)
            ,totalMeshes(0)
            ,controlMeshes(0)
            ,prepared(false)
            ,meshPointsEnd(meshPoints.end())
            ,minStepSize(0.001)
            ,maxStepSize(0.01)
            ,controlPeriod(0.01)
            ,nx(0)
            ,nu(0)
            ,numberOfVariables(0)
            ,constraintsPerInstant(0)
            ,numberOfConstraints(0)
            ,plusInfinity(1e19)
            ,minusInfinity(-1e19)
            ,solved(false)
            {}

            MultipleShootingTranscriptionPimpl(const std::shared_ptr<OptimalControlProblem> problem,
                                            const std::shared_ptr<Integrator> integrationMethod)
            :ocproblem(problem)
            ,integrator(integrationMethod)
            ,totalMeshes(0)
            ,controlMeshes(0)
            ,prepared(false)
            ,meshPointsEnd(meshPoints.end())
            ,minStepSize(0.001)
            ,maxStepSize(0.01)
            ,controlPeriod(0.01)
            ,nx(0)
            ,nu(0)
            ,numberOfVariables(0)
            ,constraintsPerInstant(0)
            ,numberOfConstraints(0)
            ,plusInfinity(1e19)
            ,minusInfinity(-1e19)
            ,solved(false)
            {}
        };

        MultipleShootingTranscription::MultipleShootingTranscription()
        :m_pimpl(new MultipleShootingTranscriptionPimpl())
        {
            assert(m_pimpl);
        }

        MultipleShootingTranscription::MultipleShootingTranscription(const std::shared_ptr<OptimalControlProblem> problem,
                                                               const std::shared_ptr<Integrator> integrationMethod)
        :m_pimpl(new MultipleShootingTranscriptionPimpl(problem, integrationMethod))
        {
            assert(m_pimpl);
        }

        size_t MultipleShootingTranscription::setControlMeshPoints()
        {
            double endTime = m_pimpl->ocproblem->finalTime(), initTime = m_pimpl->ocproblem->initialTime();
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

                m_pimpl->addMeshPoint(newMeshPoint);
                time += m_pimpl->controlPeriod;
                controlMeshes++;
            }

            std::vector<MeshPoint>::iterator lastControlMeshIterator = (m_pimpl->meshPointsEnd)-1;

            if ((lastControlMeshIterator->origin == MeshPointOrigin::LastPoint()) && (m_pimpl->integrator->info().isExplicit())) {//the last control input would have no effect
                controlMeshes--;
                m_pimpl->setIgnoredMesh(lastControlMeshIterator);
            }

            newMeshPoint.origin = MeshPointOrigin::LastPoint();
            newMeshPoint.type = MeshPointType::State;
            newMeshPoint.time = endTime;

            if (lastControlMeshIterator->time < endTime){
                if ((lastControlMeshIterator->time + m_pimpl->minStepSize) > endTime){ //if last control mesh point is too close to the end, remove it and place a state mesh point at the end
                    m_pimpl->setIgnoredMesh(lastControlMeshIterator);
                    controlMeshes--;
                }
                m_pimpl->addMeshPoint(newMeshPoint);  //add a mesh point at the end;
            }
            return controlMeshes;
        }

        bool MultipleShootingTranscription::preliminaryChecks()
        {
            if (!(m_pimpl->ocproblem)){
                reportError("MultipleShootingTranscription", "prepare",
                            "Optimal control problem not set.");
                return false;
            }

            if (!(m_pimpl->integrator)) {
                reportError("MultipleShootingTranscription", "prepare",
                            "Integrator not set.");
                return false;
            }

            if ((m_pimpl->ocproblem->finalTime() - m_pimpl->ocproblem->initialTime()) < m_pimpl->minStepSize){
                reportError("MultipleShootingTranscription", "prepare",
                            "The time horizon defined in the OptimalControlProblem is smaller than the minimum step size.");
                return false;
            }

            if (m_pimpl->controlPeriod < m_pimpl->minStepSize){
                reportError("MultipleShootingTranscription", "prepare", "The control period cannot be lower than the minimum step size.");
                return false;
            }

            if ((m_pimpl->controlPeriod >= m_pimpl->maxStepSize) && (m_pimpl->controlPeriod <= (2 * m_pimpl->minStepSize))){
                reportError("MultipleShootingTranscription", "prepare",
                            "Cannot add control mesh points when the controller period is in the range [dTMax, 2*dTmin]."); // the first mesh point is a control mesh. The second mesh than would be too far from the first but not far enough to put a state mesh point in the middle.
                return false;
            }

            if (!(m_pimpl->prepared)){
                if (m_pimpl->ocproblem->dynamicalSystem().expired()){
                    if (m_pimpl->integrator->dynamicalSystem().expired()){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "No dynamical system set, neither to the OptimalControlProblem nor to the Integrator object.");
                        return false;
                    }
                    if (!(m_pimpl->ocproblem->setDynamicalSystemConstraint(m_pimpl->integrator->dynamicalSystem().lock()))){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "Error while setting the dynamicalSystem to the OptimalControlProblem using the one pointer provided by the Integrator object.");
                        return false;
                    }
                } else {
                    if (!(m_pimpl->integrator->dynamicalSystem().expired()) &&
                            (m_pimpl->integrator->dynamicalSystem().lock() != m_pimpl->ocproblem->dynamicalSystem().lock())){
                        reportError("MultipleShootingTranscription", "prepare",
                                    "The selected OptimalControlProblem and the Integrator point to two different dynamical systems.");
                        return false;
                    } else if (m_pimpl->integrator->dynamicalSystem().expired()) {
                        if (!(m_pimpl->integrator->setDynamicalSystem(m_pimpl->ocproblem->dynamicalSystem().lock()))){
                            reportError("MultipleShootingTranscription", "prepare",
                                        "Error while setting the dynamicalSystem to the Integrator using the one pointer provided by the OptimalControlProblem object.");
                            return false;
                        }
                    }
                }
            }

            if (!(m_pimpl->integrator->setMaximumStepSize(m_pimpl->maxStepSize))){
                reportError("MultipleShootingTranscription", "prepare","Error while setting the maximum step size to the integrator.");
                return false;
            }

            return true;
        }


        bool MultipleShootingTranscription::setMeshPoints()
        {
            double endTime = m_pimpl->ocproblem->finalTime(), initTime = m_pimpl->ocproblem->initialTime();
            m_pimpl->resetMeshPoints();

            if (m_pimpl->controlPeriod < 2*(m_pimpl->minStepSize)){ //no other mesh point could be inserted otherwise
                size_t controlMeshes = setControlMeshPoints();
                size_t totalMeshes = static_cast<size_t>(m_pimpl->meshPointsEnd - m_pimpl->meshPoints.begin());

                if (m_pimpl->prepared){
                    if ((m_pimpl->totalMeshes != totalMeshes) || (m_pimpl->controlMeshes != controlMeshes)){
                        reportWarning("MultipleShootingSolver", "setMeshPoints", "Unable to keep number of variables constant.");
                    }
                }

                m_pimpl->controlMeshes = controlMeshes;
                m_pimpl->totalMeshes = totalMeshes;
                return true;
            }

            setControlMeshPoints();

            MeshPoint newMeshPoint;
            //Adding user defined control mesh points
            newMeshPoint.origin = MeshPointOrigin::UserControl();
            newMeshPoint.type = MeshPointType::Control;
            for (size_t i = 0; i < m_pimpl->userControlMeshes.size(); ++i){
               if ((m_pimpl->userControlMeshes[i] > initTime) && (m_pimpl->userControlMeshes[i] < endTime)){
                    newMeshPoint.time = m_pimpl->userControlMeshes[i];
                    m_pimpl->addMeshPoint(newMeshPoint);
               } else {
                   std::ostringstream errorMsg;
                   errorMsg << "Ignored user defined control mesh point at time " << m_pimpl->userControlMeshes[i] << "Out of time range.";
                   reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
               }
            }

            //Adding user defined state mesh points
            newMeshPoint.origin = MeshPointOrigin::UserState();
            newMeshPoint.type = MeshPointType::State;
            for (size_t i = 0; i < m_pimpl->userStateMeshes.size(); ++i){
                if ((m_pimpl->userStateMeshes[i] > initTime) && (m_pimpl->userStateMeshes[i] < endTime)){
                    newMeshPoint.time = m_pimpl->userStateMeshes[i];
                    m_pimpl->addMeshPoint(newMeshPoint);
                } else {
                    std::ostringstream errorMsg;
                    errorMsg << "Ignored user defined state mesh point at time " << m_pimpl->userStateMeshes[i] << "Out of time range.";
                    reportWarning("MultipleShootingSolver", "setMeshPoints", errorMsg.str().c_str());
                }
            }

            std::vector<TimeRange> &constraintsTRs = m_pimpl->ocproblem->getConstraintsTimeRanges();
            std::vector<TimeRange> &costsTRs = m_pimpl->ocproblem->getCostsTimeRanges();

            //adding mesh points for the constraint time ranges
            newMeshPoint.type = MeshPointType::State;
            for (size_t i = 0; i < constraintsTRs.size(); ++i){
                if (constraintsTRs[i].isInstant()) {
                    if (!(constraintsTRs[i].isInRange(initTime)) && !(constraintsTRs[i].isInRange(endTime)) && (constraintsTRs[i].initTime() > initTime)){ //we will have a mesh there in any case
                        newMeshPoint.origin = MeshPointOrigin::Instant();
                        newMeshPoint.time = constraintsTRs[i].initTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                }
                else {
                    newMeshPoint.origin = MeshPointOrigin::TimeRange();
                    if ((constraintsTRs[i].initTime() > initTime) && (constraintsTRs[i].initTime() < endTime)){
                        newMeshPoint.time = constraintsTRs[i].initTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                    if ((constraintsTRs[i].endTime() > initTime) && (constraintsTRs[i].endTime() < endTime)){
                        newMeshPoint.time = constraintsTRs[i].endTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                }
            }


            //adding mesh points for the costs time ranges
            for (size_t i = 0; i < costsTRs.size(); ++i){
                if (costsTRs[i].isInstant()){
                    if (!(costsTRs[i].isInRange(initTime)) && !(costsTRs[i].isInRange(endTime)) && (costsTRs[i].initTime() > initTime)) { //we will have a mesh there in any case
                        newMeshPoint.origin = MeshPointOrigin::Instant();
                        newMeshPoint.time = costsTRs[i].initTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                }
                else {
                    newMeshPoint.origin = MeshPointOrigin::TimeRange();
                    if ((costsTRs[i].initTime() > initTime) && (costsTRs[i].initTime() < endTime)){
                        newMeshPoint.time = costsTRs[i].initTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                    if ((costsTRs[i].endTime() > initTime) && (costsTRs[i].endTime() < endTime)){
                        newMeshPoint.time = costsTRs[i].endTime();
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                }
            }

            m_pimpl->cleanLeftoverMeshes(); //set to ignored the leftover meshes, i.e. those that were set in a previous call

            double tMin = m_pimpl->minStepSize;
            std::sort(m_pimpl->meshPoints.begin(), m_pimpl->meshPointsEnd,
                      [tMin](const MeshPoint&a, const MeshPoint&b) {
                            if (std::abs(b.time - a.time) < tMin)
                                return a.origin.priority() < b.origin.priority();
                            else return a.time < b.time;}); //reorder the vector. Using this ordering, by scrolling the vector, in case of two narrow points, I will get the more important first

            //Now I need to prune the vector

            std::vector<MeshPoint>::iterator mesh = m_pimpl->meshPoints.begin();
            std::vector<MeshPoint>::iterator nextMesh = mesh;
            MeshPointOrigin last = MeshPointOrigin::LastPoint();
            MeshPointOrigin ignored = MeshPointOrigin::Ignored();
            MeshPointOrigin timeRangeOrigin = MeshPointOrigin::TimeRange();

            newMeshPoint.origin = MeshPointOrigin::CompensateLongPeriod();
            newMeshPoint.type = MeshPointType::State;

            double timeDistance;
            while (mesh->origin != last){

                nextMesh = m_pimpl->findNextMeshPoint(mesh); //find next valid mesh
                timeDistance = std::abs(nextMesh->time - mesh->time); //for the way I have ordered the vector, it can be negative

                if (timeDistance > m_pimpl->maxStepSize){ //two consecutive points are too distant
                    unsigned int additionalPoints = static_cast<unsigned int>(std::ceil(timeDistance/(m_pimpl->maxStepSize))) - 1;
                    double dtAdd = timeDistance / (additionalPoints + 1); //additionalPoints + 1 is the number of segments.
                    long nextPosition = nextMesh - m_pimpl->meshPoints.begin();
                    //since tmin < tmax/2, dtAdd > tmin. Infact, the worst case is when timeDistance is nearly equal to tmax.
                    for (unsigned int i = 0; i < additionalPoints; ++i) {
                        newMeshPoint.time = mesh->time + (i + 1)*dtAdd;
                        m_pimpl->addMeshPoint(newMeshPoint);
                    }
                    nextMesh = m_pimpl->meshPoints.begin() + nextPosition;
                    mesh = nextMesh;

                } else if (timeDistance < m_pimpl->minStepSize){ //too consecutive points are too close

                    if (nextMesh->origin < mesh->origin){ //check who has the higher priority
                        m_pimpl->priorityWarning(nextMesh, timeRangeOrigin);
                        m_pimpl->setIgnoredMesh(nextMesh);
                    } else { //actually, because of the ordering, this case should never be reached.
                        m_pimpl->priorityWarning(mesh, timeRangeOrigin);
                        m_pimpl->setIgnoredMesh(mesh);
                        do {
                            --mesh;
                        } while (mesh->origin == ignored);
                        nextMesh = mesh;
                    }
                } else {
                    mesh = nextMesh;
                }
            }

            std::sort(m_pimpl->meshPoints.begin(), m_pimpl->meshPointsEnd,
                      [](const MeshPoint&a, const MeshPoint&b) {return a.time < b.time;}); //reorder the vector

            size_t totalMeshes = 0, controlMeshes = 0;
            mesh = m_pimpl->meshPoints.begin();

            while (mesh->origin != last){ //count meshes
                if (mesh->type == MeshPointType::Control){
                    controlMeshes++;
                }
                totalMeshes++;
                ++mesh;
            }
            totalMeshes++; //the last mesh
            m_pimpl->meshPointsEnd = m_pimpl->meshPoints.begin() + static_cast<long>(totalMeshes);
            assert((m_pimpl->meshPointsEnd - 1)->origin == last);

            if (m_pimpl->prepared){
                if (m_pimpl->controlMeshes == controlMeshes){

                    if (m_pimpl->totalMeshes != totalMeshes){

                        if (m_pimpl->totalMeshes < totalMeshes){ //here I need to remove meshes

                            size_t toBeRemoved = totalMeshes - m_pimpl->totalMeshes;
                            mesh = m_pimpl->meshPoints.begin();
                            nextMesh = mesh;
                            std::vector<MeshPoint>::iterator nextNextMesh = mesh;
                            while ((mesh->origin != last) && (toBeRemoved > 0)){
                                nextMesh = m_pimpl->findNextMeshPoint(mesh); //find next valid mesh

                                if (nextMesh->origin != last) {
                                    nextNextMesh = m_pimpl->findNextMeshPoint(nextMesh);
                                    timeDistance = std::abs(nextNextMesh->time - mesh->time);

                                    if ((timeDistance < m_pimpl->maxStepSize) && !(timeRangeOrigin < nextMesh->origin)){ //I can remove nextMesh
                                        m_pimpl->setIgnoredMesh(nextMesh);
                                        toBeRemoved--;
                                        totalMeshes--;
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

                        } else if (m_pimpl->totalMeshes > totalMeshes){ //here I need to add meshes
                            unsigned int toBeAdded = static_cast<unsigned int>(m_pimpl->totalMeshes - totalMeshes);
                            mesh = m_pimpl->meshPoints.begin();
                            nextMesh = mesh;
                            newMeshPoint.origin = MeshPointOrigin::FillVariables();
                            newMeshPoint.type = MeshPointType::State;
                            while ((mesh->origin != last) && (toBeAdded > 0)){
                                nextMesh = m_pimpl->findNextMeshPoint(mesh); //find next valid mesh
                                timeDistance = std::abs(nextMesh->time - mesh->time);

                                if (timeDistance > (2*m_pimpl->minStepSize)){
                                    unsigned int possibleMeshes = static_cast<unsigned int>(std::ceil(timeDistance/(m_pimpl->minStepSize))) - 1;
                                    unsigned int meshToAddHere = std::min(toBeAdded, possibleMeshes);
                                    double dT = timeDistance/(meshToAddHere + 1);
                                    long nextPosition = nextMesh - m_pimpl->meshPoints.begin();
                                    for(unsigned int m = 1; m <= meshToAddHere; m++){
                                        newMeshPoint.time = mesh->time + m*dT;
                                        m_pimpl->addMeshPoint(newMeshPoint);
                                        totalMeshes++;
                                    }
                                    nextMesh = m_pimpl->meshPoints.begin() + nextPosition;
                                }
                                mesh = nextMesh;
                            }

                            if (toBeAdded > 0){
                                reportWarning("MultipleShootingSolver", "setMeshPoints",
                                              "Unable to keep a constant number of variables. Unable to add enough mesh points.");
                            }

                        }
                        std::sort(m_pimpl->meshPoints.begin(), m_pimpl->meshPointsEnd,
                                  [](const MeshPoint&a, const MeshPoint&b) {return a.time < b.time;}); //reorder the vector
                        m_pimpl->meshPointsEnd = m_pimpl->meshPoints.begin() + static_cast<long>(totalMeshes);
                        assert((m_pimpl->meshPointsEnd - 1)->origin == last);
                    }
                } else {
                    reportWarning("MultipleShootingSolver", "setMeshPoints",
                                  "Unable to keep a constant number of variables. The control meshes are different");
                }
            }

            m_pimpl->totalMeshes = totalMeshes;
            m_pimpl->controlMeshes = controlMeshes;

            return true;
        }

        MultipleShootingTranscription::~MultipleShootingTranscription()
        {
            if (m_pimpl){
                delete m_pimpl;
                m_pimpl = nullptr;
            }
        }

        bool MultipleShootingTranscription::setOptimalControlProblem(const std::shared_ptr<OptimalControlProblem> problem)
        {
            if (m_pimpl->ocproblem){
                reportError("MultipleShootingSolver", "setOptimalControlProblem", "The OptimalControlProblem for this instance has already been set.");
                return false;
            }

            m_pimpl->ocproblem = problem;
            return true;
        }

        bool MultipleShootingTranscription::setIntegrator(const std::shared_ptr<Integrator> integrationMethod)
        {
            if (m_pimpl->integrator){
                reportError("MultipleShootingSolver", "setIntegrator", "The integration method for this instance has already been set.");
                return false;
            }

            m_pimpl->integrator = integrationMethod;
            return true;
        }

        bool MultipleShootingTranscription::setStepSizeBounds(const double minStepSize, const double maxStepsize)
        {
            if (minStepSize <= 0){
                reportError("MultipleShootingSolver", "setStepSizeBounds","The minimum step size is expected to be positive.");
                return false;
            }

            if (maxStepsize <= (2 * minStepSize)){
                reportError("MultipleShootingSolver", "setStepSizeBounds","The maximum step size is expected to be greater than twice the minimum."); //imagine to have a distance between two mesh points slightly greater than the max. It would be impossible to add a mesh point in the middle
                return false;
            }

            m_pimpl->minStepSize = minStepSize;
            m_pimpl->maxStepSize = maxStepsize;

            return true;
        }

        bool MultipleShootingTranscription::setControlPeriod(double period)
        {
            if (period <= 0){
                reportError("MultipleShootingSolver", "prepare", "The control period is supposed to be positive.");
                return false;
            }
            m_pimpl->controlPeriod = period;
            return true;
        }

        bool MultipleShootingTranscription::setAdditionalStateMeshPoints(const std::vector<double> &stateMeshes)
        {
            m_pimpl->userStateMeshes = stateMeshes;
            return true;
        }

        bool MultipleShootingTranscription::setAdditionalControlMeshPoints(const std::vector<double> &controlMeshes)
        {
            m_pimpl->userControlMeshes = controlMeshes;
            return true;
        }

        void MultipleShootingTranscription::setPlusInfinity(double plusInfinity)
        {
            assert(plusInfinity > 0);
            m_pimpl->plusInfinity = plusInfinity;
        }

        void MultipleShootingTranscription::setMinusInfinity(double minusInfinity)
        {
            assert(minusInfinity < 0);
            m_pimpl->minusInfinity = minusInfinity;
        }

        bool MultipleShootingTranscription::setInitialState(const VectorDynSize &initialState)
        {
            if (!(m_pimpl->ocproblem)){
                reportError("MultipleShootingTranscription", "setInitialState", "The optimal control problem pointer is empty.");
                return false;
            }

            if (m_pimpl->ocproblem->dynamicalSystem().expired()){
                reportError("MultipleShootingTranscription", "setInitialState", "The optimal control problem does not point to any dynamical system.");
                return false;
            }

            if (!(m_pimpl->ocproblem->dynamicalSystem().lock()->setInitialState(initialState))){
                reportError("MultipleShootingTranscription", "setInitialState", "Error while setting the initial state to the dynamical system.");
                return false;
            }

            return true;
        }

        bool MultipleShootingTranscription::getTimings(std::vector<double> &stateEvaluations, std::vector<double> &controlEvaluations)
        {
            if (!(m_pimpl->prepared)){
                reportWarning("MultipleShootingTranscription", "getTimings", "The method solve was not called yet. Computing new mesh points. These may be overwritten when calling the solve method.");

                if (!preliminaryChecks())
                    return false;

                if (!setMeshPoints()){
                    return false;
                }
            }

            if (stateEvaluations.size() != (m_pimpl->totalMeshes - 1))
                stateEvaluations.resize(m_pimpl->totalMeshes - 1);

            if (controlEvaluations.size() != m_pimpl->controlMeshes)
                controlEvaluations.resize(m_pimpl->controlMeshes);

            size_t stateIndex = 0, controlIndex = 0;

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();

            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
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

        bool MultipleShootingTranscription::getSolution(std::vector<VectorDynSize> &states, std::vector<VectorDynSize> &controls)
        {
            if (!(m_pimpl->solved)){
                reportError("MultipleShootingTranscription", "getSolution", "First you need to solve the problem once.");
                return false;
            }

            if (states.size() != (m_pimpl->totalMeshes - 1))
                states.resize((m_pimpl->totalMeshes - 1));

            if (controls.size() != m_pimpl->controlMeshes)
                controls.resize(m_pimpl->controlMeshes);

            Eigen::Map<Eigen::VectorXd> solutionMap = toEigen(m_pimpl->solution);

            size_t stateIndex = 0, controlIndex = 0;

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();

            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin != first){
                    if (states[stateIndex].size() !=  m_pimpl->nx)
                        states[stateIndex].resize(m_pimpl->nx);

                    toEigen(states[stateIndex]) = solutionMap.segment(mesh->stateIndex, m_pimpl->nx);
                    stateIndex++;
                }
                if (mesh->type == MeshPointType::Control){
                    if (controls[controlIndex].size() !=  m_pimpl->nu)
                        controls[controlIndex].resize(m_pimpl->nu);

                    toEigen(controls[controlIndex]) = solutionMap.segment(mesh->controlIndex, m_pimpl->nu);
                    controlIndex++;
                }
            }

            return true;
        }


        bool MultipleShootingTranscription::prepare()
        {

            if (!preliminaryChecks())
                return false;


            if (!setMeshPoints()){
                return false;
            }

            size_t nx = m_pimpl->integrator->dynamicalSystem().lock()->stateSpaceSize();
            m_pimpl->nx = nx;

            size_t nu = m_pimpl->integrator->dynamicalSystem().lock()->controlSpaceSize();
            m_pimpl->nu = nu;

            //TODO: I should consider also the possibility to have auxiliary variables in the integrator
            m_pimpl->numberOfVariables = (m_pimpl->totalMeshes - 1) * nx + m_pimpl->controlMeshes * nu; //the -1 removes the initial state from the set of optimization varibales

            m_pimpl->constraintsPerInstant = m_pimpl->ocproblem->getConstraintsDimension();
            size_t nc = m_pimpl->constraintsPerInstant;
            m_pimpl->numberOfConstraints = (m_pimpl->totalMeshes - 1) * nx + (m_pimpl->constraintsPerInstant) * (m_pimpl->totalMeshes); //dynamical constraints (removing the initial state) and normal constraints

            m_pimpl->allocateBuffers();

            Eigen::Map<Eigen::VectorXd> lowerBoundMap = toEigen(m_pimpl->constraintsLowerBound);
            Eigen::Map<Eigen::VectorXd> upperBoundMap = toEigen(m_pimpl->constraintsUpperBound);

            m_pimpl->resetNonZerosCount();

            std::vector<MeshPoint>::iterator mesh = m_pimpl->meshPoints.begin(), previousControlMesh = mesh;
            size_t index = 0, constraintIndex = 0;
            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            while (mesh != m_pimpl->meshPointsEnd){
                if (mesh->origin == first){
                    //setting up the indeces
                    mesh->controlIndex = index;
                    mesh->previousControlIndex = index;
                    index += nu;
                    previousControlMesh = mesh;

                    //Saving constraints bounds
                    if (!(m_pimpl->ocproblem->getConstraintsLowerBound(mesh->time, m_pimpl->minusInfinity, m_pimpl->constraintsBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating constraints lower bounds at time " << mesh->time << ".";
                        reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                    }
                    lowerBoundMap.segment(constraintIndex, nc) = toEigen(m_pimpl->constraintsBuffer);

                    if (!(m_pimpl->ocproblem->getConstraintsUpperBound(mesh->time, m_pimpl->plusInfinity, m_pimpl->constraintsBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating constraints upper bounds at time " << mesh->time << ".";
                        reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                    }
                    upperBoundMap.segment(constraintIndex, nc) = toEigen(m_pimpl->constraintsBuffer);

                    //Saving the jacobian structure due to the constraints
                    m_pimpl->addJacobianBlock(constraintIndex, nc, mesh->controlIndex, nu);
                    constraintIndex += nc;

                    //Saving the hessian structure
                    m_pimpl->addHessianBlock(mesh->controlIndex, nu, mesh->controlIndex, nu); //assume that a cost/constraint depends on the square of u

                } else if (mesh->type == MeshPointType::Control) {
                    mesh->previousControlIndex = previousControlMesh->controlIndex;
                    mesh->controlIndex = index;
                    index += nu;
                    mesh->stateIndex = index;
                    index += nx;
                    previousControlMesh = mesh;

                    //Saving dynamical constraints bounds
                    lowerBoundMap.segment(constraintIndex, nx).setZero();
                    upperBoundMap.segment(constraintIndex, nx).setZero();

                    //Saving the jacobian structure due to the dynamical constraints
                    m_pimpl->addJacobianBlock(constraintIndex, nx, mesh->previousControlIndex, nu);
                    m_pimpl->addJacobianBlock(constraintIndex, nx, mesh->controlIndex, nu);
                    m_pimpl->addJacobianBlock(constraintIndex, nx, mesh->stateIndex, nx);
                    if ((mesh - 1)->origin != first){
                        m_pimpl->addJacobianBlock(constraintIndex, nx, (mesh - 1)->stateIndex, nx);
                    }
                    constraintIndex += nx;

                    //Saving constraints bounds
                    if (!(m_pimpl->ocproblem->getConstraintsLowerBound(mesh->time, m_pimpl->minusInfinity, m_pimpl->constraintsBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating constraints lower bounds at time " << mesh->time << ".";
                        reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                    }
                    lowerBoundMap.segment(constraintIndex, nc) = toEigen(m_pimpl->constraintsBuffer);

                    if (!(m_pimpl->ocproblem->getConstraintsUpperBound(mesh->time, m_pimpl->plusInfinity, m_pimpl->constraintsBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating constraints upper bounds at time " << mesh->time << ".";
                        reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                    }
                    upperBoundMap.segment(constraintIndex, nc) = toEigen(m_pimpl->constraintsBuffer);

                    //Saving the jacobian structure due to the constraints
                    m_pimpl->addJacobianBlock(constraintIndex, nc, mesh->stateIndex, nx);
                    m_pimpl->addJacobianBlock(constraintIndex, nc, mesh->controlIndex, nu);
                    constraintIndex += nc;

                    //Saving the hessian structure
                    m_pimpl->addHessianBlock(mesh->controlIndex, nu, mesh->controlIndex, nu); //assume that a cost/constraint depends on the square of u
                    m_pimpl->addHessianBlock(mesh->stateIndex, nx, mesh->stateIndex, nx); //assume that a cost/constraint depends on the square of x

                    m_pimpl->addHessianBlock(mesh->controlIndex, nu, mesh->stateIndex, nx); //assume that a cost/constraint depends on the product of x-u
                    m_pimpl->addHessianBlock(mesh->stateIndex, nx, mesh->controlIndex, nu);

                    m_pimpl->addHessianBlock(mesh->previousControlIndex, nu, mesh->stateIndex, nx); //assume that due to the dynamics we have a cross relation between x and u-1
                    m_pimpl->addHessianBlock(mesh->stateIndex, nx, mesh->previousControlIndex, nu);

                    if ((mesh - 1)->origin != first){
                        m_pimpl->addHessianBlock((mesh - 1)->stateIndex, nx, mesh->stateIndex, nx); //assume that due to the dynamics we have a cross relation between x and x-1
                        m_pimpl->addHessianBlock(mesh->stateIndex, nx, (mesh - 1)->stateIndex, nx);
                    }


                } else if (mesh->type == MeshPointType::State) {
                    mesh->controlIndex = previousControlMesh->controlIndex;
                    mesh->previousControlIndex = previousControlMesh->controlIndex;
                    mesh->stateIndex = index;
                    index += nx;

                    //Saving dynamical constraints bounds
                    lowerBoundMap.segment(constraintIndex, nx).setZero();
                    upperBoundMap.segment(constraintIndex, nx).setZero();

                    //Saving the jacobian structure due to the dynamical constraints
                    m_pimpl->addJacobianBlock(constraintIndex, nx, mesh->controlIndex, nu);
                    m_pimpl->addJacobianBlock(constraintIndex, nx, mesh->stateIndex, nx);
                    if ((mesh - 1)->origin != first){
                        m_pimpl->addJacobianBlock(constraintIndex, nx, (mesh - 1)->stateIndex, nx);
                    }
                    constraintIndex += nx;

                    //Saving constraints bounds
                    if (!(m_pimpl->ocproblem->getConstraintsLowerBound(mesh->time, m_pimpl->minusInfinity, m_pimpl->constraintsBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating constraints lower bounds at time " << mesh->time << ".";
                        reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                    }
                    lowerBoundMap.segment(constraintIndex, nc) = toEigen(m_pimpl->constraintsBuffer);

                    if (!(m_pimpl->ocproblem->getConstraintsUpperBound(mesh->time, m_pimpl->plusInfinity, m_pimpl->constraintsBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating constraints upper bounds at time " << mesh->time << ".";
                        reportError("MultipleShootingSolver", "prepare", errorMsg.str().c_str());
                    }
                    upperBoundMap.segment(constraintIndex, nc) = toEigen(m_pimpl->constraintsBuffer);

                    m_pimpl->addJacobianBlock(constraintIndex, nc, mesh->stateIndex, nx);
                    m_pimpl->addJacobianBlock(constraintIndex, nc, mesh->controlIndex, nu);
                    constraintIndex += nc;

                    //Saving the hessian structure
                    m_pimpl->addHessianBlock(mesh->controlIndex, nu, mesh->controlIndex, nu); //assume that a cost/constraint depends on the square of u
                    m_pimpl->addHessianBlock(mesh->stateIndex, nx, mesh->stateIndex, nx); //assume that a cost/constraint depends on the square of x

                    m_pimpl->addHessianBlock(mesh->controlIndex, nu, mesh->stateIndex, nx); //assume that a cost/constraint depends on the product of x-u
                    m_pimpl->addHessianBlock(mesh->stateIndex, nx, mesh->controlIndex, nu);

                    if ((mesh - 1)->origin != first){
                        m_pimpl->addHessianBlock((mesh - 1)->stateIndex, nx, mesh->stateIndex, nx); //assume that due to the dynamics we have a cross relation between x and x-1
                        m_pimpl->addHessianBlock(mesh->stateIndex, nx, (mesh - 1)->stateIndex, nx);
                    }

                }
                ++mesh;
            }
            assert(index == m_pimpl->numberOfVariables);
            assert(constraintIndex == m_pimpl->numberOfConstraints);

            m_pimpl->prepared = true;
            return true;
        }

        void MultipleShootingTranscription::reset()
        {
            m_pimpl->prepared = false;
            m_pimpl->totalMeshes = 0;
            m_pimpl->controlMeshes = 0;
            m_pimpl->numberOfVariables = 0;
            m_pimpl->resetNonZerosCount();
            m_pimpl->resetMeshPoints();
        }

        unsigned int MultipleShootingTranscription::numberOfVariables()
        {
            return static_cast<unsigned int>(m_pimpl->numberOfVariables);
        }

        unsigned int MultipleShootingTranscription::numberOfConstraints()
        {
            return static_cast<unsigned int>(m_pimpl->numberOfConstraints);
        }

        bool MultipleShootingTranscription::getConstraintsBounds(VectorDynSize &constraintsLowerBounds, VectorDynSize &constraintsUpperBounds)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "getConstraintsInfo", "First you need to call the prepare method");
                return false;
            }

            if (constraintsLowerBounds.size() != numberOfConstraints())
                constraintsLowerBounds.resize(numberOfConstraints());

            if (constraintsUpperBounds.size() != numberOfConstraints())
                constraintsUpperBounds.resize(numberOfConstraints());

            constraintsLowerBounds = m_pimpl->constraintsLowerBound;
            constraintsUpperBounds = m_pimpl->constraintsUpperBound;

            return true;
        }

        bool MultipleShootingTranscription::getVariablesUpperBound(VectorDynSize &variablesUpperBound)
        {
            bool stateBounded = true, controlBounded = true;

            Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_pimpl->stateBuffer);
            Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_pimpl->controlBuffer);

            if (!(m_pimpl->ocproblem->getStateUpperBound(m_pimpl->stateBuffer))) {
                stateBounded = false;
                stateBufferMap.setConstant(m_pimpl->plusInfinity);
            }

            if (!(m_pimpl->ocproblem->getControlUpperBound(m_pimpl->controlBuffer))) {
                controlBounded = false;
                controlBufferMap.setConstant(m_pimpl->plusInfinity);
            }

            if (!controlBounded && !stateBounded)
                return false;

            if (variablesUpperBound.size() != m_pimpl->numberOfVariables)
                variablesUpperBound.resize(m_pimpl->numberOfVariables);
            Eigen::Map<Eigen::VectorXd> upperBoundMap = toEigen(variablesUpperBound);

            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                    upperBoundMap.segment(mesh->controlIndex, nu) = controlBufferMap;
                } else if (mesh->type == MeshPointType::Control) {
                    upperBoundMap.segment(mesh->controlIndex, nu) = controlBufferMap;
                    upperBoundMap.segment(mesh->stateIndex, nx) = stateBufferMap;
                } else if (mesh->type == MeshPointType::State) {
                    upperBoundMap.segment(mesh->stateIndex, nx) = stateBufferMap;
                }
            }
            return true;
        }

        bool MultipleShootingTranscription::getVariablesLowerBound(VectorDynSize &variablesLowerBound)
        {
            bool stateBounded = true, controlBounded = true;

            Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_pimpl->stateBuffer);
            Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_pimpl->controlBuffer);

            if (!(m_pimpl->ocproblem->getStateLowerBound(m_pimpl->stateBuffer))) {
                stateBounded = false;
                stateBufferMap.setConstant(m_pimpl->minusInfinity);
            }

            if (!(m_pimpl->ocproblem->getControlLowerBound(m_pimpl->controlBuffer))) {
                controlBounded = false;
                controlBufferMap.setConstant(m_pimpl->minusInfinity);
            }

            if (!controlBounded && !stateBounded)
                return false;

            if (variablesLowerBound.size() != m_pimpl->numberOfVariables)
                variablesLowerBound.resize(m_pimpl->numberOfVariables);
            Eigen::Map<Eigen::VectorXd> lowerBoundMap = toEigen(variablesLowerBound);

            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                    lowerBoundMap.segment(mesh->controlIndex, nu) = controlBufferMap;
                } else if (mesh->type == MeshPointType::Control) {
                    lowerBoundMap.segment(mesh->controlIndex, nu) = controlBufferMap;
                    lowerBoundMap.segment(mesh->stateIndex, nx) = stateBufferMap;
                } else if (mesh->type == MeshPointType::State) {
                    lowerBoundMap.segment(mesh->stateIndex, nx) = stateBufferMap;
                }
            }
            return true;
        }

        bool MultipleShootingTranscription::getConstraintsJacobianInfo(std::vector<size_t> &nonZeroElementRows, std::vector<size_t> &nonZeroElementColumns)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "getConstraintsInfo", "First you need to call the prepare method");
                return false;
            }

            if (nonZeroElementRows.size() != m_pimpl->jacobianNonZeros)
                nonZeroElementRows.resize(static_cast<unsigned int>(m_pimpl->jacobianNonZeros));

            if (nonZeroElementColumns.size() != m_pimpl->jacobianNonZeros)
                nonZeroElementColumns.resize(static_cast<unsigned int>(m_pimpl->jacobianNonZeros));


            for (unsigned int i = 0; i < m_pimpl->jacobianNonZeros; ++i){
                nonZeroElementRows[i] = m_pimpl->jacobianNZRows[i];
                nonZeroElementColumns[i] = m_pimpl->jacobianNZCols[i];
            }

            return true;
        }

        bool MultipleShootingTranscription::getHessianInfo(std::vector<size_t> &nonZeroElementRows, std::vector<size_t> &nonZeroElementColumns)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "getHessianInfo", "First you need to call the prepare method");
                return false;
            }

            if (nonZeroElementRows.size() != m_pimpl->hessianNonZeros)
                nonZeroElementRows.resize(static_cast<unsigned int>(m_pimpl->hessianNonZeros));

            if (nonZeroElementColumns.size() != m_pimpl->hessianNonZeros)
                nonZeroElementColumns.resize(static_cast<unsigned int>(m_pimpl->hessianNonZeros));

            for (unsigned int i = 0; i < m_pimpl->hessianNonZeros; ++i){
                nonZeroElementRows[i] = m_pimpl->hessianNZRows[i];
                nonZeroElementColumns[i] = m_pimpl->hessianNZCols[i];
            }

            return true;
        }

        bool MultipleShootingTranscription::setVariables(const VectorDynSize &variables)
        {
            if (variables.size() != m_pimpl->variablesBuffer.size()){
                reportError("MultipleShootingTranscription", "setVariables", "The input variables have a size different from the expected one.");
                return false;
            }

            m_pimpl->variablesBuffer = variables;
            return true;
        }

        bool MultipleShootingTranscription::evaluateCostFunction(double &costValue)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "evaluateCostFunction", "First you need to call the prepare method");
                return false;
            }

            Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_pimpl->stateBuffer);
            Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_pimpl->controlBuffer);
            Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_pimpl->variablesBuffer);

            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);

            costValue = 0;
            double singleCost;

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                    stateBufferMap = toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                } else {
                    stateBufferMap = variablesBuffer.segment(mesh->stateIndex, nx);
                }
                controlBufferMap = variablesBuffer.segment(mesh->controlIndex, nu);

                if (!(m_pimpl->ocproblem->costsEvaluation(mesh->time, m_pimpl->stateBuffer, m_pimpl->controlBuffer, singleCost))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating cost at time t = " << mesh->time << ".";
                    reportError("MultipleShootingTranscription", "evaluateCostFunction", errorMsg.str().c_str());
                    return false;
                }

                costValue += singleCost;

            }
            return true;

        }

        bool MultipleShootingTranscription::evaluateCostGradient(VectorDynSize &gradient)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "evaluateCostGradient", "First you need to call the prepare method");
                return false;
            }

            Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_pimpl->stateBuffer);
            Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_pimpl->controlBuffer);
            Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_pimpl->variablesBuffer);
            Eigen::Map<Eigen::VectorXd> costStateGradient = toEigen(m_pimpl->costStateGradientBuffer);
            Eigen::Map<Eigen::VectorXd> costControlGradient = toEigen(m_pimpl->costControlGradientBuffer);

            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);

            if (gradient.size() != m_pimpl->numberOfVariables)
                gradient.resize(static_cast<unsigned int>(m_pimpl->numberOfVariables));

            Eigen::Map<Eigen::VectorXd> gradientMap = toEigen(gradient);

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                    stateBufferMap = toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                } else {
                    stateBufferMap = variablesBuffer.segment(mesh->stateIndex, nx);
                }
                controlBufferMap = variablesBuffer.segment(mesh->controlIndex, nu);

                if (mesh->origin != first){
                    if (!(m_pimpl->ocproblem->costsFirstPartialDerivativeWRTState(mesh->time, m_pimpl->stateBuffer, m_pimpl->controlBuffer, m_pimpl->costStateGradientBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost state gradient at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostGradient", errorMsg.str().c_str());
                        return false;
                    }

                    gradientMap.segment(mesh->stateIndex, nx) = costStateGradient;
                }

                if (!(m_pimpl->ocproblem->costsFirstPartialDerivativeWRTControl(mesh->time, m_pimpl->stateBuffer, m_pimpl->controlBuffer, m_pimpl->costControlGradientBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating cost control gradient at time t = " << mesh->time << ".";
                    reportError("MultipleShootingTranscription", "evaluateCostGradient", errorMsg.str().c_str());
                    return false;
                }

                if (mesh->type == MeshPointType::Control){
                    gradientMap.segment(mesh->controlIndex, nu) = costControlGradient;
                } else if (mesh->type == MeshPointType::State) {
                    gradientMap.segment(mesh->controlIndex, nu) += costControlGradient;
                }

            }

            return true;
        }

        bool MultipleShootingTranscription::evaluateCostHessian(MatrixDynSize &hessian)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "evaluateCostHessian", "First you need to call the prepare method");
                return false;
            }

            Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_pimpl->stateBuffer);
            Eigen::Map<Eigen::VectorXd> controlBufferMap = toEigen(m_pimpl->controlBuffer);
            Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_pimpl->variablesBuffer);
            iDynTreeEigenMatrixMap costStateHessian = toEigen(m_pimpl->costHessianStateBuffer);
            iDynTreeEigenMatrixMap costControlHessian = toEigen(m_pimpl->costHessianControlBuffer);
            iDynTreeEigenMatrixMap costStateControlHessian = toEigen(m_pimpl->costHessianStateControlBuffer);
            iDynTreeEigenMatrixMap costControlStateHessian = toEigen(m_pimpl->costHessianControlStateBuffer);


            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);

            if ((hessian.rows() != m_pimpl->numberOfVariables) || (hessian.cols() != m_pimpl->numberOfVariables))
                hessian.resize(static_cast<unsigned int>(m_pimpl->numberOfVariables), static_cast<unsigned int>(m_pimpl->numberOfVariables));

            iDynTreeEigenMatrixMap hessianMap = toEigen(hessian);

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                    stateBufferMap = toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                } else {
                    stateBufferMap = variablesBuffer.segment(mesh->stateIndex, nx);
                }
                controlBufferMap = variablesBuffer.segment(mesh->controlIndex, nu);

                if (mesh->origin != first){
                    if (!(m_pimpl->ocproblem->costsSecondPartialDerivativeWRTState(mesh->time, m_pimpl->stateBuffer, m_pimpl->controlBuffer, m_pimpl->costHessianStateBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost state hessian at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostHessian", errorMsg.str().c_str());
                        return false;
                    }

                    hessianMap.block(mesh->stateIndex, mesh->stateIndex, nx, nx) = costStateHessian;

                    if (!(m_pimpl->ocproblem->costsSecondPartialDerivativeWRTStateControl(mesh->time, m_pimpl->stateBuffer, m_pimpl->controlBuffer, m_pimpl->costHessianStateControlBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating cost state-control hessian at time t = " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateCostHessian", errorMsg.str().c_str());
                        return false;
                    }

                    hessianMap.block(mesh->stateIndex, mesh->controlIndex, nx, nu) = costStateControlHessian;
                    costControlStateHessian = costStateControlHessian.transpose();
                    hessianMap.block(mesh->controlIndex, mesh->stateIndex, nu, nx) = costControlStateHessian;
                }

                if (!(m_pimpl->ocproblem->costsSecondPartialDerivativeWRTControl(mesh->time, m_pimpl->stateBuffer, m_pimpl->controlBuffer, m_pimpl->costHessianControlBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating cost control hessian at time t = " << mesh->time << ".";
                    reportError("MultipleShootingTranscription", "evaluateCostHessian", errorMsg.str().c_str());
                    return false;
                }

                if (mesh->type == MeshPointType::Control){
                    hessianMap.block(mesh->controlIndex, mesh->controlIndex, nu, nu) = costControlHessian;
                } else if (mesh->type == MeshPointType::State) {
                    hessianMap.block(mesh->controlIndex, mesh->controlIndex, nu, nu) += costControlHessian;
                }
            }
            return true;
        }

        bool MultipleShootingTranscription::evaluateConstraints(VectorDynSize &constraints)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "evaluateConstraints", "First you need to call the prepare method");
                return false;
            }

            Eigen::Map<Eigen::VectorXd> stateBufferMap = toEigen(m_pimpl->stateBuffer);
            Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_pimpl->variablesBuffer);
            Eigen::Map<Eigen::VectorXd> constraintsBufferMap = toEigen(m_pimpl->constraintsBuffer);
            Eigen::Map<Eigen::VectorXd> currentState = toEigen(m_pimpl->collocationStateBuffer[1]);
            Eigen::Map<Eigen::VectorXd> previousState = toEigen(m_pimpl->collocationStateBuffer[0]);
            Eigen::Map<Eigen::VectorXd> currentControl = toEigen(m_pimpl->collocationControlBuffer[1]);
            Eigen::Map<Eigen::VectorXd> previousControl = toEigen(m_pimpl->collocationControlBuffer[0]);


            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);
            Eigen::Index nc = static_cast<Eigen::Index>(m_pimpl->constraintsPerInstant);

            if (constraints.size() != m_pimpl->numberOfConstraints)
                constraints.resize(static_cast<unsigned int>(m_pimpl->numberOfConstraints));

            Eigen::Map<Eigen::VectorXd> constraintsMap = toEigen(constraints);


            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            Eigen::Index constraintIndex = 0;
            double dT = 0;
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                     currentState= toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                } else {
                    currentState = variablesBuffer.segment(mesh->stateIndex, nx);
                    if ((mesh -1)->origin == first){
                        previousState = toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                    } else {
                        previousState = variablesBuffer.segment((mesh - 1)->stateIndex, nx);
                    }
                }
                currentControl  = variablesBuffer.segment(mesh->controlIndex, nu);
                previousControl = variablesBuffer.segment(mesh->previousControlIndex, nu);

                if (mesh->origin != first){
                    dT = mesh->time - (mesh - 1)->time;
                    if (!(m_pimpl->integrator->evaluateCollocationConstraint(mesh->time, m_pimpl->collocationStateBuffer, m_pimpl->collocationControlBuffer, dT, m_pimpl->stateBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating the collocation constraint at time " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateConstraints", errorMsg.str().c_str());
                        return false;
                    }
                    constraintsMap.segment(constraintIndex, nx) = stateBufferMap;
                    constraintIndex += nx;
                }

                if (!(m_pimpl->ocproblem->constraintsEvaluation(mesh->time, m_pimpl->collocationStateBuffer[1], m_pimpl->collocationControlBuffer[1], m_pimpl->constraintsBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating the constraints at time " << mesh->time << ".";
                    reportError("MultipleShootingTranscription", "evaluateConstraints", errorMsg.str().c_str());
                    return false;
                }
                constraintsMap.segment(constraintIndex, nc) = constraintsBufferMap;
                constraintIndex += nc;
            }
            assert(static_cast<size_t>(constraintIndex) == m_pimpl->numberOfConstraints);
            return true;
        }

        bool MultipleShootingTranscription::evaluateConstraintsJacobian(MatrixDynSize &jacobian)
        {
            if (!(m_pimpl->prepared)){
                reportError("MultipleShootingTranscription", "evaluateConstraints", "First you need to call the prepare method");
                return false;
            }

            Eigen::Map<Eigen::VectorXd> variablesBuffer = toEigen(m_pimpl->variablesBuffer);
            Eigen::Map<Eigen::VectorXd> currentState = toEigen(m_pimpl->collocationStateBuffer[1]);
            Eigen::Map<Eigen::VectorXd> previousState = toEigen(m_pimpl->collocationStateBuffer[0]);
            Eigen::Map<Eigen::VectorXd> currentControl = toEigen(m_pimpl->collocationControlBuffer[1]);
            Eigen::Map<Eigen::VectorXd> previousControl = toEigen(m_pimpl->collocationControlBuffer[0]);


            Eigen::Index nx = static_cast<Eigen::Index>(m_pimpl->nx);
            Eigen::Index nu = static_cast<Eigen::Index>(m_pimpl->nu);
            Eigen::Index nc = static_cast<Eigen::Index>(m_pimpl->constraintsPerInstant);

            if (jacobian.rows() != m_pimpl->numberOfConstraints || jacobian.cols() != m_pimpl->numberOfVariables)
                jacobian.resize(static_cast<unsigned int>(m_pimpl->numberOfConstraints), static_cast<unsigned int>(m_pimpl->numberOfVariables));

            iDynTreeEigenMatrixMap jacobianMap = toEigen(jacobian);

            MeshPointOrigin first = MeshPointOrigin::FirstPoint();
            Eigen::Index constraintIndex = 0;
            double dT = 0;
            for (auto mesh = m_pimpl->meshPoints.begin(); mesh != m_pimpl->meshPointsEnd; ++mesh){
                if (mesh->origin == first){
                     currentState= toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                } else {
                    currentState = variablesBuffer.segment(mesh->stateIndex, nx);
                    if ((mesh -1)->origin == first){
                        previousState = toEigen(m_pimpl->ocproblem->dynamicalSystem().lock()->initialState());
                    } else {
                        previousState = variablesBuffer.segment((mesh - 1)->stateIndex, nx);
                    }
                }
                currentControl  = variablesBuffer.segment(mesh->controlIndex, nu);
                previousControl = variablesBuffer.segment(mesh->previousControlIndex, nu);

                if (mesh->origin != first){
                    dT = mesh->time - (mesh - 1)->time;
                    if (!(m_pimpl->integrator->evaluateCollocationConstraintJacobian(mesh->time, m_pimpl->collocationStateBuffer, m_pimpl->collocationControlBuffer, dT, m_pimpl->collocationStateJacBuffer, m_pimpl->collocationControlJacBuffer))){
                        std::ostringstream errorMsg;
                        errorMsg << "Error while evaluating the collocation constraint jacobian at time " << mesh->time << ".";
                        reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                        return false;
                    }

                    if ((mesh -1)->origin != first)
                        jacobianMap.block(constraintIndex, (mesh-1)->stateIndex, nx, nx) = toEigen(m_pimpl->collocationStateJacBuffer[0]);

                    jacobianMap.block(constraintIndex, mesh->stateIndex, nx, nx) = toEigen(m_pimpl->collocationStateJacBuffer[1]);

                    jacobianMap.block(constraintIndex, mesh->controlIndex, nx, nu) = toEigen(m_pimpl->collocationControlJacBuffer[1]);

                    if (mesh->type == MeshPointType::Control)
                        jacobianMap.block(constraintIndex, mesh->previousControlIndex, nx, nu) = toEigen(m_pimpl->collocationControlJacBuffer[0]);
                    else if (mesh->type == MeshPointType::State)
                        jacobianMap.block(constraintIndex, mesh->previousControlIndex, nx, nu) += toEigen(m_pimpl->collocationControlJacBuffer[0]); //the previous and the current control coincides
                    constraintIndex += nx;
                }

                if (!(m_pimpl->ocproblem->constraintsJacobianWRTState(mesh->time, m_pimpl->collocationStateBuffer[1], m_pimpl->collocationControlBuffer[1], m_pimpl->constraintsStateJacBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating the constraints state jacobian at time " << mesh->time << ".";
                    reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                    return false;
                }

                if (mesh->origin != first)
                    jacobianMap.block(constraintIndex, mesh->stateIndex, nc, nx) = toEigen(m_pimpl->constraintsStateJacBuffer);

                if (!(m_pimpl->ocproblem->constraintsJacobianWRTControl(mesh->time, m_pimpl->collocationStateBuffer[1], m_pimpl->collocationControlBuffer[1], m_pimpl->constraintsControlJacBuffer))){
                    std::ostringstream errorMsg;
                    errorMsg << "Error while evaluating the constraints control jacobian at time " << mesh->time << ".";
                    reportError("MultipleShootingTranscription", "evaluateConstraintsJacobian", errorMsg.str().c_str());
                    return false;
                }

                jacobianMap.block(constraintIndex, mesh->controlIndex, nc, nu) = toEigen(m_pimpl->constraintsControlJacBuffer);
                constraintIndex += nc;
            }
            assert(static_cast<size_t>(constraintIndex) == m_pimpl->numberOfConstraints);
            return true;
        }

        bool MultipleShootingTranscription::evaluateConstraintsHessian(const VectorDynSize &constraintsMultipliers, MatrixDynSize &hessian)
        {
            if (!(toEigen(constraintsMultipliers).isZero(0))){
                reportWarning("MultipleShootingTranscription", "evaluateConstraintsHessian", "The constraints hessian is currently unavailable.");
            }
            hessian.zero();
            return true;
        }


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

        bool MultipleShootingSolver::getTimings(std::vector<double> &stateEvaluations, std::vector<double> &controlEvaluations)
        {
            return m_transcription->getTimings(stateEvaluations, controlEvaluations);
        }


        bool MultipleShootingSolver::solve()
        {
            if (!m_optimizer){
                reportError("MultipleShootingSolver", "solve", "No optimizer selected.");
                return false;
            }
            if (m_optimizer->solve()){
                if (!(m_optimizer->getPrimalVariables(m_transcription->m_pimpl->solution))){
                    reportError("MultipleShootingSolver", "solve", "Error while retrieving the primal variables from the optimizer.");
                    return false;
                }
            } else {
                reportError("MultipleShootingSolver", "solve", "Error when calling the optimizer solve method.");
                return false;
            }
            m_transcription->m_pimpl->solved = true;
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
    }
}
