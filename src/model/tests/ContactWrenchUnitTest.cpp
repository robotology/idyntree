
#include <iDynTree/ContactWrench.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Model.h>
#include <iDynTree/Link.h>

using namespace iDynTree;


class OneLinkModel
{
private:
    iDynTree::Model m_model;
    const std::string m_linkName = "link";
public:

    OneLinkModel()

    {   
        // Create a model with one link
        Link link;
        m_model.addLink(m_linkName, link);

    }

    void addContactWrench()
    {
        // Add a frame to the link. The frame will be used to apply the contact wrench.
        const std::string frameName = "frame";
        double x = 2.0; // X coordinate of the frame
        iDynTree::Transform link_H_frame(iDynTree::Rotation::Identity(), iDynTree::Position(x, 0, 0));
        m_model.addAdditionalFrameToLink(m_linkName, frameName, link_H_frame);

        // Create a LinkContactWrenches object
        iDynTree::LinkContactWrenches contactWrenches(m_model);

        // Create a ContactWrench object
        iDynTree::ContactWrench contactWrench;
        double fz = 10.0; // Force in Z direction
        contactWrench.contactPoint() = iDynTree::Position(0, 0, 0);
        iDynTree::Force force(0.0, 0.0, fz);
        iDynTree::Torque torque(0.0, 0.0, 0.0);
        contactWrench.contactWrench() = iDynTree::Wrench(force, torque);

        // Add the contact wrench to the link
        contactWrenches.addNewContactInFrame(m_model, m_model.getFrameIndex(frameName), contactWrench);
        
        // Compute the net external wrench on the link
        iDynTree::LinkNetExternalWrenches netWrenches(m_model);
        contactWrenches.computeNetWrenches(netWrenches);

        // Check the net wrench
        iDynTree::Wrench netWrench = netWrenches(m_model.getLinkIndex(m_linkName));
        // The z component of the force should be equal to fz, all other components should be zero
        ASSERT_EQUAL_DOUBLE(netWrench.getLinearVec3()(0), 0.0);
        ASSERT_EQUAL_DOUBLE(netWrench.getLinearVec3()(1), 0.0);
        ASSERT_EQUAL_DOUBLE(netWrench.getLinearVec3()(2), fz);
        // The y component of the torque should be equal to -x * fz, all other components should be zero
        ASSERT_EQUAL_DOUBLE(netWrench.getAngularVec3()(0), 0.0);
        ASSERT_EQUAL_DOUBLE(netWrench.getAngularVec3()(1), -x * fz);
        ASSERT_EQUAL_DOUBLE(netWrench.getAngularVec3()(2), 0.0);

    };
    
    ~OneLinkModel() = default;

};

int main()
{
    OneLinkModel oneLinkModel;
    oneLinkModel.addContactWrench();
    return EXIT_SUCCESS;
}