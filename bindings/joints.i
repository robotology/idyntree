
// Bindings extention of IJoint interface
// (to overcome lack of dynamic casts in high level languages)

namespace iDynTree{

%extend IJoint
{
    // Expose all sensors type

    // TODO implement this as SWIG macro

    // TODO implement this as a dynamic cast and print an error if the dynamic cast failes

    bool isRevoluteJoint() const
    {
        const iDynTree::RevoluteJoint * p =
            dynamic_cast<const iDynTree::RevoluteJoint*>($self);
        return (p != 0);
    }

    bool isFixedJoint() const
    {
        const iDynTree::FixedJoint * p =
            dynamic_cast<const iDynTree::FixedJoint*>($self);
        return (p != 0);
    }
    
    bool isPrismaticJoint() const
    {
        const iDynTree::PrismaticJoint * p =
            dynamic_cast<const iDynTree::PrismaticJoint*>($self);
        return (p != 0);
    }

    iDynTree::RevoluteJoint * asRevoluteJoint()
    {
        iDynTree::RevoluteJoint * p =
            static_cast<iDynTree::RevoluteJoint*>($self);
        return p;
    }

    iDynTree::FixedJoint * asFixedJoint()
    {
        iDynTree::FixedJoint * p =
            static_cast<iDynTree::FixedJoint*>($self);
        return p;
    }

    iDynTree::PrismaticJoint * asPrismaticJoint()
    {
        iDynTree::PrismaticJoint * p =
            static_cast<iDynTree::PrismaticJoint*>($self);
        return p;
    }
}

}
