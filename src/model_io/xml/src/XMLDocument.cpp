/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Author: Francesco Romano - Google LLC
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "XMLDocument.h"

#include "XMLElement.h"

#include <sstream>

namespace iDynTree {
    
    class XMLDocument::XMLDocumentPimpl
    {
    public:
        std::shared_ptr<XMLElement> m_root;
    };
    
    XMLDocument::XMLDocument()
    : m_pimpl(new XMLDocumentPimpl()) {}
    
    XMLDocument::~XMLDocument() {}
    
    void XMLDocument::setRootElement(std::shared_ptr<XMLElement> root)
    {
        m_pimpl->m_root = root;
    }
    
    const std::shared_ptr<XMLElement> XMLDocument::root() const
    {
        return m_pimpl->m_root;
    }
    
    std::shared_ptr<XMLElement> XMLDocument::rootElementForName(const std::string& name)
    {
        return std::make_shared<XMLElement>(name);
    }

    bool XMLDocument::documentHasBeenParsed() { return true; }
    
    std::string XMLDocument::description() const
    {
        std::ostringstream str;
        if (m_pimpl->m_root) {
            str << m_pimpl->m_root->description();
        }
        return str.str();
    }
}
