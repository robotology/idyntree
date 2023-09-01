// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "XMLDocument.h"

#include "XMLElement.h"

#include <sstream>

namespace iDynTree {
    
    class XMLDocument::XMLDocumentPimpl
    {
    public:
        std::shared_ptr<XMLElement> m_root;
        XMLParserState& m_parserState;

        explicit XMLDocumentPimpl(XMLParserState& parserState)
        : m_parserState(parserState) {}
    };
    
    XMLDocument::XMLDocument(XMLParserState& parserState)
    : m_pimpl(new XMLDocumentPimpl(parserState)) {}
    
    XMLDocument::~XMLDocument() {}
    
    void XMLDocument::setRootElement(std::shared_ptr<XMLElement> root)
    {
        m_pimpl->m_root = root;
    }
    
    const std::shared_ptr<XMLElement> XMLDocument::root() const
    {
        return m_pimpl->m_root;
    }
    
    std::shared_ptr<XMLElement> XMLDocument::rootElementForName(const std::string& name,
                                                                const std::vector<std::string>& packageDirs)
    {
        return std::make_shared<XMLElement>(m_pimpl->m_parserState, name);
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

    XMLParserState& XMLDocument::getParserState() { return m_pimpl->m_parserState; }
}
