// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_XML_XMLDOCUMENT_H
#define IDYNTREE_MODELIO_XML_XMLDOCUMENT_H

#include <memory>
#include <vector>
#include <string>

namespace iDynTree {
    class XMLDocument;
    class XMLElement;
    class XMLParser;
    class XMLParserState;
}

class iDynTree::XMLDocument {
    class XMLDocumentPimpl;
    std::unique_ptr<XMLDocumentPimpl> m_pimpl;
    
    friend class XMLParser;
    void setRootElement(std::shared_ptr<XMLElement> root);
    
public:
    
    explicit XMLDocument(XMLParserState& parserState);
    virtual ~XMLDocument();

    /**
     * Factory method to create the XML root element given for the specified name.
     *
     * @param name name of the element to create
     * @return a new parser element for the corresponding tag
     */
    virtual std::shared_ptr<XMLElement> rootElementForName(const std::string& name,
                                                           const std::vector<std::string>& packageDirs);

    // TODO: find a better name
    virtual bool documentHasBeenParsed();
    
    const std::shared_ptr<XMLElement> root() const;
    std::string description() const;

protected:
    XMLParserState& getParserState();
};

#endif /* end of include guard: IDYNTREE_MODELIO_XML_XMLDOCUMENT_H */
