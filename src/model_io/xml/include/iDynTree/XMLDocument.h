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
