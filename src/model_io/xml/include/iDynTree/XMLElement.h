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

#ifndef IDYNTREE_MODELIO_XML_XMLELEMENT_H
#define IDYNTREE_MODELIO_XML_XMLELEMENT_H

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace iDynTree {
    class XMLAttribute;
    class XMLElement;
    class XMLParser;
}


/**
 * Class representing an XML element.
 *
 * This class represents an XML element encountered during parsing,
 * i.e. the XML part between (and comprising) <tag> ... </tag>
 */
class iDynTree::XMLElement {
private:
    class XMLElementPimpl;
    std::unique_ptr<XMLElementPimpl> m_pimpl;
    
    friend class XMLParser;

    /**
     * Adds a child element to the current element
     *
     * @param child the child element
     */
    void addChildElement(std::shared_ptr<XMLElement> child);
    
public:
    /**
     * Default constructor.
     *
     * Constructs an unnamed XML Element with no attributes.
     */
    XMLElement();
    
    /**
     * Constructs a named XML Element with no attributes.
     * @param name the name of the XML element.
     */
    XMLElement(const std::string& name);
    
    /**
     * Constructs a named XML Element with the specified attributes.
     *
     * This is the designated initializer.
     * @param name the name of the XML element.
     * @param attributes the attributes of the XML element.
     */
    XMLElement(const std::string& name,
               const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes);
    
    /**
     * Destructor
     */
    virtual ~XMLElement();

    /**
     * Sets a callback that will be called when attributes will be parsed
     *
     * The function should have the following signature `const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& -> bool`.
     * The attributes parsed from the XML will be passed as argument to the function.
     * In case of error, it should return false, true otherwise.
     * @param callback the callback to be set
     */
    void setAttributeCallback(std::function<bool(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>&)> callback);

    /**
     * Sets a callback that will be called when the element has been finished parsing.
     *
     * The function should have the following signature `void -> void`.
     * @param callback the callback
     */
    void setExitScopeCallback(std::function<void()>  callback);

    /**
     * Sets a callback that will be called when a child element has been finished parsing
     *
     * The callback should have the following signature `std::shared_ptr<XMLElement> -> void`,
     * where the element passed is the child element that has been parsed.
     * @param callback the callback
     */
    void setChildHasBeenParsedCallback(std::function<void(std::shared_ptr<XMLElement>)> callback);
    
    std::function<bool(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>&)> attributeCallback() const;
    std::function<void()> exitScopeCallback() const;
    
    std::string name() const;
    const std::vector<std::shared_ptr<XMLElement>> children() const;
    
    const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>> attributes() const;
    
    /**
     * Factory method to create child element given the name.
     *
     * @param name name of the element to create
     * @return a new parser element for the corresponding tag
     */
    virtual std::shared_ptr<XMLElement> childElementForName(const std::string& name);
    
    virtual void exitElementScope();

    virtual void childHasBeenParsed(std::shared_ptr<XMLElement> parsedChild);
    
    virtual bool setAttributes(const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>>& attributes);
    
    virtual void parsedCharacters(const std::string& characters);
    
    std::string getParsedTextContent() const;
    
    std::string description() const;

};


#endif /* end of include guard: IDYNTREE_MODELIO_XML_XMLELEMENT_H */
