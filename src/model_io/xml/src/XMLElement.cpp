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

#include "XMLElement.h"

#include "XMLAttribute.h"

#include <sstream>
#include <unordered_map>
#include <vector>


namespace iDynTree {
    
    //MARK: - XMLElementPimpl definition
    
    class XMLElement::XMLElementPimpl {
    public:
        // Functions that can be set to change the behaviour.
        // Useful as an alternative to inheritance
        std::function<bool(const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>&)> f_attribute;
        std::function<void()> f_exitScope;
        std::function<void(std::shared_ptr<XMLElement>)> f_childParsed;
        
        // contains the textual content of the element
        std::stringstream m_charactersStream;
        
        std::string m_name;
        std::vector<std::shared_ptr<XMLElement>> m_children;
        std::unordered_map<std::string, std::shared_ptr<XMLAttribute>> m_attributes;
    };
    
    //MARK: - XMLElement definition
    
    XMLElement::XMLElement()
    : XMLElement("") {}
    
    XMLElement::XMLElement(const std::string& name)
    : XMLElement(name, std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>()) {}
    
    XMLElement::XMLElement(const std::string& name,
                           const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes)
    : m_pimpl(new XMLElementPimpl())
    {
        m_pimpl->m_name = name;
        m_pimpl->m_attributes = attributes;
    }
    
    XMLElement::~XMLElement() {}
    
    void XMLElement::addChildElement(std::shared_ptr<XMLElement> child)
    {
        m_pimpl->m_children.push_back(child);
    }
    
    void XMLElement::setAttributeCallback(std::function<bool(const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>&)> callback)
    {
        m_pimpl->f_attribute = callback;
    }
    
    void XMLElement::setExitScopeCallback(std::function<void()> callback)
    {
        m_pimpl->f_exitScope = callback;
    }

    void XMLElement::setChildHasBeenParsedCallback(std::function<void(std::shared_ptr<XMLElement>)> callback)
    {
        m_pimpl->f_childParsed = callback;
    }
    
    std::function<bool(const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>&)> XMLElement::attributeCallback() const
    {
        return m_pimpl->f_attribute;
    }
    std::function<void()> XMLElement::exitScopeCallback() const
    {
        return m_pimpl->f_exitScope;
    }
    
    std::string XMLElement::name() const { return m_pimpl->m_name; }
    
    const std::vector<std::shared_ptr<XMLElement>> XMLElement::children() const
    {
        return m_pimpl->m_children;
    }
    
    std::shared_ptr<XMLElement> XMLElement::childElementForName(const std::string& name)
    {
        return std::make_shared<XMLElement>(name);
    }
    
    void XMLElement::exitElementScope()
    {
        if (m_pimpl->f_exitScope) {
            m_pimpl->f_exitScope();
        }
    }

    void XMLElement::childHasBeenParsed(std::shared_ptr<XMLElement> parsedChild)
    {
        if (m_pimpl->f_childParsed) {
            m_pimpl->f_childParsed(parsedChild);
        }
    }
    
    bool XMLElement::setAttributes(const std::unordered_map<std::string, std::shared_ptr<XMLAttribute>>& attributes)
    {
        m_pimpl->m_attributes = attributes;
        if (m_pimpl->f_attribute) return m_pimpl->f_attribute(attributes);
        return true;
    }
    
    const std::unordered_map<std::string, std::shared_ptr<iDynTree::XMLAttribute>> XMLElement::attributes() const
    {
        return m_pimpl->m_attributes;
    }
    
    void XMLElement::parsedCharacters(const std::string& characters)
    {
        m_pimpl->m_charactersStream << characters;
    }
    
    std::string XMLElement::getParsedTextContent() const
    {
        return m_pimpl->m_charactersStream.str();
    }
    
    std::string XMLElement::description() const
    {
        std::ostringstream str;
        str << "<" << m_pimpl->m_name;
        for (const auto& attribute : m_pimpl->m_attributes) {
            str << " " << attribute.second->description();
        }
        str << ">" << std::endl;
        for (const auto& child : m_pimpl->m_children) {
            str << child->description() << std::endl;
        }
        str << getParsedTextContent();
        str << "</" << m_pimpl->m_name << ">" << std::endl;
        return str.str();
    }
}
