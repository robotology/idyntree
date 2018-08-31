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

#include "XMLAttribute.h"

#include <sstream>

namespace iDynTree {
    
    //MARK: - XMLAttribute definition
    
    XMLAttribute::XMLAttribute(std::string name, std::string value, std::string prefix, std::string uri)
    : m_name(name)
    , m_value(value)
    , m_prefix(prefix)
    , m_uri(uri) {}
    
    void XMLAttribute::setName(std::string name)
    {
        m_name = name;
    }
    
    void XMLAttribute::setValue(std::string value)
    {
        m_value = value;
    }
    
    void XMLAttribute::setPrefix(std::string prefix)
    {
        m_prefix = prefix;
    }
    
    void XMLAttribute::setURI(std::string uri)
    {
        m_uri = uri;
    }
    
    std::string XMLAttribute::name() const
    {
        return m_name;
    }
    std::string XMLAttribute::value() const
    {
        return m_value;
    }
    std::string XMLAttribute::prefix() const
    {
        return m_prefix;
    }
    std::string XMLAttribute::uri() const
    {
        return m_uri;
    }
    
    const std::string XMLAttribute::description() const
    {
        std::ostringstream str;
        if (!m_prefix.empty()) {
            str << m_prefix << ":";
        }
        str << m_name << "=" << "\"" << m_value <<"\"";
        return str.str();
    }
}
