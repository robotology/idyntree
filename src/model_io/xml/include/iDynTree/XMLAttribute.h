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

#ifndef IDYNTREE_MODELIO_XML_XMLATTRIBUTE_H
#define IDYNTREE_MODELIO_XML_XMLATTRIBUTE_H

#include <string>

namespace iDynTree {
    class XMLAttribute;
}

class iDynTree::XMLAttribute {
private:
    std::string m_name;
    std::string m_value;
    std::string m_prefix;
    std::string m_uri;
    
public:
    XMLAttribute(std::string name, std::string value, std::string prefix, std::string uri);
    
    // We allow to modify the attribute after it has been created.
    // This might be useful for XML processing
    
    void setName(std::string name);
    void setValue(std::string value);
    void setPrefix(std::string prefix);
    void setURI(std::string uri);
    
    std::string name() const;
    std::string value() const;
    std::string prefix() const;
    std::string uri() const;
    
    const std::string description() const;
};

#endif /* end of include guard: IDYNTREE_MODELIO_XML_XMLATTRIBUTE_H */
