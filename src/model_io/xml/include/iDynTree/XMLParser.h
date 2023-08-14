// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_MODELIO_XML_XMLPARSER_H
#define IDYNTREE_MODELIO_XML_XMLPARSER_H

#include <functional>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <unordered_map>

namespace iDynTree {
    
    class XMLParser;
    class XMLElement;
    class XMLDocument;
    class XMLParserState;
}

//TODO: handle errors

/** XML Parser class
 *
 * Use this class to parse XML files. It currently supports the following features
 * - XSD validation
 * - Extensibility (by providing a different root object for the XML hierarchy.
 *
 * ## Common usage
 *
 * ```{.cpp}
 *
 * // Allocate parser
 * std::shared_ptr<XMLParser> parser = std::make_shared<XMLParser>();
 * // If needed, you can customize the class representing the XML Document
 * // just set a function returning a subclass of XMLElement
 * parser->setDocumentFactory([]{ return std::shared_ptr<XMLDocument>(new MyCustomXMLDocument); });
 * // Parse
 * parser->parseXMLFile(filename);
 * // You can retrieve the Document after the parsing
 * std::shared_ptr<const XMLDocument> document = parser->document();
 *
 * ```
 * ## Description
 *
 * ## Validation support
 *
 * The parser supports XSD schema validation. This options is controlled by two methods:
 *
 * - `setValidateXML` accepts a boolean, to enable the validation
 * - `setSchemaLocation` specifies the location of the XSD schema.
 *
 * If both variables are set, the validation will take place before parsing.
 * Note that currently the errors are output to standard error (or output?) and not handled directly
 * in the code. It might be possible to handle those in code though (feature request).
 *
 * ## Todos:
 * Possible feature requests:
 *
 * - parse entities out of context (xmlSAXParseEntity)
 * - Validation for in memory parsing
 * - Use libxml in memory tree (that can be modified and dumped to file) instead of custom tree
 */
class iDynTree::XMLParser {
    
    class XMLParserPimpl;
    std::unique_ptr<XMLParserPimpl> m_pimpl;
    
public:

    /**
     * Default constructor
     */
    XMLParser();

    /**
     * Destructor
     */
    ~XMLParser();

    /**
     * Returns true if the parsed tree is ketp in memory.
     * @see setKeepTreeInMemory(bool)
     * @return true if the parsed tree is kept on memory.
     */
    bool keepTreeInMemory() const;

    /**
     * Set if the parse tree should be kept in memory
     * @see keepTreeInMemory
     * @param keepTreeInMemory true if the tree should be kept in memory.
     */
    void setKeepTreeInMemory(bool keepTreeInMemory);


    void setPackageDirs(const std::vector<std::string>& packageDirs);
    const std::vector<std::string>& packageDirs() const;

    /**
     * Returns true if the parser logs the parsing to standard output.
     * @see setLogParsing(bool)
     * @return true if log to standard output is enabled.
     */
    bool logParsing() const;

    /**
     * Set the logging option of the parser.
     * see logParsing
     * @param enableLogging true if the parser should log to standard output.
     */
    void setLogParsing(bool enableLogging);

    /**
     * Returns true if the validation option is enabled.
     * @see setValidateXML(bool)
     * @see schemaLocation
     * @see setSchemaLocation(std::string)
     * @return true if the XML should be validated against a schema.
     */
    bool validateXML() const;

    /**
     * Set the option to perform Schema validation.
     *
     * @note both an XSD schema location and this option should be specified
     * to perform validation.
     *
     * @see validateXML
     * @see schemaLocation
     * @see setSchemaLocation(std::string)
     *
     * @param validate true if the XML should be validated against a XSD.
     */
    void setValidateXML(bool validate);
    
    /**
     * Returns the current schema location used for validation.
     *
     * @see validateXML
     * @see setValidateXML(bool)
     * @see setSchemaLocation(std::string)
     *
     * @return the current schema location.
     */
    std::string schemaLocation() const;

    /**
     * Sets the XSD schema location.
     *
     * @note to perform the validation, the option should be explicitly set, together with the schema location.
     *
     * @see validateXML
     * @see setValidateXML(bool)
     * @see schemaLocation(std::string)
     *
     * @param schemaLocation the new XSD schema location.
     */
    void setSchemaLocation(std::string schemaLocation);

    /**
     * Parse the specified XML file.
     *
     * If the validation option is enabled, the XML file will be also validated against the specified
     * XSD schema.
     *
     * @see setValidateXML(bool)
     * @see setSchemaLocation(std::string)
     * @see parseXMLString(std::string)
     *
     * @param absoluteFileName the XML file to be parsed.
     * @return true if the document is valid and successfully parsed.
     */
    bool parseXMLFile(std::string absoluteFileName);

    /**
     * Parse the specified XML document string.
     *
     * @note XSD validation is currently not supported for in-memory parsing.
     * Use parseXMLFile(std::string) instead
     *
     * @see parseXMLFile(std::string)
     *
     * @param xmlString string containing a valid XML content.
     * @return true if the XML document is valid and successfully parsed.
     */
    bool parseXMLString(std::string xmlString);
    
    /**
     * Set the factory function responsible of creating a new XMLDocument element.
     *
     * By specifying a new factory function, it is possible to change how the XML document will be
     * represented in memory.
     * The signature of the function is `(XMLParserState&) -> std::shared_ptr<XMLDocument>`, i.e. a function
     * accepting a reference to the parser state and returning a `std::shared_ptr` to an `XMLDocument` object.
     *
     @param factory the function that will be called for instantiating a new XMLDocument object.
     */
    void setDocumentFactory(std::function<std::shared_ptr<XMLDocument>(XMLParserState&)> factory);
    
    // TODO: check if we want to return a copy, a const or something

    /**
     * Returns the current parsed document.
     *
     * @return the parsed document.
     */
    std::shared_ptr<const XMLDocument> document() const;
    
};


class iDynTree::XMLParserState {
public:
    void setParsingError();
  
private:
    friend XMLParser;
    void resetState();  // Acquires m_mutex.
    bool getParsingErrorState();  // Acquires m_mutex.

    mutable std::mutex m_mutex;
    bool m_parsing_error;  // Protected by m_mutex.
};



#endif /* end of include guard: IDYNTREE_MODELIO_XML_XMLPARSER_H */
