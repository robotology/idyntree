#include <iDynTree/TestUtils.h>

#include <iDynTree/XMLParser.h>
#include <iDynTree/XMLElement.h>
#include <iDynTree/XMLAttribute.h>
#include <iDynTree/XMLDocument.h>

#include <iostream>
#include <string>


int main() {
    
    std::string schema = std::string(IDYNTREE_TEST_FILES_DIR) + "/" + "schema.xsd";
    
    std::string validXML = std::string(IDYNTREE_TEST_FILES_DIR) + "/" + "valid.xml";
    std::string nonValidXML = std::string(IDYNTREE_TEST_FILES_DIR) + "/" + "invalid_schema.xml";
    std::string notValidFormatXML = std::string(IDYNTREE_TEST_FILES_DIR) + "/" + "invalid_xml.xml";
    std::string doubleRootXML = std::string(IDYNTREE_TEST_FILES_DIR) + "/" + "double_root.xml";
    
    
    // allocate parser
    iDynTree::XMLParser parser;
    parser.setSchemaLocation(schema);
//    parser.setLogParsing(true);
    parser.setKeepTreeInMemory(true);
    parser.setValidateXML(true);
    
    std::cerr << "Parsing " << validXML << std::endl;
    ASSERT_IS_TRUE(parser.parseXMLFile(validXML));
    std::cerr << "Successfuly parsed " << validXML << std::endl << std::endl;
    
    //Test changing the structure
//    auto attribute = parser.document()->root()->attributes().find("orderid");
//    attribute->second->setValue("ciao");
    
//    std::cerr << parser.document()->description() << std::endl << std::endl;

    std::cerr << "Parsing " << nonValidXML << std::endl;
    ASSERT_IS_FALSE(parser.parseXMLFile(nonValidXML));
    std::cerr << "Could not parse " << nonValidXML << std::endl << std::endl;
    
    // This instead should be parsed
    // Disable validation: well formed XML should be parsed, even if they do not validate
    // against the XSD schema.
    parser.setValidateXML(false);
    
    std::cerr << "Parsing (No validation) " << nonValidXML << std::endl;
    ASSERT_IS_TRUE(parser.parseXMLFile(nonValidXML));
    std::cerr << "Successfuly parsed " << nonValidXML << std::endl << std::endl;

    std::cerr << "Parsing " << notValidFormatXML << std::endl;
    ASSERT_IS_FALSE(parser.parseXMLFile(notValidFormatXML));
    std::cerr << "Could not parse " << notValidFormatXML << std::endl << std::endl;
    
    std::cerr << "Parsing " << doubleRootXML << std::endl;
    ASSERT_IS_FALSE(parser.parseXMLFile(doubleRootXML));
    std::cerr << "Could not parse " << doubleRootXML << std::endl << std::endl;
    
    return EXIT_SUCCESS;
}
