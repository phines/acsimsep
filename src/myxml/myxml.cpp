#include "myxml.hpp"
using namespace std;

/// return the content of the node
string MyXmlNode::content() {
	string result;
	if (is_null()) throw "Attempt to read the content of an empty xml node.";
	xmlChar * xmlStr = xmlNodeGetContent( node_ );
	if ( xmlStr != NULL ) {
		result = (char*) xmlStr;
		xmlFree(xmlStr);
	}
	return result;
}
/// return the content in numberic form
double MyXmlNode::numeric_content() {
	if (is_null()) throw "Attempt to read the content of an empty xml node.";
	double result = MYXML_EMPTY;
	xmlChar * xmlStr = xmlNodeGetContent( node_ );
	if ( xmlStr != NULL ) {
		result = atof( (char*)xmlStr );
		xmlFree(xmlStr);
	}
	return result;
}
/// get the contents of a child node
string MyXmlNode::get_attribute( const string & attribute_name ) {
	if (is_null()) throw "Attempt to read the attributes of an empty xml node.";
	string result;
	xmlChar * xmlStr = xmlGetProp( node_, (xmlChar*) attribute_name.c_str() );
	if( xmlStr!=NULL ) {
		result = (char*)xmlStr;
		xmlFree(xmlStr);
	}
	return result;
}
/// get the contents of a child node
double MyXmlNode::get_numeric_attribute( const string & attribute_name ) {
	if (is_null()) throw "Attempt to read the attributes of an empty xml node.";
	double result = MYXML_EMPTY;
	xmlChar * xmlStr = xmlGetProp( node_, (xmlChar*) attribute_name.c_str() );
	if( xmlStr!=NULL ) {
		result = atof( (char*) xmlStr );
		xmlFree(xmlStr);
	}
	return result;
}
/// get the content of a requested child node
string MyXmlNode::get_child_content( const string & child_name ) {
	if (is_null()) throw "Attempt to access the child of an empty xml node.";
	MyXmlNode node;
	
	for( node=child(); not node.is_null(); node=node.next() ) {
		if ( node.name()==child_name ) return node.content();
	}
	return "";
}
/// get the numeric content of a requested child node
double MyXmlNode::get_child_numeric_content( const string & child_name ) {
	if (is_null()) throw "Attempt to access the child of an empty xml node.";
	MyXmlNode node;
	
	for( node=child(); not node.is_null(); node=node.next() ) {
		if ( node.name()==child_name ) return node.numeric_content();
	}
	return MYXML_EMPTY;
}

/// constructor
MyXmlDocument::MyXmlDocument() {
	LIBXML_TEST_VERSION
	doc_=NULL;
}

/// destructor
MyXmlDocument::~MyXmlDocument() {
	if (doc_!=NULL) xmlFreeDoc( doc_ );
	doc_=NULL;
	xmlCleanupParser();
}

/// parse xml data from a file
bool MyXmlDocument::read_file( const string & filename ) {
	// free the existing document
	if (doc_!=NULL) xmlFreeDoc( doc_ );
	doc_=NULL;
	// read the file
	doc_ = xmlReadFile( filename.c_str(), NULL, 0 );
	// return true if the doc is non-null
	return (doc_!=NULL);
}

/// parse xml data from a string (does not validate)
bool MyXmlDocument::read_string( const string & xml_str ) {
	// free the existing document
	if (doc_!=NULL) xmlFreeDoc( doc_ );
	doc_=NULL;
	// read the file
	doc_ = xmlReadDoc( (xmlChar*)xml_str.c_str(), "noname.xml", NULL, 0 );
	// return true if the doc is non-null
	return (doc_!=NULL);
}

/// return the root node of the document
MyXmlNode MyXmlDocument::root_node() {
	if (doc_==NULL) throw "Cannot get root node of an empty document.";	
	return MyXmlNode( xmlDocGetRootElement( doc_ ) );
}

/// print all nodes
void print_all_nodes( MyXmlNode node_in ) {
	MyXmlNode node=node_in;
	
	if ( node.is_null() ) return;
	
	if ( node.has_content() ) {
		printf(" Name: %s, Content: %s\n", node.name().c_str(), node.content().c_str() );
	}
	if ( node.has_children() ) {
		print_all_nodes( node.child() );
	}
	if ( node.has_siblings() ) {
		print_all_nodes( node.next() );
	}
}

/// print all of the nodes in the document
void MyXmlDocument::print() {
	if (doc_==NULL) throw "Cannot print an empty document.";	
	print_all_nodes( root_node() );
}



#ifndef LIBXML_TREE_ENABLED
int main(void) {
	fprintf(stderr, "Tree support not compiled in\n");
	exit(1);
}
#endif /* end LIBXML_TREE_ENABLED */
