#ifndef MYXML_HPP
#define MYXML_HPP

#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <string>
#include <stdlib.h>

#define MYXML_EMPTY -99999999

/// a class for read only access to an XML node
///  does not support nodes with more than one attribute
class MyXmlNode {
	public:
		MyXmlNode() { node_=NULL; }
		/// const
		MyXmlNode( xmlNode * node ) { node_=node; }
		/// check to see if this is a leaf node
		bool is_null() { return node_==NULL; }
		/// return the next node-sibling 
		MyXmlNode next() { return MyXmlNode( node_->next ); }
		/// return the previous node-sibling
		MyXmlNode previous() { return MyXmlNode( node_->prev ); }
		/// return the first child of the node
		MyXmlNode child() { return MyXmlNode( node_->children ); }
		/// return the parent of the node
		MyXmlNode parent() { return MyXmlNode( node_->parent ); }
		/// return the name of the node
		std::string name() { return std::string( (char*)node_->name ); }
		/// return the content of the node
		std::string content();
		/// return the content in numberic form
		double numeric_content();
		/// get an attribute value from the node
		std::string get_attribute( const std::string & attribute_name );
		/// get an attribute value from the node
		std::string getAttribute( const std::string & attribute_name ) {
			return get_attribute( attribute_name );
		}
		/// get a numeric attribute value from the node
		double get_numeric_attribute( const std::string & attribute_name );
		/// get a numeric attribute value from the node
		double getNumericAttribute( const std::string & attribute_name ) {
			return get_numeric_attribute( attribute_name );
		}
		/// get the content of a requested child node
		std::string get_child_content( const std::string & child_name );
		/// get the content of a requested child node
		std::string getChildContent( const std::string & child_name ) {
			return get_child_content( child_name );
		}
		/// get the numeric content of a requested child node
		double get_child_numeric_content( const std::string & child_name );
		/// get the numeric content of a requested child node
		double getChildNumericContent( const std::string & child_name ) {
			return get_child_numeric_content( child_name );
		}
		/// return true if the node has content
		bool has_content()  { return node_->type==XML_ELEMENT_NODE; }
		bool has_children() { return not(node_->children==NULL); }
		bool has_siblings() { return node_->next!=NULL; }
		/// return true if the node has attributes
		bool has_attributes() { return ( node_->properties != NULL ); }
		/// operator++
		void operator++() { node_ = node_->next; }
	private:
		xmlNode * node_;
};

class MyXmlDocument {
	public:
		/// constructor
		MyXmlDocument();
		/// destructor
		~MyXmlDocument();
		/// parse xml data from a file
		bool read_file( const std::string & filename );
		/// parse xml data from a string (does not validate)
		bool read_string( const std::string & xml_str );
		/// check to see if the xml is valid
		bool is_valid();
		/// return the root node of the document
		MyXmlNode root_node();
		/// print a form of the document
		void print();
	private:
		xmlDoc * doc_;
};
#endif
