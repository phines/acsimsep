/**
 * section: Tree
 * synopsis: Navigates a tree to print element names
 * purpose: Parse a file to a tree, use xmlDocGetRootElement() to
 *          get the root element, then walk the document and print
 *          all the element name in document order.
 * usage: tree1 filename_or_URL
 * test: tree1 test2.xml > tree1.tmp ; diff tree1.tmp tree1.res ; rm tree1.tmp
 * author: Dodji Seketeli
 * copy: see Copyright for the status of this software.
 */

#include "myxml.hpp"
#include <iostream>

using namespace std;

/**
 * Simple example to parse a file called "file.xml", 
 * walk down the DOM, and print the name of the 
 * xml elements nodes.
 */
int main(int argc, char **argv)
{
	MyXmlDocument doc;
	char *filename = NULL;
	char default_filename[] = "test.xml";
	char xml_string[] = "<bus number=\"5\"><type>REF</type><Vmag>1.05</Vmag><Vang>0.001</Vang><Vmax>1.1</Vmax><Vmin>0.9</Vmin><area>1</area><zone>1</zone><locX>0</locX><locY>0</locY><freq>60</freq></bus>";
	
	printf("myxml test program.\n");
	if (argc < 2) {
		printf("No XML input file given.\n");
		filename = default_filename;
		printf("Using default: %s.\n", filename);
	} else {
		filename = argv[1];
	}
	
	// parse a file 
	printf("Reading file: %s\n", filename);	
	if ( not doc.read_file( filename ) ) {
		throw "error";
	}
	// print the document
	doc.print();
	
	// parse the string
	printf("Reading string: %s\n", xml_string );
	if (not doc.read_string( xml_string ) ) {
		throw "error";
	}
	doc.print();
	int number = (int) doc.root_node().get_numeric_attribute( "number" );
	double Vmag = doc.root_node().get_child_numeric_content( "Vmag" );
	double Vang = doc.root_node().get_child_numeric_content( "Vang" );
	printf("Bus %d has voltage: %f /_ %f\n", number, Vmag, Vang );
	
	return 0;
}
