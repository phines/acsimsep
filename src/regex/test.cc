#include "regex.h"
#include <iostream>

using namespace std;

int main(void)
{
	string text = "Some text <tag1 id=5> some more\n <b>text</b> </tag1> \n \n<tag2 id=7> again some more text </tag2>";
	string tag;
	string content;
	string annotations;	
	int    start=0;
	
	cout << "Here is some xml that we will parse" << endl;
	cout << text << endl;
	cout << "Here is the result of the parsing" << endl;
	while ( get_next_xml( text, tag, content, annotations, start ) )
	{
		cout << "tag:         " << tag << endl;
		cout << "content:     " << content << endl;
		cout << "annotations: " << annotations << endl;
	}
	return 0;
}
