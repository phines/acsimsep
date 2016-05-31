// regex.h
//  simple regular expression handler

#ifndef MY_REGEX_H
#define MY_REGEX_H
#include <vector>
#include <complex>
#include <string>

/// match_t is used to store the strings extracted by the regular expression
class match_t : public std::vector<std::string>
{
	public:
		unsigned first;
		unsigned last;
		match_t()    { first=0; last=0; };	
		void reset() { first=0; last=0; };
};

typedef match_t matchVec_t;

/// regex -- for evaluating a regular expression
bool regex( const std::string& input, const std::string& pattern, match_t& match_results );
bool regex( const std::string& input, const std::string& pattern);

/// slurp_file(filename, output_string)
///  Reads filename, and puts the contents into output_string
bool slurp_file( const char* filename, std::string& output_string );


/// get_next_xml (input, tag, content, attributes, start_point=0)
///  Finds the first closed xml tag and puts the result in tag, content, attributes.
///  Annotations will be the text "id=N" in: "<tag id=N> text </tag>"
bool get_next_xml ( const std::string& input, std::string& tag, std::string& content, std::string& attributes, int& start_point );
bool get_next_xml ( const std::string& input, std::string& tag, std::string& content, std::string& attributes );

/// get_next_xml (input, tag, content, start_point=0)
///  Finds the first closed xml tag and puts the result in tag, content.
///   It will start its search at start_point, and update start_point to point to the end of the tag + 1 or 0 if failure.
bool get_next_xml ( const std::string& input, std::string& tag, std::string& content, int& start_point );
bool get_next_xml ( const std::string& input, std::string& tag, std::string& content );

/// write_xml (string, tag, content)
///  Appends tag and content to string
void write_xml ( std::string& str, const std::string& tag, const std::string& content );
void write_xml ( std::string& str, const std::string& tag, std::complex<double> content );
void write_xml ( std::string& str, const std::string& tag, double content );
void write_xml ( std::string& str, const std::string& tag, int content );

#endif
