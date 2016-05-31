#include "regex.hpp"
#include <stdio.h>
#include <assert.h>
#include <pcre.h>
#include <iostream>
#include <fstream>

#define OVECTOR_SIZE 999
#define MAX_STR_SIZE 1000

using namespace std;

/// slurp_file
bool slurp_file(const char *filename, string& fileText)
{
	string line;
	ifstream in(filename);
	
	// check the validity of the stream
	if (!in) return false;
	while ( getline(in, line) )
	{
		fileText += line + "\n";
	}
	return true;
}

// regex
bool regex( const string& input, const string& pattern, match_t& matchVec)
{
	pcre*       re;
	const char*	err;
	int         erroffset;
	int         rc;
	int         ovector[OVECTOR_SIZE];
	bool        isMatch = false;
	string      match;
	unsigned long i, start=0, finish=0, length;
	
	// prep the inputs
	matchVec.clear();
	matchVec.first = matchVec.last;
	
	// initialize the ovector
	for (i=0;i<OVECTOR_SIZE;i++) ovector[i]=0;

	// check to make sure that it is not too long
	if ( matchVec.first >= input.size() ) {
		//printf("first = %d, size = %d", matchVec.first, str.size());
		return false;
	}

	// compile the expression
	re = pcre_compile(
           pattern.c_str(),  /* the pattern */
           0,                /* default options */
           &err,	           /* for error message */
           &erroffset,       /* for error offset */
           NULL);            /* use default character tables */
	if (re == NULL) {
		printf("regex() error: %s\n", err);
		return false;
	}
	
	// execute
	rc = pcre_exec(
		re,             /* result of pcre_compile() */
		NULL,           /* we didn’t study the pattern */
		input.c_str(),	/* the subject string */
		input.size(), 	/* the length of the subject string */
		matchVec.first, /* start at offset 0 in the subject */
		0,              /* default options */
		ovector,        /* vector of integers for substring information */
		OVECTOR_SIZE);  /* number of elements (NOT size in bytes) */
	
	if ( rc >= 0 )
	{
		isMatch = true;
		for ( i=0; i<OVECTOR_SIZE*2/3; i+=2 )
		{
			start  = ovector[i];
			finish = ovector[i+1];
			//printf("\n%d - %d", start, finish);
			if ( (start==0 && finish==0) || (start>input.size()) || (finish>input.size()) ) break;
			else
			{
				length 	= finish - start;
				match.assign(input.c_str(), start, length);
				//printf("\n%s", match.c_str() );
				matchVec.push_back(match);
			}
		}
		matchVec.first = ovector[0];
		matchVec.last  = ovector[1];
		if (ovector[0]==0 && ovector[1]==0)
			return false;
	}
	else if (rc != PCRE_ERROR_NOMATCH)
	{
		printf("regex(): match error\n");
		return false;
	}
	
	// free memory
	pcre_free( re );

	return isMatch;
}

bool regex( const string& str, const string& pattern )
{
	match_t dummy;
	return regex( str, pattern, dummy );
}

// get_next_xml (input, tag, content, attributes, start_point );
bool get_next_xml ( const string& input, string& tag, string& content, string& attributes, int& start_point )
{
	match_t match;
	match.first = start_point;
	match.last  = start_point;
	char pattern[] = "(?s)<\\s*(\\w+)([^>]*)>(.*?)<\\s*/\\1\\s*>";
	
	if ( regex ( input, pattern, match ) )
	{
		tag = match[1];
		content = match[3];
		attributes = match[2];
		start_point = match.last;
		return true;
	}
	else 
	{
		// leave the start point unchanged
		return false;
	}
	
	return false;
}

// get_next_xml (input, tag, content, attributes, start_point=0 );
bool get_next_xml ( const string& input, string& tag, string& content, string& attributes )
{
	int zero=0;
	return get_next_xml (input, tag, content, attributes, zero);
}

// get_next_xml (input, tag, content, start_point=0 );
bool get_next_xml ( const string& input, string& tag, string& content, int& start_point )
{
	std::string attributes(" ");
	return get_next_xml (input, tag, content, attributes, start_point);
}

// get_next_xml (input, tag, content, start_point=0 );
bool get_next_xml ( const string& input, string& tag, string& content)
{
	int zero=0;
	std::string attributes(" ");
	return get_next_xml (input, tag, content, attributes, zero);
}

void write_xml ( string& str, const string& tag, const string& content )
{
	int sz = tag.size()*2 + content.size() + 10;
	char* tmp = new char[sz];
	
	sprintf( tmp, "<%s>%s</%s>", tag.c_str(), content.c_str(), tag.c_str() );
	str.append(tmp);
	
	delete [] tmp;
}
void write_xml ( std::string& str, const string& tag, double content )
{
	char str_value[50];
	
	sprintf(str_value, "%g", content);
	write_xml ( str, tag, str_value );
}
void write_xml ( std::string& str, const string& tag, int content )
{
	char str_value[50];
	
	sprintf(str_value, "%d", content);
	write_xml ( str, tag, str_value );
}
void write_xml ( std::string& str, const string& tag, complex<double> content )
{
	char str_value[50];
	
	sprintf( str_value, "%g, %g", content.real(), content.imag() );
	write_xml ( str, tag, str_value );
}

