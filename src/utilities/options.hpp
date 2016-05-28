#ifndef OPTIONS_HPP
#define OPTIONS_HPP

#include <map>
#include <string>

/// options_t is a class that can be used to store options that would be read from a file.
///  Adding an option this way is simpler than creating new class variables
class options_t {
	public:
		/// read the options found in @param filename, and put them in the structure
		bool read_file( const std::string & filename, bool showErrorMessages=true );
		/// get the option with name @param opt_name
		std::string get( const std::string & opt_name );
		/// get a numeric option
		double get_numeric( const std::string & opt_name );
		/// set a string value
		void set( const std::string & opt_name, const std::string & value );
		/// set a numeric value
		void set( const std::string & opt_name, double value );
	private:
		/// the actual structure that stores the options
		std::map<std::string,std::string> opts_;
};

#endif
