#ifndef VECTOR_BASE_H
#define VECTOR_BASE_H

#include "matrixable.h"

/// class vector_base_t
///  Overloads the matrix_base_t class specifically for vectors
class vector_base_t : public Matrixable
{
	public:
		/// constructor
		vector_base_t() : length_(0), transposed_(false) {};
		/// rows - returns the number of rows in the vector
		inline unsigned rows() const { return transposed_ ? 1 : length_; };
		/// cols - returns the number of columns in the vector
		inline unsigned cols() const { return transposed_ ? length_ : 1; };
		/// length - returns the length of the vector
		inline unsigned length() const { return length_; };
		/// size - returns the size of the vector ( same as length() )
		inline unsigned size() const { return length_; };
		/// transpose - transposes the vector
		inline void transpose()  { transposed_=!transposed_; };
		/// transposed - returns true if the vector is transposed
		inline bool transposed() const { return transposed_; };
	protected:
		// member variables
		unsigned length_;     ///< the number of elements in the vector
		bool     transposed_; ///< true if the vector is transposed
};

#endif
