//-----------------------------------------------------------------------------
// Copyright (c) 2012 GarageGames, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#ifndef _MMATRIX_H_
#define _MMATRIX_H_
#include <algorithm>
#ifndef _MPLANE_H_
#include "math/mPlane.h"
#endif

#ifndef _MBOX_H_
#include "math/mBox.h"
#endif

#ifndef _MPOINT4_H_
#include "math/mPoint4.h"
#endif

#ifndef _ENGINETYPEINFO_H_
#include "console/engineTypeInfo.h"
#endif


/// 4x4 Matrix Class
///
/// This runs at F32 precision.

class MatrixF
{
   friend class MatrixFEngineExport;
private:
   F32 m[16];     ///< Note: Torque uses row-major matrices

public:
   /// Create an uninitialized matrix.
   ///
   /// @param   identity    If true, initialize to the identity matrix.
   explicit MatrixF(bool identity=false);

   /// Create a matrix to rotate about origin by e.
   /// @see set
   explicit MatrixF( const EulerF &e);

   /// Create a matrix to rotate about p by e.
   /// @see set
   MatrixF( const EulerF &e, const Point3F& p);

   /// Get the index in m to element in column i, row j
   ///
   /// This is necessary as we have m as a one dimensional array.
   ///
   /// @param   i   Column desired.
   /// @param   j   Row desired.
   static U32 idx(U32 i, U32 j) { return (i + j*4); }

   /// Initialize matrix to rotate about origin by e.
   MatrixF& set( const EulerF &e);

   /// Initialize matrix to rotate about p by e.
   MatrixF& set( const EulerF &e, const Point3F& p);

   /// Initialize matrix with a cross product of p.
   MatrixF& setCrossProduct( const Point3F &p);

   /// Initialize matrix with a tensor product of p.
   MatrixF& setTensorProduct( const Point3F &p, const Point3F& q);

   operator F32*() { return (m); }              ///< Allow people to get at m.
   operator const F32*() const { return (F32*)(m); }  ///< Allow people to get at m.

   bool isAffine() const;                       ///< Check to see if this is an affine matrix.
   bool isIdentity() const;                     ///< Checks for identity matrix.

   /// Make this an identity matrix.
   MatrixF& identity();

   /// Invert m.
   MatrixF& inverse();
   /// Copy the inversion of this into out matrix.
   void invertTo( MatrixF *out );

   /// Take inverse of matrix assuming it is affine (rotation,
   /// scale, sheer, translation only).
   MatrixF& affineInverse();           

   /// Swap rows and columns.
   MatrixF& transpose();

   /// M * Matrix(p) -> M
   MatrixF& scale( const Point3F &s );            
   MatrixF& scale( F32 s ) { return scale( Point3F( s, s, s ) ); }

   /// Return scale assuming scale was applied via mat.scale(s).
   Point3F getScale() const;

   EulerF toEuler() const;

   /// Compute the inverse of the matrix.
   ///
   /// Computes inverse of full 4x4 matrix. Returns false and performs no inverse if
   /// the determinant is 0.
   ///
   /// Note: In most cases you want to use the normal inverse function.  This method should
   ///       be used if the matrix has something other than (0,0,0,1) in the bottom row.
   bool fullInverse();

   /// Reverse depth for projection matrix
   /// Simplifies reversal matrix mult to 4 subtractions
   void reverseProjection();
   /// Swaps rows and columns into matrix.
   void transposeTo(F32 *matrix) const;

   /// Normalize the matrix.
   void normalize();

   /// Copy the requested column into a Point4F.
   void getColumn(S32 col, Point4F *cptr) const;
   Point4F getColumn4F(S32 col) const { Point4F ret; getColumn(col,&ret); return ret; }

   /// Copy the requested column into a Point3F.
   ///
   /// This drops the bottom-most row.
   void getColumn(S32 col, Point3F *cptr) const;
   Point3F getColumn3F(S32 col) const { Point3F ret; getColumn(col,&ret); return ret; }

   /// Set the specified column from a Point4F.
   void setColumn(S32 col, const Point4F& cptr);

   /// Set the specified column from a Point3F.
   ///
   /// The bottom-most row is not set.
   void setColumn(S32 col, const Point3F& cptr);

   /// Copy the specified row into a Point4F.
   void getRow(S32 row, Point4F *cptr) const;
   Point4F getRow4F(S32 row) const { Point4F ret; getRow(row,&ret); return ret; }

   /// Copy the specified row into a Point3F.
   ///
   /// Right-most item is dropped.
   void getRow(S32 row, Point3F *cptr) const;
   Point3F getRow3F(S32 row) const { Point3F ret; getRow(row,&ret); return ret; }

   /// Set the specified row from a Point4F.
   void setRow(S32 row, const Point4F& cptr);

   /// Set the specified row from a Point3F.
   ///
   /// The right-most item is not set.
   void setRow(S32 row, const Point3F& cptr);

   /// Get the position of the matrix.
   ///
   /// This is the 4th column of the matrix.
   Point3F getPosition() const;

   /// Set the position of the matrix.
   ///
   /// This is the 4th column of the matrix.
   void setPosition( const Point3F &pos ) { setColumn( 3, pos ); }

   /// Add the passed delta to the matrix position.
   void displace( const Point3F &delta );

   /// Get the x axis of the matrix.
   ///
   /// This is the 1st column of the matrix and is
   /// normally considered the right vector.
   VectorF getRightVector() const;

   /// Get the y axis of the matrix.
   ///
   /// This is the 2nd column of the matrix and is
   /// normally considered the forward vector.   
   VectorF getForwardVector() const;   

   /// Get the z axis of the matrix.
   ///
   /// This is the 3rd column of the matrix and is
   /// normally considered the up vector.   
   VectorF getUpVector() const;

   MatrixF&  mul(const MatrixF &a);                    ///< M * a -> M
   MatrixF&  mulL(const MatrixF &a);                   ///< a * M -> M
   MatrixF&  mul(const MatrixF &a, const MatrixF &b);  ///< a * b -> M

   // Scalar multiplies
   MatrixF&  mul(const F32 a);                         ///< M * a -> M
   MatrixF&  mul(const MatrixF &a, const F32 b);       ///< a * b -> M


   void mul( Point4F& p ) const;                       ///< M * p -> p (full [4x4] * [1x4])
   void mulP( Point3F& p ) const;                      ///< M * p -> p (assume w = 1.0f)
   void mulP( const Point3F &p, Point3F *d) const;     ///< M * p -> d (assume w = 1.0f)
   void mulV( VectorF& p ) const;                      ///< M * v -> v (assume w = 0.0f)
   void mulV( const VectorF &p, Point3F *d) const;     ///< M * v -> d (assume w = 0.0f)

   void mul(Box3F& b) const;                           ///< Axial box -> Axial Box
   
   MatrixF& add( const MatrixF& m );

   /// Convenience function to allow people to treat this like an array.
   F32& operator ()(S32 row, S32 col) { return m[idx(col,row)]; }
   F32 operator ()(S32 row, S32 col) const { return m[idx(col,row)]; }

   void dumpMatrix(const char *caption=NULL) const;

   // Math operator overloads
   //------------------------------------
   friend MatrixF operator * ( const MatrixF &m1, const MatrixF &m2 );
   MatrixF& operator *= ( const MatrixF &m );
   MatrixF &operator = (const MatrixF &m);
   bool isNaN();
   // Static identity matrix
   const static MatrixF Identity;
};

class MatrixFEngineExport
{
public:
   static EngineFieldTable::Field getMatrixField();
};


//--------------------------------------
// Inline Functions

inline MatrixF::MatrixF(bool _identity)
{
   if (_identity)
      identity();
   else
      std::fill_n(m, 16, 0);
}

inline MatrixF::MatrixF( const EulerF &e )
{
   set(e);
}

inline MatrixF::MatrixF( const EulerF &e, const Point3F& p )
{
   set(e,p);
}

inline MatrixF& MatrixF::set( const EulerF &e)
{
   m_matF_set_euler( e, *this );
   return (*this);
}


inline MatrixF& MatrixF::set( const EulerF &e, const Point3F& p)
{
   m_matF_set_euler_point( e, p, *this );
   return (*this);
}

inline MatrixF& MatrixF::setCrossProduct( const Point3F &p)
{
   m[1] = -(m[4] = p.z);
   m[8] = -(m[2] = p.y);
   m[6] = -(m[9] = p.x);
   m[0] = m[3] = m[5] = m[7] = m[10] = m[11] =
      m[12] = m[13] = m[14] = 0.0f;
   m[15] = 1;
   return (*this);
}

inline MatrixF& MatrixF::setTensorProduct( const Point3F &p, const Point3F &q)
{
   m[0] = p.x * q.x;
   m[1] = p.x * q.y;
   m[2] = p.x * q.z;
   m[4] = p.y * q.x;
   m[5] = p.y * q.y;
   m[6] = p.y * q.z;
   m[8] = p.z * q.x;
   m[9] = p.z * q.y;
   m[10] = p.z * q.z;
   m[3] = m[7] = m[11] = m[12] = m[13] = m[14] = 0.0f;
   m[15] = 1.0f;
   return (*this);
}

inline bool MatrixF::isIdentity() const
{
   return
   m[0]  == 1.0f &&
   m[1]  == 0.0f &&
   m[2]  == 0.0f &&
   m[3]  == 0.0f &&
   m[4]  == 0.0f &&
   m[5]  == 1.0f &&
   m[6]  == 0.0f &&
   m[7]  == 0.0f &&
   m[8]  == 0.0f &&
   m[9]  == 0.0f &&
   m[10] == 1.0f &&
   m[11] == 0.0f &&
   m[12] == 0.0f &&
   m[13] == 0.0f &&
   m[14] == 0.0f &&
   m[15] == 1.0f;
}

inline MatrixF& MatrixF::identity()
{
   m[0]  = 1.0f;
   m[1]  = 0.0f;
   m[2]  = 0.0f;
   m[3]  = 0.0f;
   m[4]  = 0.0f;
   m[5]  = 1.0f;
   m[6]  = 0.0f;
   m[7]  = 0.0f;
   m[8]  = 0.0f;
   m[9]  = 0.0f;
   m[10] = 1.0f;
   m[11] = 0.0f;
   m[12] = 0.0f;
   m[13] = 0.0f;
   m[14] = 0.0f;
   m[15] = 1.0f;
   return (*this);
}


inline MatrixF& MatrixF::inverse()
{
   m_matF_inverse(m);
   return (*this);
}

inline void MatrixF::invertTo( MatrixF *out )
{
   m_matF_invert_to(m,*out);
}

inline MatrixF& MatrixF::affineInverse()
{
//   AssertFatal(isAffine() == true, "Error, this matrix is not an affine transform");
   m_matF_affineInverse(m);
   return (*this);
}

inline MatrixF& MatrixF::transpose()
{
   m_matF_transpose(m);
   return (*this);
}

inline MatrixF& MatrixF::scale(const Point3F& p)
{
   m_matF_scale(m,p);
   return *this;
}

inline Point3F MatrixF::getScale() const
{
   Point3F scale;
   scale.x = mSqrt(m[0]*m[0] + m[4] * m[4] + m[8] * m[8]);
   scale.y = mSqrt(m[1]*m[1] + m[5] * m[5] + m[9] * m[9]);
   scale.z = mSqrt(m[2]*m[2] + m[6] * m[6] + m[10] * m[10]);
   return scale;
}

inline void MatrixF::normalize()
{
   m_matF_normalize(m);
}

inline MatrixF& MatrixF::mul( const MatrixF &a )
{  // M * a -> M
   AssertFatal(&a != this, "MatrixF::mul - a.mul(a) is invalid!");

   MatrixF tempThis(*this);
   m_matF_x_matF(tempThis, a, *this);
   return (*this);
}

inline MatrixF& MatrixF::mulL( const MatrixF &a )
{  // a * M -> M
   AssertFatal(&a != this, "MatrixF::mulL - a.mul(a) is invalid!");

   MatrixF tempThis(*this);
   m_matF_x_matF(a, tempThis, *this);
   return (*this);
}

inline MatrixF& MatrixF::mul( const MatrixF &a, const MatrixF &b )
{  // a * b -> M
   AssertFatal((&a != this) && (&b != this), "MatrixF::mul - a.mul(a, b) a.mul(b, a) a.mul(a, a) is invalid!");

   m_matF_x_matF(a, b, *this);
   return (*this);
}


inline MatrixF& MatrixF::mul(const F32 a)
{
   for (U32 i = 0; i < 16; i++)
      m[i] *= a;

   return *this;
}


inline MatrixF& MatrixF::mul(const MatrixF &a, const F32 b)
{
   *this = a;
   mul(b);

   return *this;
}

inline void MatrixF::mul( Point4F& p ) const
{
   Point4F temp;
   m_matF_x_point4F(*this, &p.x, &temp.x);
   p = temp;
}

inline void MatrixF::mulP( Point3F& p) const
{
   // M * p -> d
   Point3F d;
   m_matF_x_point3F(*this, &p.x, &d.x);
   p = d;
}

inline void MatrixF::mulP( const Point3F &p, Point3F *d) const
{
   // M * p -> d
   m_matF_x_point3F(*this, &p.x, &d->x);
}

inline void MatrixF::mulV( VectorF& v) const
{
   // M * v -> v
   VectorF temp;
   m_matF_x_vectorF(*this, &v.x, &temp.x);
   v = temp;
}

inline void MatrixF::mulV( const VectorF &v, Point3F *d) const
{
   // M * v -> d
   m_matF_x_vectorF(*this, &v.x, &d->x);
}

inline void MatrixF::mul(Box3F& b) const
{
   m_matF_x_box3F(*this, &b.minExtents.x, &b.maxExtents.x);
}

inline MatrixF& MatrixF::add( const MatrixF& a )
{
   for( U32 i = 0; i < 16; ++ i )
      m[ i ] += a.m[ i ];
      
   return *this;
}

inline void MatrixF::getColumn(S32 col, Point4F *cptr) const
{
   cptr->x = m[col];
   cptr->y = m[col+4];
   cptr->z = m[col+8];
   cptr->w = m[col+12];
}

inline void MatrixF::getColumn(S32 col, Point3F *cptr) const
{
   cptr->x = m[col];
   cptr->y = m[col+4];
   cptr->z = m[col+8];
}

inline void MatrixF::setColumn(S32 col, const Point4F &cptr)
{
   m[col]   = cptr.x;
   m[col+4] = cptr.y;
   m[col+8] = cptr.z;
   m[col+12]= cptr.w;
}

inline void MatrixF::setColumn(S32 col, const Point3F &cptr)
{
   m[col]   = cptr.x;
   m[col+4] = cptr.y;
   m[col+8] = cptr.z;
}


inline void MatrixF::getRow(S32 col, Point4F *cptr) const
{
   col *= 4;
   cptr->x = m[col++];
   cptr->y = m[col++];
   cptr->z = m[col++];
   cptr->w = m[col];
}

inline void MatrixF::getRow(S32 col, Point3F *cptr) const
{
   col *= 4;
   cptr->x = m[col++];
   cptr->y = m[col++];
   cptr->z = m[col];
}

inline void MatrixF::setRow(S32 col, const Point4F &cptr)
{
   col *= 4;
   m[col++] = cptr.x;
   m[col++] = cptr.y;
   m[col++] = cptr.z;
   m[col]   = cptr.w;
}

inline void MatrixF::setRow(S32 col, const Point3F &cptr)
{
   col *= 4;
   m[col++] = cptr.x;
   m[col++] = cptr.y;
   m[col]   = cptr.z;
}

inline Point3F MatrixF::getPosition() const
{
   return Point3F( m[3], m[3+4], m[3+8] );
}

inline void MatrixF::displace( const Point3F &delta )
{
   m[3]   += delta.x;
   m[3+4] += delta.y;
   m[3+8] += delta.z;
}

inline VectorF MatrixF::getForwardVector() const
{
   VectorF vec;
   getColumn( 1, &vec );
   return vec;
}

inline VectorF MatrixF::getRightVector() const
{
   VectorF vec;
   getColumn( 0, &vec );
   return vec;
}

inline VectorF MatrixF::getUpVector() const
{
   VectorF vec;
   getColumn( 2, &vec );
   return vec;
}

//------------------------------------
// Math operator overloads
//------------------------------------
inline MatrixF operator * ( const MatrixF &m1, const MatrixF &m2 )
{
   // temp = m1 * m2
   MatrixF temp;
   m_matF_x_matF(m1, m2, temp);
   return temp;
}

inline MatrixF& MatrixF::operator *= ( const MatrixF &m1 )
{
   MatrixF tempThis(*this);
   m_matF_x_matF(tempThis, m1, *this);
   return (*this);
}

inline MatrixF &MatrixF::operator = (const MatrixF &m1)
{
   for (U32 i=0;i<16;i++)
   this->m[i] = m1.m[i];
   return (*this);
}
inline bool MatrixF::isNaN()
{
   bool isaNaN = false;
   for (U32 i = 0; i < 16; i++)
      if (mIsNaN_F(m[i]))
         isaNaN = true;
   return isaNaN;
}
//------------------------------------
// Non-member methods
//------------------------------------

inline void mTransformPlane(const MatrixF& mat, const Point3F& scale, const PlaneF&  plane, PlaneF * result)
{
   m_matF_x_scale_x_planeF(mat, &scale.x, &plane.x, &result->x);
}

//------------------------------------
// Templatized matrix class to replace MATRIXF above
//------------------------------------

template<typename DATA_TYPE, U32 rows, U32 cols>
class Matrix {
   friend class MatrixTemplateExport;
private:
   DATA_TYPE data[rows * cols];

public:

   static_assert(rows >= 2 && cols >= 2, "Matrix must have at least 2 rows and 2 cols.");

   // ------ Setters and initializers ------
   explicit Matrix(bool identity = false) {
      std::fill(data, data + (rows * cols), DATA_TYPE(0));

      if (identity) {
         for (U32 i = 0; i < rows; i++) {
            for (U32 j = 0; j < cols; j++) {
               // others already get filled with 0
               if (j == i)
                  (*this)(i, j) = static_cast<DATA_TYPE>(1);
            }
         }
      }
   }

   explicit Matrix(const EulerF& e);
   /// Make this an identity matrix.
   Matrix<DATA_TYPE, rows, cols>& identity();

   Matrix<DATA_TYPE, rows, cols>& set(const EulerF& e);

   Matrix(const EulerF& e, const Point3F p);
   Matrix<DATA_TYPE, rows, cols>& set(const EulerF& e, const Point3F p);

   Matrix<DATA_TYPE, rows, cols>& inverse();
   Matrix<DATA_TYPE, rows, cols>& transpose();
   void invert();

   Matrix<DATA_TYPE, rows, cols>& setCrossProduct(const Point3F& p);
   Matrix<DATA_TYPE, rows, cols>& setTensorProduct(const Point3F& p, const Point3F& q);

   /// M * Matrix(p) -> M
   Matrix<DATA_TYPE, rows, cols>& scale(const Point3F& s);
   Matrix<DATA_TYPE, rows, cols>& scale(DATA_TYPE s) { return scale(Point3F(s, s, s)); }

   void setColumn(S32 col, const Point4F& cptr);
   void setColumn(S32 col, const Point3F& cptr);
   
   void setRow(S32 row, const Point4F& cptr);
   void setRow(S32 row, const Point3F& cptr);

   ///< M * a -> M
   Matrix<DATA_TYPE, rows, cols>& mul(const Matrix<DATA_TYPE, rows, cols>& a)
   { return *this * a; }
   ///< a * M -> M
   Matrix<DATA_TYPE, rows, cols>& mulL(const Matrix<DATA_TYPE, rows, cols>& a)
   { return *this = a * *this; }
   ///< a * b -> M
   Matrix<DATA_TYPE, rows, cols>& mul(const Matrix<DATA_TYPE, rows, cols>& a, const Matrix<DATA_TYPE, rows, cols>& b)
   { return *this = a * b; }
   ///< M * a -> M
   Matrix<DATA_TYPE, rows, cols>& mul(const F32 a)
   { return *this * a; }
   ///< a * b -> M
   Matrix<DATA_TYPE, rows, cols>& mul(const Matrix<DATA_TYPE, rows, cols>& a, const F32 b)
   { return *this = a * b; }


   ///< M * p -> p (full [4x4] * [1x4])
   void mul(Point4F& p) const { p = *this * p; }
   ///< M * p -> p (assume w = 1.0f)
   void mulP(Point3F& p) const { p = *this * p; }
   ///< M * p -> d (assume w = 1.0f)
   void mulP(const Point3F& p, Point3F* d) const { *d = *this * p; }
   ///< M * v -> v (assume w = 0.0f)
   void mulV(VectorF& v) const
   {
      AssertFatal(rows == 4 && cols == 4, "Multiplying VectorF with matrix requires 4x4");
      VectorF result(
         (*this)(0, 0) * v.x + (*this)(0, 1) * v.y + (*this)(0, 2) * v.z,
         (*this)(1, 0) * v.x + (*this)(1, 1) * v.y + (*this)(1, 2) * v.z,
         (*this)(2, 0) * v.x + (*this)(2, 1) * v.y + (*this)(2, 2) * v.z
      );

      v = result;

   }
   ///< M * v -> d (assume w = 0.0f)
   void mulV(const VectorF& v, Point3F* d) const
   {
      AssertFatal(rows == 4 && cols == 4, "Multiplying VectorF with matrix requires 4x4");
      VectorF result(
         (*this)(0, 0) * v.x + (*this)(0, 1) * v.y + (*this)(0, 2) * v.z,
         (*this)(1, 0) * v.x + (*this)(1, 1) * v.y + (*this)(1, 2) * v.z,
         (*this)(2, 0) * v.x + (*this)(2, 1) * v.y + (*this)(2, 2) * v.z
      );

      d->x = result.x;
      d->y = result.y;
      d->z = result.z;
   }

   ///< Axial box -> Axial Box (too big a function to be inline)
   void mul(Box3F& box) const;

   // ------ Getters ------
   bool isAffine() const;
   bool isIdentity() const;
   Point3F getScale() const;
   
   EulerF toEuler() const;

   Point3F getPosition() const;

   void getColumn(S32 col, Point4F* cptr) const;
   Point4F getColumn4F(S32 col) const { Point4F ret; getColumn(col, &ret); return ret; }

   void getColumn(S32 col, Point3F* cptr) const;
   Point3F getColumn3F(S32 col) const { Point3F ret; getColumn(col, &ret); return ret; }

   void getRow(S32 row, Point4F* cptr) const;
   Point4F getRow4F(S32 row) const { Point4F ret; getRow(row, &ret); return ret; }

   void getRow(S32 row, Point3F* cptr) const;
   Point3F getRow3F(S32 row) const { Point3F ret; getRow(row, &ret); return ret; }

   DATA_TYPE* getData() {
      return data;
   }

   const DATA_TYPE* getData() const {
      return data;
   }

   void dumpMatrix(const char* caption = NULL) const;
   // Static identity matrix
   static const Matrix Identity;

   // ------ Operators ------

   Matrix<DATA_TYPE, rows, cols> operator * (const Matrix<DATA_TYPE, rows, cols>& other) const {
      Matrix<DATA_TYPE, rows, cols> result;

      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            result(i, j) = 0;
            for (U32 k = 0; k < cols; k++) {
               result(i, j) += (*this)(i, k) * other(k, j);
            }
         }
      }

      return result;
   }

   Matrix<DATA_TYPE, rows, cols> operator *= (const Matrix<DATA_TYPE, rows, cols>& other) {
      *this = *this * other;
      return *this;
   }

   Matrix<DATA_TYPE, rows, cols> operator * (const DATA_TYPE scalar) const {
      Matrix<DATA_TYPE, rows, cols> result;
      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            result(i, j) = (*this)(i, j) * scalar;
         }
      }

      return result;
   }
   Matrix<DATA_TYPE, rows, cols>& operator *= (const DATA_TYPE scalar) {
      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            (*this)(i, j) *= scalar;
         }
      }

      return *this;
   }

   Point3F operator*(const Point3F& point) const {
      AssertFatal(rows == 4 && cols == 4, "Multiplying point3 with matrix requires 4x4");
      return Point3F(
         (*this)(0, 0) * point.x + (*this)(0, 1) * point.y + (*this)(0, 2) * point.z + (*this)(0, 3),
         (*this)(1, 0) * point.x + (*this)(1, 1) * point.y + (*this)(1, 2) * point.z + (*this)(1, 3),
         (*this)(2, 0) * point.x + (*this)(2, 1) * point.y + (*this)(2, 2) * point.z + (*this)(2, 3)
      );
   }

   Point4F operator*(const Point4F& point) const {
      AssertFatal(rows == 4 && cols == 4, "Multiplying point4 with matrix requires 4x4");
      return Point4F(
         (*this)(0, 0) * point.x + (*this)(0, 1) * point.y + (*this)(0, 2) * point.z + (*this)(0, 3) * point.w,
         (*this)(1, 0) * point.x + (*this)(1, 1) * point.y + (*this)(1, 2) * point.z + (*this)(1, 3) * point.w,
         (*this)(2, 0) * point.x + (*this)(2, 1) * point.y + (*this)(2, 2) * point.z + (*this)(2, 3) * point.w,
         (*this)(3, 0) * point.x + (*this)(3, 1) * point.y + (*this)(3, 2) * point.z + (*this)(3, 3) * point.w
      );
   }

   Matrix<DATA_TYPE, rows, cols>& operator = (const Matrix<DATA_TYPE, rows, cols>& other) {
      if (this != &other) {
         std::copy(other.data, other.data + rows * cols, this->data);
      }

      return *this;
   }

   bool operator == (const Matrix<DATA_TYPE, rows, cols>& other) const {
      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            if ((*this)(i, j) != other(i, j))
               return false;
         }
      }
      return true;
   }

   bool operator != (const Matrix<DATA_TYPE, rows, cols>& other) const {
      return !(*this == other);
   }

   operator DATA_TYPE* () { return (data); }
   operator const DATA_TYPE* () const { return (DATA_TYPE*)(data); }

   DATA_TYPE& operator () (U32 row, U32 col) {
      if (row >= rows || col >= cols)
         AssertFatal(false, "Matrix indices out of range");

      return data[col * rows + row];
   }

   const DATA_TYPE& operator () (U32 row, U32 col) const {
      if (row >= rows || col >= cols)
         AssertFatal(false, "Matrix indices out of range");

      return data[col * rows + row];
   }

};

//--------------------------------------------
// INLINE FUNCTIONS
//--------------------------------------------
template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::transpose()
{
   // square matrices can just swap, non square requires a temp mat.
   if (rows == cols) {
      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            std::swap((*this)(j, i), (*this)(i, j));
         }
      }
   }
   else {
      Matrix<DATA_TYPE, rows, cols> result;
      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            result(j, i) = (*this)(i, j);
         }
      }
      std::copy(std::begin(result.data), std::end(result.data), std::begin(data));
   }

   return (*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::identity()
{
   for (U32 i = 0; i < rows; i++) {
      for (U32 j = 0; j < cols; j++) {
         if (j == i)
            (*this)(i, j) = static_cast<DATA_TYPE>(1);
         else
            (*this)(i, j) = static_cast<DATA_TYPE>(0);
      }
   }

   return (*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::scale(const Point3F& s)
{
   // torques scale applies directly, does not create another matrix to multiply with the translation matrix.
   AssertFatal(rows >= 3 && cols >= 3, "Scale can only be applied 3x3 or more");
   for (U32 i = 0; i < 3; i++) {
      for (U32 j = 0; j < 3; j++) {
         DATA_TYPE scale = (i == 0) ? s.x : (i == 1) ? s.y : s.z;
         (*this)(i, j) *= scale;
      }
   }

   return (*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline bool Matrix<DATA_TYPE, rows, cols>::isIdentity() const {
   for (U32 i = 0; i < rows; i++) {
      for (U32 j = 0; j < cols; j++) {
         if (j == i) {
            if((*this)(i, j) != static_cast<DATA_TYPE>(1)) {
               return false;
            }
         }
         else {
            if((*this)(i, j) != static_cast<DATA_TYPE>(0)) {
               return false;
            }
         }
      }
   }
   
   return true;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline Point3F Matrix<DATA_TYPE, rows, cols>::getScale() const
{
   // this function assumes the matrix has scale applied through the scale(const Point3F& s) function.
   // for now assume float since we have point3F.
   AssertFatal(rows >= 3 && cols >= 3, "Scale can only be applied 3x3 or more");

   Point3F scale;

   scale.x = mSqrt((*this)(0, 0) * (*this)(0, 0) + (*this)(1, 0) * (*this)(1, 0) + (*this)(2, 0) * (*this)(2, 0));
   scale.y = mSqrt((*this)(0, 1) * (*this)(0, 1) + (*this)(1, 1) * (*this)(1, 1) + (*this)(2, 1) * (*this)(2, 1));
   scale.z = mSqrt((*this)(0, 2) * (*this)(0, 2) + (*this)(1, 2) * (*this)(1, 2) + (*this)(2, 2) * (*this)(2, 2));

   return scale;
}



template<typename DATA_TYPE, U32 rows, U32 cols>
inline Point3F Matrix<DATA_TYPE, rows, cols>::getPosition() const
{
   Point3F pos;
   getColumn(3, &pos);
   return pos;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::getColumn(S32 col, Point4F* cptr) const
{
   if (rows >= 2)
   {
      cptr->x = (*this)(0, col);
      cptr->y = (*this)(1, col);
   }

   if (rows >= 3)
      cptr->z = (*this)(2, col);
   else
      cptr->z = 0.0f;

   if (rows >= 4)
      cptr->w = (*this)(3, col);
   else
      cptr->w = 0.0f;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::getColumn(S32 col, Point3F* cptr) const
{
   if (rows >= 2)
   {
      cptr->x = (*this)(0, col);
      cptr->y = (*this)(1, col);
   }

   if (rows >= 3)
      cptr->z = (*this)(2, col);
   else
      cptr->z = 0.0f;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::setColumn(S32 col, const Point4F &cptr) {
   if(rows >= 2)
   {
      (*this)(0, col) = cptr.x;
      (*this)(1, col) = cptr.y;
   }
   
   if(rows >= 3)
      (*this)(2, col) = cptr.z;
   
   if(rows >= 4)
      (*this)(3, col) = cptr.w;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::setColumn(S32 col, const Point3F &cptr) {
   if(rows >= 2)
   {
      (*this)(0, col) = cptr.x;
      (*this)(1, col) = cptr.y;
   }
   
   if(rows >= 3)
      (*this)(2, col) = cptr.z;
   
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::getRow(S32 row, Point4F* cptr) const
{
   if (cols >= 2)
   {
      cptr->x = (*this)(row, 0);
      cptr->y = (*this)(row, 1);
   }

   if (cols >= 3)
      cptr->z = (*this)(row, 2);
   else
      cptr->z = 0.0f;

   if (cols >= 4)
      cptr->w = (*this)(row, 3);
   else
      cptr->w = 0.0f;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::getRow(S32 row, Point3F* cptr) const
{
   if (cols >= 2)
   {
      cptr->x = (*this)(row, 0);
      cptr->y = (*this)(row, 1);
   }

   if (cols >= 3)
      cptr->z = (*this)(row, 2);
   else
      cptr->z = 0.0f;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::setRow(S32 row, const Point4F& cptr) {
   if(cols >= 2)
   {
      (*this)(row, 0) = cptr.x;
      (*this)(row, 1) = cptr.y;
   }
   
   if(cols >= 3)
      (*this)(row, 2) = cptr.z;
   
   if(cols >= 4)
      (*this)(row, 3) = cptr.w;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::setRow(S32 row, const Point3F& cptr) {
   if(cols >= 2)
   {
      (*this)(row, 0) = cptr.x;
      (*this)(row, 1) = cptr.y;
   }
   
   if(cols >= 3)
      (*this)(row, 2) = cptr.z;
   
}

//--------------------------------------------
// INLINE FUNCTIONS END
//--------------------------------------------

typedef Matrix<F32, 4, 4> Matrix4F;

class MatrixTemplateExport
{
public:
   template <typename T, U32 rows, U32 cols>
   static EngineFieldTable::Field getMatrixField();
};

template<typename T, U32 rows, U32 cols>
inline EngineFieldTable::Field MatrixTemplateExport::getMatrixField()
{
   typedef Matrix<T, rows, cols> ThisType;
   return _FIELD_AS(T, data, data, rows * cols, "");
}



#endif //_MMATRIX_H_
