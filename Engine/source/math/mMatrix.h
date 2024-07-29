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

#ifndef _FRAMEALLOCATOR_H_
#include "core/frameAllocator.h"
#endif

#ifndef _STRINGFUNCTIONS_H_
#include "core/strings/stringFunctions.h"
#endif

#ifndef _CONSOLE_H_
#include "console/console.h"
#endif

#ifndef USE_TEMPLATE_MATRIX

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

#else // !USE_TEMPLATE_MATRIX

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
   void reverseProjection();
   void normalize();

   Matrix<DATA_TYPE, rows, cols>& set(const EulerF& e);

   Matrix(const EulerF& e, const Point3F p);
   Matrix<DATA_TYPE, rows, cols>& set(const EulerF& e, const Point3F p);

   Matrix<DATA_TYPE, rows, cols> inverse();
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
   void displace(const Point3F& delta);
   bool fullInverse();

   void setPosition(const Point3F& pos) { setColumn(3, pos); }

   ///< M * a -> M
   Matrix<DATA_TYPE, rows, cols>& mul(const Matrix<DATA_TYPE, rows, cols>& a)
   {
      *this = *this * a;  return *this;
   }
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
   bool isNaN() {
      for (U32 i = 0; i < rows; i++) {
         for (U32 j = 0; j < cols; j++) {
            if (mIsNaN_F((*this)(i, j)))
               return true;
         }
      }

      return false;
   }
   // row + col * cols
   static U32 idx(U32 i, U32 j) { return (i + j * cols); }
   bool isAffine() const;
   bool isIdentity() const;
   /// Take inverse of matrix assuming it is affine (rotation,
   /// scale, sheer, translation only).
   Matrix<DATA_TYPE, rows, cols> affineInverse();

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

   VectorF getRightVector() const;
   VectorF getForwardVector() const;
   VectorF getUpVector() const;

   DATA_TYPE* getData() {
      return data;
   }

   const DATA_TYPE* getData() const {
      return data;
   }

   void transposeTo(Matrix<DATA_TYPE, cols, rows>& matrix) const {
      for (U32 i = 0; i < rows; ++i) {
         for (U32 j = 0; j < cols; ++j) {
            matrix(j, i) = (*this)(i, j);
         }
      }
   }

   void invertTo(Matrix<DATA_TYPE, cols, rows>* matrix) const;
   void invertTo(Matrix<DATA_TYPE, cols, rows>* matrix);

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

      return data[idx(col,row)];
   }

   const DATA_TYPE& operator () (U32 row, U32 col) const {
      if (row >= rows || col >= cols)
         AssertFatal(false, "Matrix indices out of range");

      return data[idx(col, row)];
   }

};

//--------------------------------------------
// INLINE FUNCTIONS
//--------------------------------------------
template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::transpose()
{
   Matrix<DATA_TYPE, rows, cols> result;
   for (U32 i = 0; i < rows; i++) {
      for (U32 j = 0; j < cols; j++) {
         result(j, i) = (*this)(i, j);
      }
   }
   std::copy(std::begin(result.data), std::end(result.data), std::begin(data));

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
inline void Matrix<DATA_TYPE, rows, cols>::normalize()
{
   AssertFatal(rows >= 3 && cols >= 3, "Normalize can only be applied 3x3 or more");
   Point3F col0, col1, col2;
   getColumn(0, &col0);
   getColumn(1, &col1);

   mCross(col0, col1, &col2);
   mCross(col2, col0, &col1);

   col0.normalize();
   col1.normalize();
   col2.normalize();

   setColumn(0, col0);
   setColumn(1, col1);
   setColumn(2, col2);
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
inline VectorF Matrix<DATA_TYPE, rows, cols>::getRightVector() const
{
   VectorF vec;
   getColumn(0, &vec);
   return vec;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline VectorF Matrix<DATA_TYPE, rows, cols>::getForwardVector() const
{
   VectorF vec;
   getColumn(1, &vec);
   return vec;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline VectorF Matrix<DATA_TYPE, rows, cols>::getUpVector() const
{
   VectorF vec;
   getColumn(2, &vec);
   return vec;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::invertTo(Matrix<DATA_TYPE, cols, rows>* matrix) const
{
   Matrix<DATA_TYPE, rows, cols> invMatrix;
   for (U32 i = 0; i < rows; ++i) {
      for (U32 j = 0; j < cols; ++j) {
         invMatrix(i, j) = (*this)(i, j);
      }
   }

   invMatrix.inverse();

   for (U32 i = 0; i < rows; ++i) {
      for (U32 j = 0; j < cols; ++j) {
         (*matrix)(i, j) = invMatrix(i, j);
      }
   }
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::invertTo(Matrix<DATA_TYPE, cols, rows>* matrix)
{
   Matrix<DATA_TYPE, rows, cols> invMatrix = this->inverse();

   for (U32 i = 0; i < rows; ++i) {
      for (U32 j = 0; j < cols; ++j) {
         (*matrix)(i, j) = invMatrix(i, j);
      }
   }
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

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::displace(const Point3F& delta)
{
   (*this)(0, 3) += delta.x;
   (*this)(1, 3) += delta.y;
   (*this)(2, 3) += delta.z;
}


template<typename DATA_TYPE, U32 rows, U32 cols>
void Matrix<DATA_TYPE, rows, cols>::reverseProjection()
{
   AssertFatal(rows == 4 && cols == 4, "reverseProjection requires a 4x4 matrix.");

   (*this)(2, 0) = (*this)(3, 0) - (*this)(2, 0);
   (*this)(2, 1) = (*this)(3, 1) - (*this)(2, 1);
   (*this)(2, 2) = (*this)(3, 2) - (*this)(2, 2);
   (*this)(2, 3) = (*this)(3, 3) - (*this)(2, 3);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
const Matrix<DATA_TYPE, rows, cols> Matrix<DATA_TYPE, rows, cols>::Identity = []() {
   Matrix<DATA_TYPE, rows, cols> identity(true);
   return identity;
}();

template<typename DATA_TYPE, U32 rows, U32 cols>
Matrix<DATA_TYPE, rows, cols>::Matrix(const EulerF& e)
{
   set(e);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::set(const EulerF& e)
{
   // when the template refactor is done, euler will be able to be setup in different ways
   AssertFatal(rows >= 3 && cols >= 3, "EulerF can only initialize 3x3 or more");
   static_assert(std::is_same<DATA_TYPE, float>::value, "Can only initialize eulers with floats for now");

   F32 cosPitch, sinPitch;
   mSinCos(e.x, sinPitch, cosPitch);

   F32 cosYaw, sinYaw;
   mSinCos(e.y, sinYaw, cosYaw);

   F32 cosRoll, sinRoll;
   mSinCos(e.z, sinRoll, cosRoll);

   enum {
      AXIS_X = (1 << 0),
      AXIS_Y = (1 << 1),
      AXIS_Z = (1 << 2)
   };

   U32 axis = 0;
   if (e.x != 0.0f) axis |= AXIS_X;
   if (e.y != 0.0f) axis |= AXIS_Y;
   if (e.z != 0.0f) axis |= AXIS_Z;

   switch (axis) {
   case 0:
      (*this) = Matrix<DATA_TYPE, rows, cols>(true);
      break;
   case AXIS_X:
      (*this)(0, 0) = 1.0f;  (*this)(1, 0) = 0.0f;       (*this)(2, 0) = 0.0f;
      (*this)(0, 1) = 0.0f;  (*this)(1, 1) = cosPitch;   (*this)(2, 1) = -sinPitch;
      (*this)(0, 2) = 0.0f;  (*this)(1, 2) = sinPitch;   (*this)(2, 2) = cosPitch;
      break;
   case AXIS_Y:
      (*this)(0, 0) = cosYaw;    (*this)(1, 0) = 0.0f;   (*this)(2, 0) = sinYaw;
      (*this)(0, 1) = 0.0f;      (*this)(1, 1) = 1.0f;   (*this)(2, 1) = 0.0f;
      (*this)(0, 2) = -sinYaw;   (*this)(1, 2) = 0.0f;   (*this)(2, 2) = cosYaw;
      break;
   case AXIS_Z:
      (*this)(0, 0) = cosRoll;   (*this)(1, 0) = -sinRoll;  (*this)(2, 0) = 0.0f;
      (*this)(0, 1) = sinRoll;   (*this)(1, 1) = cosRoll;   (*this)(2, 1) = 0.0f;
      (*this)(0, 2) = 0.0f;      (*this)(1, 2) = 0.0f;      (*this)(2, 2) = 0.0f;
      break;
   default:
      F32 r1 = cosYaw * cosRoll;
      F32 r2 = cosYaw * sinRoll;
      F32 r3 = sinYaw * cosRoll;
      F32 r4 = sinYaw * sinRoll;

      // the matrix looks like this:
      //  r1 - (r4 * sin(x))     r2 + (r3 * sin(x))   -cos(x) * sin(y)
      //  -cos(x) * sin(z)       cos(x) * cos(z)      sin(x)
      //  r3 + (r2 * sin(x))     r4 - (r1 * sin(x))   cos(x) * cos(y)
      //
      // where:
      //  r1 = cos(y) * cos(z)
      //  r2 = cos(y) * sin(z)
      //  r3 = sin(y) * cos(z)
      //  r4 = sin(y) * sin(z)

      // init the euler 3x3 rotation matrix.
      (*this)(0, 0) = r1 - (r4 * sinPitch);  (*this)(1, 0) = -cosPitch * sinRoll;   (*this)(2, 0) = r3 + (r2 * sinPitch);
      (*this)(0, 1) = r2 + (r3 * sinPitch);  (*this)(1, 1) = cosPitch * cosRoll;    (*this)(2, 1) = r4 - (r1 * sinPitch);
      (*this)(0, 2) = -cosPitch * sinYaw;    (*this)(1, 2) = sinPitch;              (*this)(2, 2) = cosPitch * cosYaw;
      break;
   }

   if (rows == 4) {
      (*this)(3, 0) = 0.0f;
      (*this)(3, 1) = 0.0f;
      (*this)(3, 2) = 0.0f;
   }

   if (cols == 4) {
      (*this)(0, 3) = 0.0f;
      (*this)(1, 3) = 0.0f;
      (*this)(2, 3) = 0.0f;
   }

   if (rows == 4 && cols == 4) {
      (*this)(3, 3) = 1.0f;
   }

   return(*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
Matrix<DATA_TYPE, rows, cols>::Matrix(const EulerF& e, const Point3F p)
{
   set(e, p);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::set(const EulerF& e, const Point3F p)
{
   AssertFatal(rows >= 3 && cols >= 4, "Euler and Point can only initialize 3x4 or more");
   // call set euler, this already sets the last row if it exists.
   set(e);

   // does this need to multiply with the result of the euler? or are we just setting position.
   (*this)(0, 3) = p.x;
   (*this)(1, 3) = p.y;
   (*this)(2, 3) = p.z;

   return (*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
Matrix<DATA_TYPE, rows, cols> Matrix<DATA_TYPE, rows, cols>::inverse()
{
   // TODO: insert return statement here
   AssertFatal(rows == cols, "Can only perform inverse on square matrices.");
   const U32 size = rows;

   // Create augmented matrix [this | I]
   Matrix<DATA_TYPE, size, 2 * size> augmentedMatrix;
   Matrix<DATA_TYPE, size, size> resultMatrix;

   for (U32 i = 0; i < size; i++) {
      for (U32 j = 0; j < size; j++) {
         augmentedMatrix(i, j) = (*this)(i, j);
         augmentedMatrix(i, j + size) = (i == j) ? static_cast<DATA_TYPE>(1) : static_cast<DATA_TYPE>(0);
      }
   }

   // Apply gauss-joran elimination
   for (U32 i = 0; i < size; i++) {
      U32 pivotRow = i;

      for (U32 k = i + 1; k < size; k++) {
         // use std::abs until the templated math functions are in place.
         if (std::abs(augmentedMatrix(k, i)) > std::abs(augmentedMatrix(pivotRow, i))) {
            pivotRow = k;
         }
      }

      // Swap if needed.
      if (i != pivotRow) {
         for (U32 j = 0; j < 2 * size; j++) {
            std::swap(augmentedMatrix(i, j), augmentedMatrix(pivotRow, j));
         }
      }

      // Early out if pivot is 0, return identity matrix.
      if (augmentedMatrix(i, i) == static_cast<DATA_TYPE>(0)) {
         return Matrix<DATA_TYPE, rows, cols>(true);
      }

      DATA_TYPE pivotVal = augmentedMatrix(i, i);

      // scale the pivot
      for (U32 j = 0; j < 2 * size; j++) {
         augmentedMatrix(i, j) /= pivotVal;
      }

      // Eliminate the current column in all other rows
      for (U32 k = 0; k < size; k++) {
         if (k != i) {
            DATA_TYPE factor = augmentedMatrix(k, i);
            for (U32 j = 0; j < 2 * size; j++) {
               augmentedMatrix(k, j) -= factor * augmentedMatrix(i, j);
            }
         }
      }
   }

   for (U32 i = 0; i < size; i++) {
      for (U32 j = 0; j < size; j++) {
         resultMatrix(i, j) = augmentedMatrix(i, j + size);
      }
   }

   return resultMatrix;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline bool Matrix<DATA_TYPE, rows, cols>::fullInverse()
{
   Matrix<DATA_TYPE, rows, cols> inv = this->inverse();

   if (inv.isIdentity())
      return false;

   *this = inv;
   return true;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::invert()
{
   (*this) = inverse();
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::setCrossProduct(const Point3F& p)
{
   AssertFatal(rows == 4 && cols == 4, "Cross product only supported on 4x4 for now");

   (*this)(0, 0) = 0;
   (*this)(0, 1) = -p.z;
   (*this)(0, 2) = p.y;
   (*this)(0, 3) = 0;

   (*this)(1, 0) = p.z;
   (*this)(1, 1) = 0;
   (*this)(1, 2) = -p.x;
   (*this)(1, 3) = 0;

   (*this)(2, 0) = -p.y;
   (*this)(2, 1) = p.x;
   (*this)(2, 2) = 0;
   (*this)(2, 3) = 0;

   (*this)(3, 0) = 0;
   (*this)(3, 1) = 0;
   (*this)(3, 2) = 0;
   (*this)(3, 3) = 1;

   return (*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::setTensorProduct(const Point3F& p, const Point3F& q)
{
   AssertFatal(rows == 4 && cols == 4, "Tensor product only supported on 4x4 for now");

   (*this)(0, 0) = p.x * q.x;
   (*this)(0, 1) = p.x * q.y;
   (*this)(0, 2) = p.x * q.z;
   (*this)(0, 3) = 0;

   (*this)(1, 0) = p.y * q.x;
   (*this)(1, 1) = p.y * q.y;
   (*this)(1, 2) = p.y * q.z;
   (*this)(1, 3) = 0;

   (*this)(2, 0) = p.z * q.x;
   (*this)(2, 1) = p.z * q.y;
   (*this)(2, 2) = p.z * q.z;
   (*this)(2, 3) = 0;

   (*this)(3, 0) = 0;
   (*this)(3, 1) = 0;
   (*this)(3, 2) = 0;
   (*this)(3, 3) = 1;

   return (*this);
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::mul(Box3F& box) const
{
   AssertFatal(rows == 4 && cols == 4, "Multiplying Box3F with matrix requires 4x4");

   // Create an array of all 8 corners of the box
   Point3F corners[8] = {
       Point3F(box.minExtents.x, box.minExtents.y, box.minExtents.z),
       Point3F(box.minExtents.x, box.minExtents.y, box.maxExtents.z),
       Point3F(box.minExtents.x, box.maxExtents.y, box.minExtents.z),
       Point3F(box.minExtents.x, box.maxExtents.y, box.maxExtents.z),
       Point3F(box.maxExtents.x, box.minExtents.y, box.minExtents.z),
       Point3F(box.maxExtents.x, box.minExtents.y, box.maxExtents.z),
       Point3F(box.maxExtents.x, box.maxExtents.y, box.minExtents.z),
       Point3F(box.maxExtents.x, box.maxExtents.y, box.maxExtents.z),
   };

   for (U32 i = 0; i < 8; i++) {
      corners[i] = (*this) * corners[i];
   }

   box.minExtents = corners[0];
   box.maxExtents = corners[0];
   for (U32 i = 1; i < 8; ++i) {
      box.minExtents.x = mMin(box.minExtents.x, corners[i].x);
      box.minExtents.y = mMin(box.minExtents.y, corners[i].y);
      box.minExtents.z = mMin(box.minExtents.z, corners[i].z);

      box.maxExtents.x = mMax(box.maxExtents.x, corners[i].x);
      box.maxExtents.y = mMax(box.maxExtents.y, corners[i].y);
      box.maxExtents.z = mMax(box.maxExtents.z, corners[i].z);
   }
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline bool Matrix<DATA_TYPE, rows, cols>::isAffine() const
{
   if ((*this)(rows - 1, cols - 1) != 1.0f) {
      return false;
   }

   for (U32 col = 0; col < cols - 1; ++col) {
      if ((*this)(rows - 1, col) != 0.0f) {
         return false;
      }
   }

   Point3F one, two, three;
   getColumn(0, &one);
   getColumn(1, &two);
   getColumn(2, &three);

   // check columns
   {
      if (mDot(one, two) > 0.0001f ||
         mDot(one, three) > 0.0001f ||
         mDot(two, three) > 0.0001f)
         return false;

      if (mFabs(1.0f - one.lenSquared()) > 0.0001f ||
         mFabs(1.0f - two.lenSquared()) > 0.0001f ||
         mFabs(1.0f - three.lenSquared()) > 0.0001f)
         return false;
   }

   getRow(0, &one);
   getRow(1, &two);
   getRow(2, &three);
   // check rows
   {
      if (mDot(one, two) > 0.0001f ||
         mDot(one, three) > 0.0001f ||
         mDot(two, three) > 0.0001f)
         return false;

      if (mFabs(1.0f - one.lenSquared()) > 0.0001f ||
         mFabs(1.0f - two.lenSquared()) > 0.0001f ||
         mFabs(1.0f - three.lenSquared()) > 0.0001f)
         return false;
   }

   return true;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline Matrix<DATA_TYPE, rows, cols> Matrix<DATA_TYPE, rows, cols>::affineInverse()
{
   AssertFatal(rows >= 4 && cols >= 4, "affineInverse requires at least 4x4");
   Matrix<DATA_TYPE, 3, 3> subMatrix;

   for (U32 i = 0; i < 3; i++) {
      for (U32 j = 0; j < 3; j++) {
         subMatrix(i, j) = (*this)(i, j);
      }
   }

   subMatrix.transpose();

   Point3F pos = getPosition();
   (*this)(0, 3) = mDot(subMatrix.getColumn3F(0), pos);
   (*this)(1, 3) = mDot(subMatrix.getColumn3F(1), pos);
   (*this)(2, 3) = mDot(subMatrix.getColumn3F(2), pos);

   return *this;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline EulerF Matrix<DATA_TYPE, rows, cols>::toEuler() const
{
   AssertFatal(rows >= 3 && cols >= 3, "Euler rotations require at least a 3x3 matrix.");
   // Extract rotation matrix components
   const DATA_TYPE m00 = (*this)(0, 0);
   const DATA_TYPE m01 = (*this)(0, 1);
   const DATA_TYPE m02 = (*this)(0, 2);
   const DATA_TYPE m10 = (*this)(1, 0);
   const DATA_TYPE m11 = (*this)(1, 1);
   const DATA_TYPE m21 = (*this)(2, 1);
   const DATA_TYPE m22 = (*this)(2, 2);

   // like all others assume float for now.
   EulerF r;

   r.x = mAsin(mClampF(m21, -1.0, 1.0));
   if (mCos(r.x) != 0.0f) {
      r.y = mAtan2(-m02, m22); // yaw
      r.z = mAtan2(-m10, m11); // roll
   }
   else {
      r.y = 0.0f;
      r.z = mAtan2(m01, m00); // this rolls when pitch is +90 degrees
   }

   return r;
}

template<typename DATA_TYPE, U32 rows, U32 cols>
inline void Matrix<DATA_TYPE, rows, cols>::dumpMatrix(const char* caption) const
{
   U32 size = (caption == NULL) ? 0 : dStrlen(caption);
   FrameTemp<char> spacer(size + 1);
   char* spacerRef = spacer;

   // is_floating_point should return true for floats and doubles.
   const char* formatSpec = std::is_floating_point_v<DATA_TYPE> ? " %-8.4f" : " %d";

   dMemset(spacerRef, ' ', size);
   // null terminate.
   spacerRef[size] = '\0';

   /*Con::printf("%s = | %-8.4f %-8.4f %-8.4f %-8.4f |", caption, m[idx(0, 0)], m[idx(0, 1)], m[idx(0, 2)], m[idx(0, 3)]);
   Con::printf("%s   | %-8.4f %-8.4f %-8.4f %-8.4f |", spacerRef, m[idx(1, 0)], m[idx(1, 1)], m[idx(1, 2)], m[idx(1, 3)]);
   Con::printf("%s   | %-8.4f %-8.4f %-8.4f %-8.4f |", spacerRef, m[idx(2, 0)], m[idx(2, 1)], m[idx(2, 2)], m[idx(2, 3)]);
   Con::printf("%s   | %-8.4f %-8.4f %-8.4f %-8.4f |", spacerRef, m[idx(3, 0)], m[idx(3, 1)], m[idx(3, 2)], m[idx(3, 3)]);*/

   StringBuilder str;
   str.format("%s = |", caption);
   for (U32 i = 0; i < rows; i++) {
      if (i > 0) {
         str.append(spacerRef);
      }

      for (U32 j = 0; j < cols; j++) {
         str.format(formatSpec, (*this)(i, j));
      }
      str.append(" |\n");
   }

   Con::printf("%s", str.end().c_str());
}

//------------------------------------
// Non-member methods
//------------------------------------

inline void mTransformPlane(
   const MatrixF& mat,
   const Point3F& scale,
   const PlaneF& plane,
   PlaneF* result
) {
   // Create a non-const copy of the matrix
   MatrixF matCopy = mat;

   // Create the inverse scale matrix
   MatrixF invScale = MatrixF::Identity;
   invScale(0, 0) = 1.0f / scale.x;
   invScale(1, 1) = 1.0f / scale.y;
   invScale(2, 2) = 1.0f / scale.z;

   // Compute the inverse transpose of the matrix
   MatrixF invTrMatrix = matCopy.transpose().affineInverse() * invScale;

   // Transform the plane normal
   Point3F norm(plane.x, plane.y, plane.z);
   norm = invTrMatrix * norm;
   float normLength = std::sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
   norm.x /= normLength;
   norm.y /= normLength;
   norm.z /= normLength;

   // Transform the plane point
   Point3F point = norm * (-plane.d);
   MMatrixF temp = mat;
   point.x *= scale.x;
   point.y *= scale.y;
   point.z *= scale.z;
   point = temp * point;

   // Recompute the plane distance
   PlaneF resultPlane(point, norm);
   result->x = resultPlane.x;
   result->y = resultPlane.y;
   result->z = resultPlane.z;
   result->d = resultPlane.d;
}

//--------------------------------------------
// INLINE FUNCTIONS END
//--------------------------------------------

typedef Matrix<F32, 4, 4> MatrixF;

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

#endif // !USE_TEMPLATE_MATRIX

#endif //_MMATRIX_H_
