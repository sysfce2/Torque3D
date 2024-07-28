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

#include "core/strings/stringFunctions.h"
#include "core/frameAllocator.h"

#include "math/mMatrix.h"
#include "console/console.h"

#include "console/enginePrimitives.h"
#include "console/engineTypes.h"

const MatrixF MatrixF::Identity( true );

// idx(i,j) is index to element in column i, row j

void MatrixF::transposeTo(F32 *matrix) const
{
   matrix[idx(0,0)] = m[idx(0,0)];
   matrix[idx(0,1)] = m[idx(1,0)];
   matrix[idx(0,2)] = m[idx(2,0)];
   matrix[idx(0,3)] = m[idx(3,0)];
   matrix[idx(1,0)] = m[idx(0,1)];
   matrix[idx(1,1)] = m[idx(1,1)];
   matrix[idx(1,2)] = m[idx(2,1)];
   matrix[idx(1,3)] = m[idx(3,1)];
   matrix[idx(2,0)] = m[idx(0,2)];
   matrix[idx(2,1)] = m[idx(1,2)];
   matrix[idx(2,2)] = m[idx(2,2)];
   matrix[idx(2,3)] = m[idx(3,2)];
   matrix[idx(3,0)] = m[idx(0,3)];
   matrix[idx(3,1)] = m[idx(1,3)];
   matrix[idx(3,2)] = m[idx(2,3)];
   matrix[idx(3,3)] = m[idx(3,3)];
}

bool MatrixF::isAffine() const
{
   // An affine transform is defined by the following structure
   //
   // [ X X X P ]
   // [ X X X P ]
   // [ X X X P ]
   // [ 0 0 0 1 ]
   //
   //  Where X is an orthonormal 3x3 submatrix and P is an arbitrary translation
   //  We'll check in the following order:
   //   1: [3][3] must be 1
   //   2: Shear portion must be zero
   //   3: Dot products of rows and columns must be zero
   //   4: Length of rows and columns must be 1
   //
   if (m[idx(3,3)] != 1.0f)
      return false;

   if (m[idx(0,3)] != 0.0f ||
       m[idx(1,3)] != 0.0f ||
       m[idx(2,3)] != 0.0f)
      return false;

   Point3F one, two, three;
   getColumn(0, &one);
   getColumn(1, &two);
   getColumn(2, &three);
   if (mDot(one, two)   > 0.0001f ||
       mDot(one, three) > 0.0001f ||
       mDot(two, three) > 0.0001f)
      return false;

   if (mFabs(1.0f - one.lenSquared()) > 0.0001f ||
       mFabs(1.0f - two.lenSquared()) > 0.0001f ||
       mFabs(1.0f - three.lenSquared()) > 0.0001f)
      return false;

   getRow(0, &one);
   getRow(1, &two);
   getRow(2, &three);
   if (mDot(one, two)   > 0.0001f ||
       mDot(one, three) > 0.0001f ||
       mDot(two, three) > 0.0001f)
      return false;

   if (mFabs(1.0f - one.lenSquared()) > 0.0001f ||
       mFabs(1.0f - two.lenSquared()) > 0.0001f ||
       mFabs(1.0f - three.lenSquared()) > 0.0001f)
      return false;

   // We're ok.
   return true;
}

// Perform inverse on full 4x4 matrix.  Used in special cases only, so not at all optimized.
bool MatrixF::fullInverse()
{
   Point4F a,b,c,d;
   getRow(0,&a);
   getRow(1,&b);
   getRow(2,&c);
   getRow(3,&d);

   // det = a0*b1*c2*d3 - a0*b1*c3*d2 - a0*c1*b2*d3 + a0*c1*b3*d2 + a0*d1*b2*c3 - a0*d1*b3*c2 -
   //       b0*a1*c2*d3 + b0*a1*c3*d2 + b0*c1*a2*d3 - b0*c1*a3*d2 - b0*d1*a2*c3 + b0*d1*a3*c2 +
   //       c0*a1*b2*d3 - c0*a1*b3*d2 - c0*b1*a2*d3 + c0*b1*a3*d2 + c0*d1*a2*b3 - c0*d1*a3*b2 -
   //       d0*a1*b2*c3 + d0*a1*b3*c2 + d0*b1*a2*c3 - d0*b1*a3*c2 - d0*c1*a2*b3 + d0*c1*a3*b2
   F32 det = a.x*b.y*c.z*d.w - a.x*b.y*c.w*d.z - a.x*c.y*b.z*d.w + a.x*c.y*b.w*d.z + a.x*d.y*b.z*c.w - a.x*d.y*b.w*c.z
           - b.x*a.y*c.z*d.w + b.x*a.y*c.w*d.z + b.x*c.y*a.z*d.w - b.x*c.y*a.w*d.z - b.x*d.y*a.z*c.w + b.x*d.y*a.w*c.z
           + c.x*a.y*b.z*d.w - c.x*a.y*b.w*d.z - c.x*b.y*a.z*d.w + c.x*b.y*a.w*d.z + c.x*d.y*a.z*b.w - c.x*d.y*a.w*b.z
           - d.x*a.y*b.z*c.w + d.x*a.y*b.w*c.z + d.x*b.y*a.z*c.w - d.x*b.y*a.w*c.z - d.x*c.y*a.z*b.w + d.x*c.y*a.w*b.z;

   if (mFabs(det)<0.00001f)
      return false;

   Point4F aa,bb,cc,dd;
   aa.x =  b.y*c.z*d.w - b.y*c.w*d.z - c.y*b.z*d.w + c.y*b.w*d.z + d.y*b.z*c.w - d.y*b.w*c.z;
   aa.y = -a.y*c.z*d.w + a.y*c.w*d.z + c.y*a.z*d.w - c.y*a.w*d.z - d.y*a.z*c.w + d.y*a.w*c.z;
   aa.z =  a.y*b.z*d.w - a.y*b.w*d.z - b.y*a.z*d.w + b.y*a.w*d.z + d.y*a.z*b.w - d.y*a.w*b.z;
   aa.w = -a.y*b.z*c.w + a.y*b.w*c.z + b.y*a.z*c.w - b.y*a.w*c.z - c.y*a.z*b.w + c.y*a.w*b.z;

   bb.x = -b.x*c.z*d.w + b.x*c.w*d.z + c.x*b.z*d.w - c.x*b.w*d.z - d.x*b.z*c.w + d.x*b.w*c.z;
   bb.y =  a.x*c.z*d.w - a.x*c.w*d.z - c.x*a.z*d.w + c.x*a.w*d.z + d.x*a.z*c.w - d.x*a.w*c.z;
   bb.z = -a.x*b.z*d.w + a.x*b.w*d.z + b.x*a.z*d.w - b.x*a.w*d.z - d.x*a.z*b.w + d.x*a.w*b.z;
   bb.w =  a.x*b.z*c.w - a.x*b.w*c.z - b.x*a.z*c.w + b.x*a.w*c.z + c.x*a.z*b.w - c.x*a.w*b.z;

   cc.x =  b.x*c.y*d.w - b.x*c.w*d.y - c.x*b.y*d.w + c.x*b.w*d.y + d.x*b.y*c.w - d.x*b.w*c.y;
   cc.y = -a.x*c.y*d.w + a.x*c.w*d.y + c.x*a.y*d.w - c.x*a.w*d.y - d.x*a.y*c.w + d.x*a.w*c.y;
   cc.z =  a.x*b.y*d.w - a.x*b.w*d.y - b.x*a.y*d.w + b.x*a.w*d.y + d.x*a.y*b.w - d.x*a.w*b.y;
   cc.w = -a.x*b.y*c.w + a.x*b.w*c.y + b.x*a.y*c.w - b.x*a.w*c.y - c.x*a.y*b.w + c.x*a.w*b.y;

   dd.x = -b.x*c.y*d.z + b.x*c.z*d.y + c.x*b.y*d.z - c.x*b.z*d.y - d.x*b.y*c.z + d.x*b.z*c.y;
   dd.y =  a.x*c.y*d.z - a.x*c.z*d.y - c.x*a.y*d.z + c.x*a.z*d.y + d.x*a.y*c.z - d.x*a.z*c.y;
   dd.z = -a.x*b.y*d.z + a.x*b.z*d.y + b.x*a.y*d.z - b.x*a.z*d.y - d.x*a.y*b.z + d.x*a.z*b.y;
   dd.w =  a.x*b.y*c.z - a.x*b.z*c.y - b.x*a.y*c.z + b.x*a.z*c.y + c.x*a.y*b.z - c.x*a.z*b.y;

   setRow(0,aa);
   setRow(1,bb);
   setRow(2,cc);
   setRow(3,dd);

   mul(1.0f/det);

   return true;
}

void MatrixF::reverseProjection()
{
   m[idx(0, 2)] = m[idx(0, 3)] - m[idx(0, 2)];
   m[idx(1, 2)] = m[idx(1, 3)] - m[idx(1, 2)];
   m[idx(2, 2)] = m[idx(2, 3)] - m[idx(2, 2)];
   m[idx(3, 2)] = m[idx(3, 3)] - m[idx(3, 2)];
}

EulerF MatrixF::toEuler() const
{
   const F32 * mat = m;

   EulerF r;
   r.x = mAsin(mClampF(mat[MatrixF::idx(2,1)], -1.0, 1.0));

   if(mCos(r.x) != 0.f)
   {
      r.y = mAtan2(-mat[MatrixF::idx(2,0)], mat[MatrixF::idx(2,2)]);
      r.z = mAtan2(-mat[MatrixF::idx(0,1)], mat[MatrixF::idx(1,1)]);
   }
   else
   {
      r.y = 0.f;
      r.z = mAtan2(mat[MatrixF::idx(1,0)], mat[MatrixF::idx(0,0)]);
   }

   return r;
}

void MatrixF::dumpMatrix(const char *caption /* =NULL */) const
{
   U32 size = (caption == NULL)? 0 : dStrlen(caption);
   FrameTemp<char> spacer(size+1);
   char *spacerRef = spacer;

   dMemset(spacerRef, ' ', size);
   spacerRef[size] = 0;

   Con::printf("%s = | %-8.4f %-8.4f %-8.4f %-8.4f |", caption,    m[idx(0,0)], m[idx(0, 1)], m[idx(0, 2)], m[idx(0, 3)]);
   Con::printf("%s   | %-8.4f %-8.4f %-8.4f %-8.4f |", spacerRef,  m[idx(1,0)], m[idx(1, 1)], m[idx(1, 2)], m[idx(1, 3)]);
   Con::printf("%s   | %-8.4f %-8.4f %-8.4f %-8.4f |", spacerRef,  m[idx(2,0)], m[idx(2, 1)], m[idx(2, 2)], m[idx(2, 3)]);
   Con::printf("%s   | %-8.4f %-8.4f %-8.4f %-8.4f |", spacerRef,  m[idx(3,0)], m[idx(3, 1)], m[idx(3, 2)], m[idx(3, 3)]);
}

EngineFieldTable::Field MatrixFEngineExport::getMatrixField()
{
   typedef MatrixF ThisType;
   return _FIELD_AS(F32, m, m, 16, "");
}

//------------------------------------
// Templatized matrix class to replace MATRIXF above
//------------------------------------

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
Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::inverse()
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

      // Early out if pivot is 0, return a new empty matrix.
      if (augmentedMatrix(i, i) == static_cast<DATA_TYPE>(0)) {
         return Matrix<DATA_TYPE, rows, cols>();
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
void Matrix<DATA_TYPE, rows, cols>::invert()
{
   (*this) = inverse();
}

template<typename DATA_TYPE, U32 rows, U32 cols>
Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::setCrossProduct(const Point3F& p)
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
Matrix<DATA_TYPE, rows, cols>& Matrix<DATA_TYPE, rows, cols>::setTensorProduct(const Point3F& p, const Point3F& q)
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
void Matrix<DATA_TYPE, rows, cols>::mul(Box3F& box) const
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
bool Matrix<DATA_TYPE, rows, cols>::isAffine() const
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
EulerF Matrix<DATA_TYPE, rows, cols>::toEuler() const
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
void Matrix<DATA_TYPE, rows, cols>::dumpMatrix(const char* caption) const
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
