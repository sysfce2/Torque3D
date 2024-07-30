#include "testing/unitTesting.h"

#include "platform/platform.h"
#include "console/simBase.h"
#include "console/consoleTypes.h"
#include "console/scriptObjects.h"
#include "console/simBase.h"
#include "console/engineAPI.h"
#include "math/mMath.h"
#include "math/util/frustum.h"

TEST(MatrixTest, TestIdentityInit)
{
   MatrixF test(true);

   EXPECT_NEAR(test(0, 0), 1.0f, 0.001f);
   EXPECT_NEAR(test(1, 1), 1.0f, 0.001f);
   EXPECT_NEAR(test(2, 2), 1.0f, 0.001f);
   EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestIdentitySet)
{
   MatrixF test(false);
   test.identity();

   EXPECT_NEAR(test(0, 0), 1.0f, 0.001f);
   EXPECT_NEAR(test(1, 1), 1.0f, 0.001f);
   EXPECT_NEAR(test(2, 2), 1.0f, 0.001f);
   EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestIsIdentity)
{
   MatrixF test(true);
   EXPECT_TRUE(test.isIdentity());
}

TEST(MatrixTest, TestEulerInit)
{
   MatrixF test(EulerF(1.0f, 0.0f, 1.0f));

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);   EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);   EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f); EXPECT_NEAR(test(2, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);      EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}


TEST(MatrixTest, TestEulerSet)
{
   MatrixF test(true);

   test.set(EulerF(1.0f, 0.0f, 1.0f));

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);   EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);   EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f); EXPECT_NEAR(test(2, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);      EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestToEuler)
{
   MatrixF test(true);

   test.set(EulerF(1.0f, 0.0f, 1.0f));

   EulerF euler = test.toEuler();

   EXPECT_NEAR(euler.x, 1.0f, 0.001f); EXPECT_NEAR(euler.y, 0.0f, 0.001f); EXPECT_NEAR(euler.z, 1.0f, 0.001f);
}

TEST(MatrixTest, TestEulerPointInit)
{
   MatrixF test(EulerF(1.0f, 0.0f, 1.0f), Point3F(5.0f, 0.0f, 1.0f));

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);   EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);   EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f); EXPECT_NEAR(test(2, 3), 1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);      EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}


TEST(MatrixTest, TestEulerPointSet)
{
   MatrixF test(true);

   test.set(EulerF(1.0f, 0.0f, 1.0f), Point3F(5.0f, 0.0f, 1.0f));

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);   EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);   EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f); EXPECT_NEAR(test(2, 3), 1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);      EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestSetCrossProduct)
{
   MatrixF test(true);

   test.setCrossProduct(Point3F(5.0f, 0.0f, 1.0f));

   EXPECT_NEAR(test(0, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(0, 1), -1.0f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 1.0f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.0, 0.001f);    EXPECT_NEAR(test(1, 2), -5.0f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(2, 1), 5.0f, 0.001f);   EXPECT_NEAR(test(2, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(2, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);   EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestSetTensorProduct)
{
   MatrixF test(true);

   test.setTensorProduct(Point3F(5.0f, 2.0f, 1.0f), Point3F(5.0f, 2.0f, 1.0f));

   EXPECT_NEAR(test(0, 0), 25.0f, 0.001f); EXPECT_NEAR(test(0, 1), 10.0f, 0.001f);  EXPECT_NEAR(test(0, 2), 5.0f, 0.001f);  EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 10.0f, 0.001f); EXPECT_NEAR(test(1, 1), 4.0, 0.001f);    EXPECT_NEAR(test(1, 2), 2.0f, 0.001f);  EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 5.0f, 0.001f);  EXPECT_NEAR(test(2, 1), 2.0f, 0.001f);   EXPECT_NEAR(test(2, 2), 1.0f, 0.001f);  EXPECT_NEAR(test(2, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);   EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestMulFunction)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mul(test2);

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(0, 3), 5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 3), 2.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), 1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestMulOperator)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test = test * test2;

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(0, 3), 5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 3), 2.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), 1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestMulLFunction)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(0, 3), 4.3845f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 3), -0.8479f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.7081f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), 3.1714, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestMulArgMatrixFunction)
{
   MatrixF testResult(true);

   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   testResult.mul(test2, test);

   EXPECT_NEAR(testResult(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(testResult(0, 1), 0.8415f, 0.001f);  EXPECT_NEAR(testResult(0, 2), 0.0f, 0.001f);     EXPECT_NEAR(testResult(0, 3), 4.3845f, 0.001f);
   EXPECT_NEAR(testResult(1, 0), -0.4546f, 0.001f); EXPECT_NEAR(testResult(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(testResult(1, 2), 0.8415f, 0.001f);  EXPECT_NEAR(testResult(1, 3), -0.8479f, 0.001f);
   EXPECT_NEAR(testResult(2, 0), 0.7081f, 0.001f);  EXPECT_NEAR(testResult(2, 1), -0.4546f, 0.001f); EXPECT_NEAR(testResult(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(testResult(2, 3), 3.1714, 0.001f);
   EXPECT_NEAR(testResult(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(testResult(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(testResult(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(testResult(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestMulScalarFunction)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   test.mul(2.0f);

   EXPECT_NEAR(test(0, 0), 1.0806f, 0.001f);  EXPECT_NEAR(test(0, 1), 1.6829f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 8.7689f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.9093f, 0.001f); EXPECT_NEAR(test(1, 1), 0.5839f, 0.001f);  EXPECT_NEAR(test(1, 2), 1.6829f, 0.001f); EXPECT_NEAR(test(1, 3), -1.6958f, 0.001f);
   EXPECT_NEAR(test(2, 0), 1.4161f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.9093f, 0.001f); EXPECT_NEAR(test(2, 2), 1.0806f, 0.001f); EXPECT_NEAR(test(2, 3), 6.3427f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 2.0f, 0.001f);
}

TEST(MatrixTest, TestMulMatScalarFunction)
{
   MatrixF testTran(true);
   testTran.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   testTran.mulL(test2);

   MatrixF test(true);
   test.mul(testTran, 2.0f);

   EXPECT_NEAR(test(0, 0), 1.0806f, 0.001f);  EXPECT_NEAR(test(0, 1), 1.6829f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 8.7689f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.9093f, 0.001f); EXPECT_NEAR(test(1, 1), 0.5839f, 0.001f);  EXPECT_NEAR(test(1, 2), 1.6829f, 0.001f); EXPECT_NEAR(test(1, 3), -1.6958f, 0.001f);
   EXPECT_NEAR(test(2, 0), 1.4161f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.9093f, 0.001f); EXPECT_NEAR(test(2, 2), 1.0806f, 0.001f); EXPECT_NEAR(test(2, 3), 6.3427f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 2.0f, 0.001f);
}

TEST(MatrixTest, TestMulPoint4)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   Point4F testPoint(0.5f, 1.0f, 2.0f, 1.0f);
   test.mul(testPoint);

   EXPECT_NEAR(testPoint.x, 5.496f, 0.001f);  EXPECT_NEAR(testPoint.y, 0.899f, 0.001f);  EXPECT_NEAR(testPoint.z, 4.151f, 0.001f); EXPECT_NEAR(testPoint.w, 1.0f, 0.001f);
}

TEST(MatrixTest, TestMulPoint3)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   Point3F testPoint(0.5f, 1.0f, 2.0f);
   test.mulP(testPoint);

   EXPECT_NEAR(testPoint.x, 5.496f, 0.001f);  EXPECT_NEAR(testPoint.y, 0.899f, 0.001f);  EXPECT_NEAR(testPoint.z, 4.151f, 0.001f);
}

TEST(MatrixTest, TestMulPoint3ToPoint3)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   Point3F point(0.5f, 1.0f, 2.0f);
   Point3F testPoint;
   test.mulP(point, &testPoint);

   EXPECT_NEAR(testPoint.x, 5.496f, 0.001f);  EXPECT_NEAR(testPoint.y, 0.899f, 0.001f);  EXPECT_NEAR(testPoint.z, 4.151f, 0.001f);
}

TEST(MatrixTest, TestMulVector)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   VectorF testPoint(0.5f, 1.0f, 2.0f);
   test.mulV(testPoint);

   EXPECT_NEAR(testPoint.x, 1.111f, 0.001f);  EXPECT_NEAR(testPoint.y, 1.747f, 0.001f);  EXPECT_NEAR(testPoint.z, 0.979f, 0.001f);
}

TEST(MatrixTest, TestMulVectorToPoint3)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   VectorF vec(0.5f, 1.0f, 2.0f);
   Point3F testPoint;
   test.mulV(vec, &testPoint);

   EXPECT_NEAR(testPoint.x, 1.111f, 0.001f);  EXPECT_NEAR(testPoint.y, 1.747f, 0.001f);  EXPECT_NEAR(testPoint.z, 0.979f, 0.001f);
}

TEST(MatrixTest, TestMulBox)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   Box3F testBox(1.0f);

   test.mul(testBox);

   EXPECT_NEAR(testBox.minExtents.x, 4.5f, 0.001f); EXPECT_NEAR(testBox.minExtents.y, 1.5f, 0.001f); EXPECT_NEAR(testBox.minExtents.z, 0.5f, 0.001f);
   EXPECT_NEAR(testBox.maxExtents.x, 5.5f, 0.001f); EXPECT_NEAR(testBox.maxExtents.y, 2.5f, 0.001f); EXPECT_NEAR(testBox.maxExtents.z, 1.5f, 0.001f);
}

TEST(MatrixTest, TestReverseProjection)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));

   Frustum testFrustum(false, -1.0f, 1.0f, 1.0f, -1.0f, 0.1f, 100.0f, test);

   testFrustum.getProjectionMatrix(&test);


   // test before and after reverse.
   EXPECT_NEAR(test(0, 0), 0.1f, 0.001f);  EXPECT_NEAR(test(0, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(1, 2), 0.1, 0.001f);   EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.001f, 0.001f);  EXPECT_NEAR(test(2, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(2, 3), 0.1001f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 1), 1.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 3), 0.0f, 0.001f);

   test.reverseProjection();

   EXPECT_NEAR(test(0, 0), 0.1f, 0.001f);  EXPECT_NEAR(test(0, 1), 0.0f, 0.001f);   EXPECT_NEAR(test(0, 2), 0.0f, 0.001f); EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.0f, 0.001f);   EXPECT_NEAR(test(1, 2), 0.1, 0.001f);  EXPECT_NEAR(test(1, 3), 0.0, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(2, 1), 1.001f, 0.001f); EXPECT_NEAR(test(2, 2), 0.0f, 0.001f); EXPECT_NEAR(test(2, 3), -0.1001f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);  EXPECT_NEAR(test(3, 1), 1.0f, 0.001f);   EXPECT_NEAR(test(3, 2), 0.0f, 0.001f); EXPECT_NEAR(test(3, 3), 0.0f, 0.001f);
}

TEST(MatrixTest, TestInverse)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   test.inverse();

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(0, 2), 0.7081f, 0.001f);  EXPECT_NEAR(test(0, 3), -5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 3), -2.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0, 0.001f);      EXPECT_NEAR(test(2, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), -1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestInvertTo)
{
   MatrixF test1(true);
   test1.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test1.mulL(test2);

   MatrixF test(true);

   test1.invertTo(&test);

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(0, 2), 0.7081f, 0.001f);  EXPECT_NEAR(test(0, 3), -5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 3), -2.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0, 0.001f);      EXPECT_NEAR(test(2, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), -1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestIsAffine)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   EXPECT_TRUE(test.isAffine());
}

TEST(MatrixTest, TestScale)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   test.scale(2.0f);

   EXPECT_NEAR(test(0, 0), 1.0806f, 0.001f);  EXPECT_NEAR(test(0, 1), 1.6829f, 0.001f);  EXPECT_NEAR(test(0, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(0, 3), 4.3845f, 0.001f);
   EXPECT_NEAR(test(1, 0), -0.9093f, 0.001f); EXPECT_NEAR(test(1, 1), 0.5839f, 0.001f);  EXPECT_NEAR(test(1, 2), 1.6829f, 0.001f); EXPECT_NEAR(test(1, 3), -0.8479f, 0.001f);
   EXPECT_NEAR(test(2, 0), 1.4161f, 0.001f);  EXPECT_NEAR(test(2, 1), -0.9093f, 0.001f); EXPECT_NEAR(test(2, 2), 1.0806f, 0.001f); EXPECT_NEAR(test(2, 3), 3.1714f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);    EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestGetScale)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   test.scale(2.0f);

   Point3F scale;
   scale = test.getScale();

   EXPECT_NEAR(scale.x, 2.0f, 0.001f);  EXPECT_NEAR(scale.y, 2.0f, 0.001f);  EXPECT_NEAR(scale.z, 2.0f, 0.001f);
}

TEST(MatrixTest, TestAffineInverse)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   test.affineInverse();

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(0, 2), 0.7081f, 0.001f);  EXPECT_NEAR(test(0, 3), -5.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 3), -2.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0, 0.001f);      EXPECT_NEAR(test(2, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), -1.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 1), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 2), 0.0f, 0.001f);     EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestTranspose)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   test.transpose();

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(0, 2), 0.7081f, 0.001f);  EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0, 0.001f);      EXPECT_NEAR(test(2, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 4.3845f, 0.001f);  EXPECT_NEAR(test(3, 1), -0.8479f, 0.001f); EXPECT_NEAR(test(3, 2), 3.1714, 0.001f);   EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestTransposeTo)
{
   MatrixF test1(true);
   test1.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test1.mulL(test2);

   MatrixF test(true);

   test1.transposeTo(test);

   EXPECT_NEAR(test(0, 0), 0.5403f, 0.001f);  EXPECT_NEAR(test(0, 1), -0.4546f, 0.001f); EXPECT_NEAR(test(0, 2), 0.7081f, 0.001f);  EXPECT_NEAR(test(0, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(1, 0), 0.8415f, 0.001f);  EXPECT_NEAR(test(1, 1), 0.2919f, 0.001f);  EXPECT_NEAR(test(1, 2), -0.4546f, 0.001f); EXPECT_NEAR(test(1, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(2, 0), 0.0, 0.001f);      EXPECT_NEAR(test(2, 1), 0.8415f, 0.001f);  EXPECT_NEAR(test(2, 2), 0.5403f, 0.001f);  EXPECT_NEAR(test(2, 3), 0.0f, 0.001f);
   EXPECT_NEAR(test(3, 0), 4.3845f, 0.001f);  EXPECT_NEAR(test(3, 1), -0.8479f, 0.001f); EXPECT_NEAR(test(3, 2), 3.1714, 0.001f);   EXPECT_NEAR(test(3, 3), 1.0f, 0.001f);
}

TEST(MatrixTest, TestTransformPlane)
{
   MatrixF test(true);
   test.setPosition(Point3F(5.0f, 2.0f, 1.0f));
   MatrixF test2(EulerF(1.0f, 0.0f, 1.0f));

   test.mulL(test2);

   PlaneF plane(Point3F(0.0f, 0.0f, 0.0f), Point3F(0.0f, 0.0f, 1.0f));

   PlaneF res;
   mTransformPlane(test, Point3F(1.0f, 1.0f, 1.0f), plane, &res);

   EXPECT_NEAR(res.x, 0.0f, 0.001f);
   EXPECT_NEAR(res.y, 0.8414f, 0.001f);
   EXPECT_NEAR(res.z, 0.5403f, 0.001f);
   EXPECT_NEAR(res.d, -1.0f, 0.001f);

}
