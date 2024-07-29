#include "testing/unitTesting.h"

#include "platform/platform.h"
#include "console/simBase.h"
#include "console/consoleTypes.h"
#include "console/scriptObjects.h"
#include "console/simBase.h"
#include "console/engineAPI.h"
#include "math/mMath.h"

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
