#ifndef LQR_H
#define LQR_H

#include <Arduino.h>

class LQR2 {
private:
  float A[2][2];
  float B[2];
  float Q[2][2];
  float R;
  float P[2][2];
  float K[2];
  float xRef[2];
  float uMin;
  float uMax;
  bool solved;

  static float absf(float x) {
    return x < 0.0f ? -x : x;
  }

  static void mat2Mul(const float M1[2][2], const float M2[2][2], float out[2][2]) {
    out[0][0] = M1[0][0] * M2[0][0] + M1[0][1] * M2[1][0];
    out[0][1] = M1[0][0] * M2[0][1] + M1[0][1] * M2[1][1];
    out[1][0] = M1[1][0] * M2[0][0] + M1[1][1] * M2[1][0];
    out[1][1] = M1[1][0] * M2[0][1] + M1[1][1] * M2[1][1];
  }

  static void mat2Add(const float M1[2][2], const float M2[2][2], float out[2][2]) {
    out[0][0] = M1[0][0] + M2[0][0];
    out[0][1] = M1[0][1] + M2[0][1];
    out[1][0] = M1[1][0] + M2[1][0];
    out[1][1] = M1[1][1] + M2[1][1];
  }

  static void mat2Sub(const float M1[2][2], const float M2[2][2], float out[2][2]) {
    out[0][0] = M1[0][0] - M2[0][0];
    out[0][1] = M1[0][1] - M2[0][1];
    out[1][0] = M1[1][0] - M2[1][0];
    out[1][1] = M1[1][1] - M2[1][1];
  }

  static void mat2Transpose(const float M[2][2], float out[2][2]) {
    out[0][0] = M[0][0];
    out[0][1] = M[1][0];
    out[1][0] = M[0][1];
    out[1][1] = M[1][1];
  }

  static void mat2VecMul(const float M[2][2], const float v[2], float out[2]) {
    out[0] = M[0][0] * v[0] + M[0][1] * v[1];
    out[1] = M[1][0] * v[0] + M[1][1] * v[1];
  }

  static float vecTMatVec2(const float v[2], const float M[2][2]) {
    float mv0 = M[0][0] * v[0] + M[0][1] * v[1];
    float mv1 = M[1][0] * v[0] + M[1][1] * v[1];
    return v[0] * mv0 + v[1] * mv1;
  }

  static void vecOuter2(const float v1[2], const float v2[2], float out[2][2]) {
    out[0][0] = v1[0] * v2[0];
    out[0][1] = v1[0] * v2[1];
    out[1][0] = v1[1] * v2[0];
    out[1][1] = v1[1] * v2[1];
  }

public:
  LQR2() {
    A[0][0] = 1.0f; A[0][1] = 0.0f;
    A[1][0] = 0.0f; A[1][1] = 1.0f;
    B[0] = 0.0f; B[1] = 1.0f;

    Q[0][0] = 1.0f; Q[0][1] = 0.0f;
    Q[1][0] = 0.0f; Q[1][1] = 1.0f;
    R = 1.0f;

    P[0][0] = Q[0][0]; P[0][1] = Q[0][1];
    P[1][0] = Q[1][0]; P[1][1] = Q[1][1];

    K[0] = 0.0f;
    K[1] = 0.0f;

    xRef[0] = 0.0f;
    xRef[1] = 0.0f;

    uMin = -255.0f;
    uMax = 255.0f;
    solved = false;
  }

  void setModel(const float A00, const float A01, const float A10, const float A11,
                const float B0, const float B1) {
    A[0][0] = A00; A[0][1] = A01;
    A[1][0] = A10; A[1][1] = A11;
    B[0] = B0;
    B[1] = B1;
    solved = false;
  }

  void setCost(const float Q00, const float Q01, const float Q10, const float Q11, const float Rval) {
    Q[0][0] = Q00; Q[0][1] = Q01;
    Q[1][0] = Q10; Q[1][1] = Q11;
    R = Rval;
    solved = false;
  }

  void setReference(const float x0Ref, const float x1Ref) {
    xRef[0] = x0Ref;
    xRef[1] = x1Ref;
  }

  void setOutputLimits(const float minOut, const float maxOut) {
    uMin = minOut;
    uMax = maxOut;
  }

  bool solve(const int maxIterations = 80, const float tolerance = 1e-4f) {
    float Pk[2][2] = {
      {Q[0][0], Q[0][1]},
      {Q[1][0], Q[1][1]}
    };

    float At[2][2];
    mat2Transpose(A, At);

    for (int i = 0; i < maxIterations; i++) {
      float AP[2][2];
      float AtPA[2][2];
      mat2Mul(A, Pk, AP);
      mat2Mul(At, AP, AtPA);

      float PB[2];
      mat2VecMul(Pk, B, PB);

      float BtPB = B[0] * PB[0] + B[1] * PB[1];
      float denom = R + BtPB;
      if (absf(denom) < 1e-9f) {
        solved = false;
        return false;
      }

      float AtPB[2];
      AtPB[0] = At[0][0] * PB[0] + At[0][1] * PB[1];
      AtPB[1] = At[1][0] * PB[0] + At[1][1] * PB[1];

      float BtPA[2];
      BtPA[0] = B[0] * (Pk[0][0] * A[0][0] + Pk[0][1] * A[1][0]) + B[1] * (Pk[1][0] * A[0][0] + Pk[1][1] * A[1][0]);
      BtPA[1] = B[0] * (Pk[0][0] * A[0][1] + Pk[0][1] * A[1][1]) + B[1] * (Pk[1][0] * A[0][1] + Pk[1][1] * A[1][1]);

      float correction[2][2];
      vecOuter2(AtPB, BtPA, correction);
      correction[0][0] /= denom;
      correction[0][1] /= denom;
      correction[1][0] /= denom;
      correction[1][1] /= denom;

      float nextPTemp[2][2];
      float nextP[2][2];
      mat2Sub(AtPA, correction, nextPTemp);
      mat2Add(nextPTemp, Q, nextP);

      float maxDiff = 0.0f;
      for (int r = 0; r < 2; r++) {
        for (int c = 0; c < 2; c++) {
          float d = absf(nextP[r][c] - Pk[r][c]);
          if (d > maxDiff) maxDiff = d;
          Pk[r][c] = nextP[r][c];
        }
      }

      if (maxDiff < tolerance) {
        break;
      }
    }

    P[0][0] = Pk[0][0]; P[0][1] = Pk[0][1];
    P[1][0] = Pk[1][0]; P[1][1] = Pk[1][1];

    float PB[2];
    mat2VecMul(P, B, PB);
    float denom = R + (B[0] * PB[0] + B[1] * PB[1]);
    if (absf(denom) < 1e-9f) {
      solved = false;
      return false;
    }

    K[0] = (B[0] * (P[0][0] * A[0][0] + P[0][1] * A[1][0]) + B[1] * (P[1][0] * A[0][0] + P[1][1] * A[1][0])) / denom;
    K[1] = (B[0] * (P[0][0] * A[0][1] + P[0][1] * A[1][1]) + B[1] * (P[1][0] * A[0][1] + P[1][1] * A[1][1])) / denom;

    solved = true;
    return true;
  }

  float compute(const float x0, const float x1) {
    if (!solved) {
      if (!solve()) {
        return 0.0f;
      }
    }

    float e0 = x0 - xRef[0];
    float e1 = x1 - xRef[1];

    float u = -(K[0] * e0 + K[1] * e1);
    if (u > uMax) u = uMax;
    if (u < uMin) u = uMin;
    return u;
  }

  float getK0() const { return K[0]; }
  float getK1() const { return K[1]; }

  bool isSolved() const { return solved; }
};

#endif
