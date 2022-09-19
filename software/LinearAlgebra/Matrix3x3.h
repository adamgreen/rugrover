/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef MATRIX_3x3_H_
#define MATRIX_3x3_H_

#include <arm_math.h>
#include <string.h>
#include <LinearAlgebra/Vector3.h>

class Matrix3x3
{
public:
    Matrix3x3(float m00, float m01, float m02,
              float m10, float m11, float m12,
              float m20, float m21, float m22)
    {
        m_00 = m00;
        m_01 = m01;
        m_02 = m02;

        m_10 = m10;
        m_11 = m11;
        m_12 = m12;

        m_20 = m20;
        m_21 = m21;
        m_22 = m22;
    }

    Matrix3x3()
    {
        clear();
    }

    void clear()
    {
        memset(m_data, 0, sizeof(m_data));
    }

    Vector3<float> multiply(const Vector3<float>& v) const
    {
        Vector3<float> prod;

        prod.x = m_00*v.x + m_01*v.y + m_02*v.z;
        prod.y = m_10*v.x + m_11*v.y + m_12*v.z;
        prod.z = m_20*v.x + m_21*v.y + m_22*v.z;

        return prod;
    }

    Matrix3x3 multiply(const Matrix3x3& m) const
    {
        Matrix3x3 prod;

        prod.m_00 = m_00*m.m_00 + m_01*m.m_10 + m_02*m.m_20;
        prod.m_01 = m_00*m.m_01 + m_01*m.m_11 + m_02*m.m_21;
        prod.m_02 = m_00*m.m_02 + m_01*m.m_12 + m_02*m.m_22;

        prod.m_10 = m_10*m.m_00 + m_11*m.m_10 + m_12*m.m_20;
        prod.m_11 = m_10*m.m_01 + m_11*m.m_11 + m_12*m.m_21;
        prod.m_12 = m_10*m.m_02 + m_11*m.m_12 + m_12*m.m_22;

        prod.m_20 = m_20*m.m_00 + m_21*m.m_10 + m_22*m.m_20;
        prod.m_21 = m_20*m.m_01 + m_21*m.m_11 + m_22*m.m_21;
        prod.m_22 = m_20*m.m_02 + m_21*m.m_12 + m_22*m.m_22;

        return prod;
    }

    Matrix3x3 multiplyTransposed(const Matrix3x3& m) const
    {
        Matrix3x3 prod;

        prod.m_00 = m_00*m.m_00 + m_01*m.m_01 + m_02*m.m_02;
        prod.m_01 = m_00*m.m_10 + m_01*m.m_11 + m_02*m.m_12;
        prod.m_02 = m_00*m.m_20 + m_01*m.m_21 + m_02*m.m_22;

        prod.m_10 = m_10*m.m_00 + m_11*m.m_01 + m_12*m.m_02;
        prod.m_11 = m_10*m.m_10 + m_11*m.m_11 + m_12*m.m_12;
        prod.m_12 = m_10*m.m_20 + m_11*m.m_21 + m_12*m.m_22;

        prod.m_20 = m_20*m.m_00 + m_21*m.m_01 + m_22*m.m_02;
        prod.m_21 = m_20*m.m_10 + m_21*m.m_11 + m_22*m.m_12;
        prod.m_22 = m_20*m.m_20 + m_21*m.m_21 + m_22*m.m_22;

        return prod;
    }

    Matrix3x3 inverse() const
    {
        Matrix3x3               result;
        arm_matrix_instance_f32 instanceIn = {3, 3, (float*)m_data};
        arm_matrix_instance_f32 instanceOut = {3, 3, (float*)result.m_data};

        arm_mat_inverse_f32(&instanceIn, &instanceOut);

        return result;
    }

    Matrix3x3 add(const Matrix3x3& m) const
    {
        Matrix3x3 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01 + m.m_01;
        sum.m_02 = m_02 + m.m_02;

        sum.m_10 = m_10 + m.m_10;
        sum.m_11 = m_11 + m.m_11;
        sum.m_12 = m_12 + m.m_12;

        sum.m_20 = m_20 + m.m_20;
        sum.m_21 = m_21 + m.m_21;
        sum.m_22 = m_22 + m.m_22;

        return sum;
    }

    // This function is an optimization of the general add() method which can be used when parameter m is known to only
    // contain zeroes in its non-diagonal elements.
    Matrix3x3 addDiagonal(const Matrix3x3& m) const
    {
        Matrix3x3 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01;
        sum.m_02 = m_02;

        sum.m_10 = m_10;
        sum.m_11 = m_11 + m.m_11;
        sum.m_12 = m_12;

        sum.m_20 = m_20;
        sum.m_21 = m_21;
        sum.m_22 = m_22 + m.m_22;

        return sum;
    }

    Matrix3x3 subtract(const Matrix3x3& m) const
    {
        Matrix3x3 sum;

        sum.m_00 = m_00 - m.m_00;
        sum.m_01 = m_01 - m.m_01;
        sum.m_02 = m_02 - m.m_02;

        sum.m_10 = m_10 - m.m_10;
        sum.m_11 = m_11 - m.m_11;
        sum.m_12 = m_12 - m.m_12;

        sum.m_20 = m_20 - m.m_20;
        sum.m_21 = m_21 - m.m_21;
        sum.m_22 = m_22 - m.m_22;

        return sum;
    }

    union
    {
        float m_data[3][3];
        struct
        {
            float m_00;
            float m_01;
            float m_02;

            float m_10;
            float m_11;
            float m_12;

            float m_20;
            float m_21;
            float m_22;
        };
    };
};

#endif /*MATRIX_3x3_H_ */
