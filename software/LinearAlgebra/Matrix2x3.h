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
#ifndef MATRIX_2x3_H_
#define MATRIX_2x3_H_

#include <nrf52.h>
#include <arm_math.h>
#include <string.h>
#include "Vector2.h"
#include "Vector3.h"

class Matrix2x2;
class Matrix3x3;

class Matrix2x3
{
public:
    Matrix2x3(float m00, float m01, float m02,
              float m10, float m11, float m12)
    {
        m_00 = m00;
        m_01 = m01;
        m_02 = m02;

        m_10 = m10;
        m_11 = m11;
        m_12 = m12;
    }

    Matrix2x3()
    {
        clear();
    }

    void clear()
    {
        memset(m_data, 0, sizeof(m_data));
    }

    Matrix2x3 multiply(const Matrix3x3& m) const;

    Vector2<float> multiply(const Vector3<float>& v) const
    {
        Vector2<float> prod;

        prod.x = m_00*v.x + m_01*v.y + m_02*v.z;
        prod.y = m_10*v.x + m_11*v.y + m_12*v.z;

        return prod;
    }

    Matrix2x2 multiplyTransposed(const Matrix2x3& m) const;

    Matrix2x3 add(const Matrix2x3& m) const
    {
        Matrix2x3 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01 + m.m_01;
        sum.m_02 = m_02 + m.m_02;

        sum.m_10 = m_10 + m.m_10;
        sum.m_11 = m_11 + m.m_11;
        sum.m_12 = m_12 + m.m_12;

        return sum;
    }

    Matrix2x3 subtract(const Matrix2x3& m) const
    {
        Matrix2x3 sum;

        sum.m_00 = m_00 - m.m_00;
        sum.m_01 = m_01 - m.m_01;
        sum.m_02 = m_02 - m.m_02;

        sum.m_10 = m_10 - m.m_10;
        sum.m_11 = m_11 - m.m_11;
        sum.m_12 = m_12 - m.m_12;

        return sum;
    }

    union
    {
        float m_data[2][3];
        struct
        {
            float m_00;
            float m_01;
            float m_02;

            float m_10;
            float m_11;
            float m_12;
        };
    };
};

#endif /*MATRIX_3x3_H_ */
