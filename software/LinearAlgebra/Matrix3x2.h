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
#ifndef MATRIX_3x2_H_
#define MATRIX_3x2_H_

#include <nrf52.h>
#include <arm_math.h>
#include <string.h>
#include "Vector2.h"
#include "Vector3.h"

class Matrix2x2;
class Matrix3x3;
class Matrix2x3;

class Matrix3x2
{
public:
    Matrix3x2(float m00, float m01,
              float m10, float m11,
              float m20, float m21)
    {
        m_00 = m00;
        m_01 = m01;

        m_10 = m10;
        m_11 = m11;

        m_20 = m20;
        m_21 = m21;
    }

    Matrix3x2()
    {
        clear();
    }

    void clear()
    {
        memset(m_data, 0, sizeof(m_data));
    }

    Matrix3x2 multiply(const Matrix2x2& m) const;
    Matrix3x3 multiply(const Matrix2x3& m) const;

    Vector3<float> multiply(const Vector2<float>& v) const
    {
        Vector3<float> prod;

        prod.x = m_00*v.x + m_01*v.y;
        prod.y = m_10*v.x + m_11*v.y;
        prod.z = m_20*v.x + m_21*v.y;

        return prod;
    }

    Matrix3x2 add(const Matrix3x2& m) const
    {
        Matrix3x2 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01 + m.m_01;

        sum.m_10 = m_10 + m.m_10;
        sum.m_11 = m_11 + m.m_11;

        sum.m_20 = m_20 + m.m_20;
        sum.m_21 = m_21 + m.m_21;

        return sum;
    }

    Matrix3x2 subtract(const Matrix3x2& m) const
    {
        Matrix3x2 sum;

        sum.m_00 = m_00 - m.m_00;
        sum.m_01 = m_01 - m.m_01;

        sum.m_10 = m_10 - m.m_10;
        sum.m_11 = m_11 - m.m_11;

        sum.m_20 = m_20 - m.m_20;
        sum.m_21 = m_21 - m.m_21;

        return sum;
    }

    union
    {
        float m_data[3][2];
        struct
        {
            float m_00;
            float m_01;

            float m_10;
            float m_11;

            float m_20;
            float m_21;
        };
    };
};

#endif /* MATRIX_3x2_H_ */
