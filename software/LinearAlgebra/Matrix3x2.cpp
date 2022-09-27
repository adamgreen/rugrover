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
#include "Matrix3x2.h"
#include "Matrix2x2.h"
#include "Matrix2x3.h"
#include "Matrix3x3.h"


Matrix3x2 Matrix3x2::multiply(const Matrix2x2& m) const
{
    Matrix3x2 prod;

    prod.m_00 = m_00*m.m_00 + m_01*m.m_10;
    prod.m_01 = m_00*m.m_01 + m_01*m.m_11;

    prod.m_10 = m_10*m.m_00 + m_11*m.m_10;
    prod.m_11 = m_10*m.m_01 + m_11*m.m_11;

    prod.m_20 = m_20*m.m_00 + m_21*m.m_10;
    prod.m_21 = m_20*m.m_01 + m_21*m.m_11;

    return prod;
}

Matrix3x3 Matrix3x2::multiply(const Matrix2x3& m) const
{
    Matrix3x3 prod;

    prod.m_00 = m_00*m.m_00 + m_01*m.m_10;
    prod.m_01 = m_00*m.m_01 + m_01*m.m_11;
    prod.m_02 = m_00*m.m_02 + m_01*m.m_12;

    prod.m_10 = m_10*m.m_00 + m_11*m.m_10;
    prod.m_11 = m_10*m.m_01 + m_11*m.m_11;
    prod.m_12 = m_10*m.m_02 + m_11*m.m_12;

    prod.m_20 = m_20*m.m_00 + m_21*m.m_10;
    prod.m_21 = m_20*m.m_01 + m_21*m.m_11;
    prod.m_22 = m_20*m.m_02 + m_21*m.m_12;

    return prod;
}
