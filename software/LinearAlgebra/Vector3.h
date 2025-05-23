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
#ifndef VECTOR3_H_
#define VECTOR3_H_

#include <assert.h>
#include <stdint.h>


template <class T>
class Vector3
{
public:
    Vector3(T x, T y, T z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vector3()
    {
        clear();
    }

    static Vector3<T> createFromSwizzledSource(Vector3<int16_t>& swizzle, Vector3<T>& v)
    {
        Vector3<T> newVector;

        for (int i = 0 ; i < 3 ; i++)
        {
            int16_t swizzleFrom = swizzle[i];
            bool    inverse = false;

            if (swizzleFrom < 0)
            {
                swizzleFrom = -swizzleFrom;
                inverse = true;
            }

            newVector[i] = inverse ? -v[swizzleFrom - 1] : v[swizzleFrom - 1];
        }

        return newVector;
    }

    void clear()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    T& operator [](int i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            assert(i >= 0 && i <= 2);
            return z;
        }
    }

    float magnitude() const
    {
        return sqrtf(x*x + y*y + z*z);
    }

    void normalize()
    {
        float magInverse = 1.0f / magnitude();
        x *= magInverse;
        y *= magInverse;
        z *= magInverse;
    }

    T dotProduct(const Vector3<T>& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }

    Vector3<T> crossProduct(const Vector3<T>& v) const
    {
        return Vector3<T>(y*v.z - z*v.y,
                         z*v.x - x*v.z,
                         x*v.y - y*v.x);
    }

    Vector3<T> multiply(T scalar) const
    {
        return Vector3<T>(x * scalar, y * scalar, z * scalar);
    }

    Vector3<T> multiply(const Vector3<T> v) const
    {
        return Vector3<T>(x * v.x, y * v.y, z * v.z);
    }

    Vector3<T> add(const Vector3<T>& v) const
    {
        return Vector3<T>(x + v.x, y + v.y, z + v.z);
    }

    Vector3<T> subtract(const Vector3<T>& v) const
    {
        return Vector3<T>(x - v.x, y - v.y, z - v.z);
    }

    T x;
    T y;
    T z;
};

#endif /* VECTOR3_H_ */
