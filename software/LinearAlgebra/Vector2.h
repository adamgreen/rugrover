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
#ifndef VECTOR2_H_
#define VECTOR2_H_

#include <assert.h>
#include <stdint.h>


template <class T>
class Vector2
{
public:
    Vector2(T x, T y)
    {
        this->x = x;
        this->y = y;
    }

    Vector2()
    {
        clear();
    }

    static Vector2<T> createFromSwizzledSource(Vector2<int16_t>& swizzle, Vector2<T>& v)
    {
        Vector2<T> newVector;

        for (int i = 0 ; i < 2 ; i++)
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
    }

    T& operator [](int i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        default:
            assert(i >= 0 && i <= 1);
            return y;
        }
    }

    float magnitude() const
    {
        return sqrtf(x*x + y*y);
    }

    void normalize()
    {
        float magInverse = 1.0f / magnitude();
        x *= magInverse;
        y *= magInverse;
    }

    T dotProduct(const Vector2<T>& v) const
    {
        return x * v.x + y * v.y;
    }

    Vector2<T> multiply(T scalar) const
    {
        return Vector2<T>(x * scalar, y * scalar);
    }

    Vector2<T> multiply(const Vector2<T> v) const
    {
        return Vector2<T>(x * v.x, y * v.y);
    }

    Vector2<T> add(const Vector2<T>& v) const
    {
        return Vector2<T>(x + v.x, y + v.y);
    }

    Vector2<T> subtract(const Vector2<T>& v) const
    {
        return Vector2<T>(x - v.x, y - v.y);
    }

    T x;
    T y;
};

#endif /* VECTOR2_H_ */
