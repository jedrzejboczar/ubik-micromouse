#pragma once

class Vec2 {
    float lin;
    float ang;

public:

    // Vec2 x Vec2
    Vec2& operator +=(const Vec2& rhs);
    friend Vec2 operator +(Vec2 lhs, const Vec2& rhs);
    Vec2& operator -=(const Vec2& rhs);
    friend Vec2 operator -(Vec2 lhs, const Vec2& rhs);
    Vec2& operator *=(const Vec2& rhs);
    friend Vec2 operator *(Vec2 lhs, const Vec2& rhs);
    Vec2& operator /=(const Vec2& rhs);
    friend Vec2 operator /(Vec2 lhs, const Vec2& rhs);
    // Vec2 x float
    Vec2& operator +=(float rhs);
    friend Vec2 operator +(Vec2 lhs, float rhs);
    friend Vec2 operator +(float lhs, Vec2 rhs);
    Vec2& operator -=(float rhs);
    friend Vec2 operator -(Vec2 lhs, float rhs);
    friend Vec2 operator -(float lhs, Vec2 rhs);
    Vec2& operator *=(float rhs);
    friend Vec2 operator *(Vec2 lhs, float rhs);
    friend Vec2 operator *(float lhs, Vec2 rhs);
    Vec2& operator /=(float rhs);
    friend Vec2 operator /(Vec2 lhs, float rhs);
    friend Vec2 operator /(float lhs, Vec2 rhs);

    Vec2& operator -() { x = -x; y = -y; return *this; }

    //ASSINGMENT AND EQUALITY OPERATIONS
    inline bool Vector2::operator == (const Vector2 & v) const { return (x == v.x) && (y == v.y); }
    inline bool Vector2::operator != (const Vector2 & v) const { return (x != v.x) || (y != v.y); }


    // https://en.cppreference.com/w/cpp/language/operators : Binary arithmetic operators
    Vec2& operator+=(const Vec2& rhs) {
        lin += rhs.lin;
        ang += rhs.ang;
        return *this;
    }
    friend Vec2 operator+(Vec2 lhs, const Vec2& rhs) {
        lhs += rhs;
        return lhs;
    }
    Vec2& operator-=(const Vec2& rhs) {
        lin -= rhs.lin;
        ang -= rhs.ang;
        return *this;
    }
    friend Vec2 operator-(Vec2 lhs, const Vec2& rhs) {
        lhs -= rhs;
        return lhs;
    }
    Vec2& operator*=(const Vec2& rhs) {
        lin *= rhs.lin;
        ang *= rhs.ang;
        return *this;
    }
    friend Vec2 operator*(Vec2 lhs, const Vec2& rhs) {
        lhs *= rhs;
        return lhs;
    }
    Vec2& operator/=(const Vec2& rhs) {
        lin /= rhs.lin;
        ang /= rhs.ang;
        return *this;
    }
    friend Vec2 operator/(Vec2 lhs, const Vec2& rhs) {
        lhs /= rhs;
        return lhs;
    }

    friend Vec2 abs(Vec2 vec) {
        vec.lin = std::abs(vec.lin);
        vec.ang = std::abs(vec.ang);
        return vec;
    }

};
