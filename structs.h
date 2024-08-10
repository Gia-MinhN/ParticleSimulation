struct Vector2
{
    float x, y;

    // Scalar

    inline Vector2 operator+(float val) {
        return {x + val, y + val};
    }

    inline Vector2 operator-(float val) {
        return {x - val, y - val};
    }

    inline Vector2 operator*(float val) {
        return {x*val, y*val};
    }

    inline Vector2 operator/(float val) {
        return {x/val, y/val};
    }

    // Vector

    inline Vector2 operator+(Vector2 other) {
        return {x + other.x, y + other.y};
    }

    inline Vector2 operator-(Vector2 other) {
        return {x - other.x, y - other.y};
    }

    inline Vector2 operator*(Vector2 other) {
        return {x*other.x, y*other.y};
    }

    inline Vector2 operator/(Vector2 other) {
        return {x/other.x, y/other.y};
    }

    // Equals

    inline bool operator==(Vector2 other) {
        return x == other.x && y == other.y;
    }
    
};

struct Particle {
    float radius, mass;
    Vector2 position, velocity;

};