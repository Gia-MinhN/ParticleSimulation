#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <SFML/Graphics.hpp>

#include "structs.h"

float distance(Vector2 a, Vector2 b) {
    return sqrt(pow((b.x - a.x), 2) + pow((b.y - a.y), 2));
}

float magnitude(Vector2 vect) {
    return sqrt(pow(vect.x, 2) + pow(vect.y, 2));
}

Vector2 unit(Vector2 vect){
    if(vect.x == 0 && vect.y == 0) {
        return {0, 0};
    }
    return vect/magnitude(vect);
}

float angle(Vector2 a, Vector2 b) {
    return atan2(a.y - b.y, a.x - b.x);
}

float dot(Vector2 a, Vector2 b) {
    return a.x*b.x + a.y*b.y;
}

Vector2 reflect(Vector2 vect, Vector2 norm) {
    return (vect - norm*2*dot(norm, vect));
}
