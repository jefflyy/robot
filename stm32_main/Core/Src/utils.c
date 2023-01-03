/* UTILS */

#include "utils.h"

int min(int a, int b){
    return a < b ? a : b;
}

int max(int a, int b){
    return a > b ? a : b;
}

int abs(int x){
    return x > 0 ? x : -x;
}

int clip(int a, int b, int c){
    return min(max(a, b), c);
}
