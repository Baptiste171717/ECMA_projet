#pragma once
#include <fstream>
#include<iostream>
#include <vector>


class Point {

    int x,y;

public :

    Point(int new_x, int new_y){
        x = new_x;
        y = new_y;
    }

    Point(){
        x = -1;
        y = -1;
    }

    bool operator<(const Point& other) const {
        return y < other.y;
    }

    int get_x()const{return x;}

    int get_y()const{return y;}

    void set_x(int new_x){x=new_x;}

    void set_y(int new_y){y = new_y;}

};
