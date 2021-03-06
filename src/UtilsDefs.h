#ifndef _UTILSDEFS_
#define _UTILSDEFS_


#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include <random>
#include <algorithm>
#include <iterator>
#include <thread>


using namespace std;
#define XAXIS 1
#define YAXIS -1
#define ALL 0

//How wide is the tile.
#define LEAFLEN (40*r)

#define Float float


inline Float min_(Float a, Float b)
{
	return a < b ? a : b;
}
inline Float max_(Float a, Float b)
{
	return a > b ? a : b;
}

class Point2D
{

public:
	Float x, y;
	Point2D(){}
	Point2D(Float _x, Float _y) :x(_x), y(_y){}
	//Point2D& operator=(const Point2D& target){ x = target.x; y = target.y; return *this; }
	Point2D operator+(const Point2D& b){ Point2D results; results.x = x + b.x; results.y = y + b.y; return results; }
	Point2D operator+(const Float b)
	{
		Point2D results;
		results.x = min_(1.0f, x + b);
		results.y = min_(1.0f, y + b);
		return results;
	}
	Point2D operator-(const Float b)
	{
		Point2D results;
		results.x = max_(0.0f, x - b);
		results.y = max_(0.0f, y - b);
		return results;
	}
	bool operator==(const Point2D &rhs) {
		// Check for self-assignment!
		if (this->x == rhs.x && this->y == rhs.y) {
			return true;
		}
		else return false;
	}

	friend ostream& operator<<(ostream& os, const Point2D& p)
	{
		os << p.x << " " << p.y << endl;
		return os;
	}
};

struct BBox
{
	BBox(){};
	BBox(Float x1, Float x2, Float y1, Float y2) :xmin(x1), xmax(x2), ymin(y1), ymax(y2){}
	Float xmin;
	Float xmax;
	Float ymin;
	Float ymax;
};

const BBox ide(0.0, 1.0, 0.0, 1.0);
const BBox gUnitGrid(0, 1, 0, 1);

inline bool within(BBox b, Point2D q)
{
	return ((q.x < b.xmax) && (q.x > b.xmin) && (q.y < b.ymax) && (q.y > b.ymin));
}

inline bool withinBox(BBox b, BBox q)
{
	return ((q.xmax <= b.xmax) && (q.xmin >= b.xmin) && (q.ymax <= b.ymax) && (q.ymin >= b.ymin));
}


inline Float DistanceSquared(Point2D p1, Point2D p2)
{
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}


inline Float min(Float a, Float b)
{
	return a < b ? a : b;
}
inline Float max(Float a, Float b)
{
	return a > b ? a : b;
}

#define X_AXIS 'x'
#define Y_AXIS 'y'

inline bool cmp_x(const Point2D &a, const Point2D &b)
{
	return a.x < b.x;
}
inline bool cmp_y(const Point2D &a, const Point2D &b)
{
	return a.y < b.y;
}



inline Float rand_normal(Float mean, Float stddev)
{//Box muller method
	static Float n2 = 0.0;
	static int n2_cached = 0;
	if (!n2_cached)
	{
		Float x, y, r;
		do
		{
			x = 2.0*rand() / RAND_MAX - 1;
			y = 2.0*rand() / RAND_MAX - 1;

			r = x*x + y*y;
		} while (r == 0.0 || r > 1.0);
		{
			Float d = sqrt(-2.0*log(r) / r);
			Float n1 = x*d;
			n2 = y*d;
			Float result = n1*stddev + mean;
			n2_cached = 1;
			return result;
		}
	}
	else
	{
		n2_cached = 0;
		return n2*stddev + mean;
	}
}


inline Float IntervalNormal(Float min, Float max)// use normal distribution instead
{
	Float rlt = rand_normal((max - min) / 2.0 + min, (max - min) / 10.0);
	if (min < rlt && rlt < max)return rlt;
	else if (rlt < min) return min;
	else return max;
	//return (min + max) / 2.0;
	//return (rand() % RAND_MAX) / Float(RAND_MAX) * (max - min) + min;
}

inline Float IntervalUniform2(Float min, Float max)// use normal distribution instead
{
	return (min + max) / 2.0;
}

inline Float IntervalUniform(Float min, Float max)// use normal distribution instead
{
	return (rand() % RAND_MAX) / Float(RAND_MAX) * (max - min) + min;
}

inline Point2D BoxUniform(BBox bbox)
{
	Point2D p;

	//p.x = IntervalUniform(bbox.xmin, bbox.xmax);
	//p.y = IntervalUniform(bbox.ymin, bbox.ymax);
	p.x = (rand() % RAND_MAX) / Float(RAND_MAX) * (bbox.xmax - bbox.xmin) + bbox.xmin;
	p.y = (rand() % RAND_MAX) / Float(RAND_MAX) * (bbox.ymax - bbox.ymin) + bbox.ymin;
	return p;
}



inline bool Circumcenter(const Point2D& p0, const Point2D& p1, const Point2D& p2, Point2D& center, Float& rad2){
	Float dA, dB, dC, aux1, aux2, div;

	dA = p0.x * p0.x + p0.y * p0.y;
	dB = p1.x * p1.x + p1.y * p1.y;
	dC = p2.x * p2.x + p2.y * p2.y;

	aux1 = (dA*(p2.y - p1.y) + dB*(p0.y - p2.y) + dC*(p1.y - p0.y));
	aux2 = -(dA*(p2.x - p1.x) + dB*(p0.x - p2.x) + dC*(p1.x - p0.x));
	div = (2 * (p0.x*(p2.y - p1.y) + p1.x*(p0.y - p2.y) + p2.x*(p1.y - p0.y)));

	if (div == 0){
		return false;
	}

	center.x = aux1 / div;
	center.y = aux2 / div;
	rad2 = ((center.x - p0.x)*(center.x - p0.x) + (center.y - p0.y)*(center.y - p0.y));

	return true;
}

//global variables

extern vector<Point2D> primaryPoints;
extern vector<Point2D> secondaryPoints;
extern BBox primary_right;
extern BBox primary_down;
extern BBox secondary_left;
extern BBox secondary_up;

extern BBox mid_NE;
extern BBox mid_SW;

extern vector<Point2D> centerPoints;

#endif