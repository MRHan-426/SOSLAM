#include "Polygon.hpp"

namespace gtsam_soslam
{
float distPoint(cv::Point2d v,cv::Point2d w) { 
	return sqrtf((v.x - w.x)*(v.x - w.x) + (v.y - w.y)*(v.y - w.y)) ;
}

bool segementIntersection(cv::Point2d p0,cv::Point2d p1,cv::Point2d p2,cv::Point2d p3,cv::Point2d * intersection) {
	cv::Point2d s1, s2;
	s1 = cv::Point2d(p1.x - p0.x, p1.y - p0.y);
	s2 = cv::Point2d(p3.x - p2.x, p3.y - p2.y);

	float s10_x = p1.x - p0.x;
	float s10_y = p1.y - p0.y;
	float s32_x = p3.x - p2.x;
	float s32_y = p3.y - p2.y;
	float denom = s10_x * s32_y - s32_x * s10_y;

	if(denom == 0) {
		return false;
	}

	bool denom_positive = denom > 0;

	float s02_x = p0.x - p2.x;
	float s02_y = p0.y - p2.y;
	float s_numer = s10_x * s02_y - s10_y * s02_x;

	if((s_numer < 0.f) == denom_positive) {
		return false;
	}

	float t_numer = s32_x * s02_y - s32_y * s02_x;
	if((t_numer < 0) == denom_positive) {
		return false;
	}

	if((s_numer > denom) == denom_positive || (t_numer > denom) == denom_positive) {
		return false;
	}

	float t = t_numer / denom;

	*intersection = cv::Point2d(p0.x + (t * s10_x), p0.y + (t * s10_y) );
	return true;
}

bool pointInPolygon(cv::Point2d p,const cv::Point2d * points,int n) {
	int i, j ;
	bool c = false;
	for(i = 0, j = n - 1; i < n; j = i++) {
		if( ( (points[i].y >= p.y ) != (points[j].y >= p.y) ) &&
			(p.x <= (points[j].x - points[i].x) * (p.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
			)
			c = !c;
	}
	return c;
}


float computeArea(const cv::Point2d * pt,int n ) {
	float area0 = 0.f;
	for (int i = 0 ; i < n ; i++ ) {
		int j = (i+1)%n;
		area0 += pt[i].x * pt[j].y;
		area0 -= pt[i].y * pt[j].x;
	}
	return 0.5f * fabs(area0);
}

struct PointAngle{
	cv::Point2d p;
	float angle;
};

cv::Point2d Polygon::getCenter() const {
	cv::Point2d center;
	center.x = 0;
	center.y = 0;
	for (int i = 0 ; i < n ; i++ ) {
		center.x += pt[i].x;
		center.y += pt[i].y;
	}
	center.x /= n;
	center.y /= n;
	return center;
}

static int comp_point_with_angle(const void * a, const void * b) {
	if ( ((PointAngle*)a)->angle <  ((PointAngle*)b)->angle ) return -1;
	else if ( ((PointAngle*)a)->angle > ((PointAngle*)b)->angle ) return 1;
	else //if ( ((PointAngle*)a)->angle == ((PointAngle*)b)->angle ) return 0;
		return 0;
}

float Polygon::area() const {
	return computeArea(pt,n);
}

void Polygon::pointsOrdered() {
	if( n <= 0) return;
	cv::Point2d center = getCenter();
	PointAngle pc[MAX_POINT_POLYGON];
	for (int i = 0 ; i < n ; i++ ) {
		pc[i].p.x = pt[i].x;
		pc[i].p.y = pt[i].y;
		pc[i].angle = atan2f((float)(pt[i].y - center.y),(float)(pt[i].x - center.x));
	}
	qsort(pc,n,sizeof(PointAngle),comp_point_with_angle);
	for (int i = 0 ; i < n ; i++ ) {
		pt[i].x = pc[i].p.x;
		pt[i].y = pc[i].p.y;
	}
}

bool Polygon::pointIsInPolygon(cv::Point2d p) const {
	return pointInPolygon(p,pt,n);
}

void intersectPolygon( const cv::Point2d * poly0, int n0,const cv::Point2d * poly1,int n1, Polygon & inter )  {
	inter.clear();
	for (int i = 0 ; i < n0 ;i++) {
		if( pointInPolygon(poly0[i],poly1,n1) ) {
			inter.add(poly0[i]);
		}
	}

	for (int i = 0 ; i < n1 ;i++) { 
		if( pointInPolygon(poly1[i],poly0,n0) ) 
			inter.add(poly1[i]);
	}

	for (int i = 0 ; i < n0 ;i++) {
		cv::Point2d p0,p1,p2,p3;
		p0 = poly0[i];
		p1 = poly0[(i+1)%n0];
		for (int j = 0 ; j < n1 ;j++) {
			p2 = poly1[j];
			p3 = poly1[(j+1)%n1];
			cv::Point2d pinter;
			if(segementIntersection(p0,p1,p2,p3,&pinter)) {
				inter.add(pinter);
			}
		}
	}
	inter.pointsOrdered();
}

void intersectPolygon( const Polygon & poly0, const Polygon & poly1, Polygon & inter ) {
	intersectPolygon(&poly0[0],poly0.size(),&poly1[0],poly1.size(),inter);
}

//------------------------------------------------------------------------------------------------------------
//             SHPC :  Sutherland-Hodgeman-Polygon-Clipping Algorihtm
//------------------------------------------------------------------------------------------------------------
static inline int cross(const cv::Point2d* a,const cv::Point2d* b) {
	return a->x * b->y - a->y * b->x;
}

static inline cv::Point2d* vsub(const cv::Point2d* a,const cv::Point2d* b, cv::Point2d* res) {
	res->x = a->x - b->x;
	res->y = a->y - b->y;
	return res;
}

static int line_sect(const cv::Point2d* x0,const cv::Point2d* x1,const cv::Point2d* y0,const cv::Point2d* y1, cv::Point2d* res) {
	cv::Point2d dx, dy, d;
	vsub(x1, x0, &dx);
	vsub(y1, y0, &dy);
	vsub(x0, y0, &d);
	float dyx = (float)cross(&dy, &dx);
	if (!dyx) return 0;
	dyx = cross(&d, &dx) / dyx;
	if (dyx <= 0 || dyx >= 1) return 0;
	res->x = int(y0->x + dyx * dy.x);
	res->y = int(y0->y + dyx * dy.y);
	return 1;
}

static int left_of(const cv::Point2d* a,const cv::Point2d* b,const cv::Point2d* c) {
	cv::Point2d tmp1, tmp2;
	int x;
	vsub(b, a, &tmp1);
	vsub(c, b, &tmp2);
	x = cross(&tmp1, &tmp2);
	return x < 0 ? -1 : x > 0;
}

static void poly_edge_clip(const Polygon* sub,const  cv::Point2d* x0,const  cv::Point2d* x1, int left, Polygon* res) {
	int i, side0, side1;
	cv::Point2d tmp;
	const cv::Point2d* v0 = sub->pt+ sub->n - 1;
	const cv::Point2d* v1;
	res->clear();

	side0 = left_of(x0, x1, v0);
	if (side0 != -left) res->add(*v0);

	for (i = 0; i < sub->n; i++) {
		v1 = sub->pt + i;
		side1 = left_of(x0, x1, v1);
		if (side0 + side1 == 0 && side0)
			/* last point and current straddle the edge */
			if (line_sect(x0, x1, v0, v1, &tmp))
				res->add(tmp);
		if (i == sub->n - 1) break;
		if (side1 != -left) res->add(*v1);
		v0 = v1;
		side0 = side1;
	}
}

static int poly_winding(const Polygon* p) {
	return left_of(p->pt, p->pt + 1, p->pt + 2);
}

void intersectPolygonSHPC(const Polygon * sub,const Polygon* clip,Polygon* res) {
	int i;
	Polygon P1,P2;
	Polygon * p1 = &P1;
	Polygon * p2 = &P2;
	Polygon * tmp ;

	int dir = poly_winding(clip);
	poly_edge_clip(sub, clip->pt + clip->n - 1, clip->pt, dir, p2);
	for (i = 0; i < clip->n - 1; i++) {
		tmp = p2; p2 = p1; p1 = tmp;
		if(p1->n == 0) {
			p2->n = 0;
			break;
		}
		poly_edge_clip(p1, clip->pt + i, clip->pt + i + 1, dir, p2);
	}
	res->clear();
	for (i = 0 ; i < p2->n ; i++) res->add(p2->pt[i]);
}

void intersectPolygonSHPC(const Polygon & sub,const Polygon& clip,Polygon& res) {
	intersectPolygonSHPC(&sub,&clip,&res);
}
}