﻿// TF_Rotation.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#define _USE_MATH_DEFINES
#include "math.h"

#define RAD2DEG(x)   x*180/M_PI
#define DEG2RAD(x)   x/180*M_PI

typedef struct
{
	double x;
	double y;
	double theta;
} Pose2D;

typedef struct
{
	double x;
	double y;
} Point2D;


Pose2D    base_link_origin;
Point2D   base_link_Point2D, base_link_map_Point2D, bl2blm, blm2bl;

//base_link_Point2D : 상대좌표 base_link 
//base_link_map_Point2D : 절대좌표 baselink
//base_link_origin : 절대좌표 상 위치



double angle_degree;	
double angle_radian;

double Rotation_matrix_inverse[2][2];
double Rotation_matrix[2][2];

void set_rotation_matrix(double m_angle_degree)
{
	angle_radian = DEG2RAD(m_angle_degree);
	Rotation_matrix[0][0] = cos(angle_radian); Rotation_matrix[0][1] = -sin(angle_radian);
	Rotation_matrix[1][0] = sin(angle_radian); Rotation_matrix[1][1] = cos(angle_radian);

}

void set_rotation_matrix_inverse(double m_angle_degree)		
{
	angle_radian = DEG2RAD(m_angle_degree);

	Rotation_matrix_inverse[0][0] = cos(angle_radian);  Rotation_matrix_inverse[0][1] = sin(angle_radian);
	Rotation_matrix_inverse[1][0] = -sin(angle_radian); Rotation_matrix_inverse[1][1] = cos(angle_radian);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//baselink -> baselinkMap


void TF_base_link_base_link_map(Point2D *base_link_Point2D, Point2D* base_link_map_point2d, Pose2D base_link_origin){
	set_rotation_matrix_inverse(base_link_origin.theta);
	base_link_map_point2d->x = (Rotation_matrix_inverse[0][0] * base_link_Point2D->x) + (Rotation_matrix_inverse[0][1] * base_link_Point2D->y);
	base_link_map_point2d->y = (Rotation_matrix_inverse[1][0] * base_link_Point2D->x) + (Rotation_matrix_inverse[1][1] * base_link_Point2D->y);
	printf("\nTF_base_link  -->   base_link_map\n");
	printf("Base_Link -> Base_Link_Map: %6.3lf  %6.3lf\n", base_link_map_point2d->x, base_link_map_point2d->y);

}	

void move2map(Point2D* base_link_map_point2d, Pose2D base_link_origin) {
	base_link_map_point2d->x = base_link_map_point2d->x + base_link_origin.x;
	base_link_map_point2d->y = base_link_map_point2d->y + base_link_origin.y;
	printf("\n base_link_origin + map\n");
	printf("Base_Link -> Base_Link_Map: %6.3lf  %6.3lf\n", base_link_map_Point2D.x, base_link_map_Point2D.y);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//baselinkMap -> baselink

void TF_base_link_map_base_link(Point2D *base_link_Point2D, Point2D* base_link_map_point2d, Pose2D base_link_origin){
	set_rotation_matrix(base_link_origin.theta);
	base_link_Point2D->x = (Rotation_matrix[0][0] * base_link_Point2D->x) + (Rotation_matrix[0][1] * base_link_Point2D->y);
	base_link_Point2D->y = (Rotation_matrix[1][0] * base_link_Point2D->x) + (Rotation_matrix[1][1] * base_link_Point2D->y);
	printf("\nTF_base_link  -->  map_base_link55\n");
	printf("Transformed Point Matrix: %6.3lf  %6.3lf\n", base_link_Point2D->x, base_link_Point2D->y);
}

void move2baselink(Point2D* base_link_Point2D, Point2D* base_link_map_point2d, Pose2D base_link_origin) {
	base_link_Point2D->x = base_link_map_point2d->x - base_link_origin.x;
	base_link_Point2D->y = base_link_map_point2d->y -base_link_origin.y;
	printf("\nTF_base_link  -->  map_base_link\n");
	printf("Transformed Point Matrix: %6.3lf  %6.3lf\n", base_link_Point2D->x, base_link_Point2D->y);
}


int _tmain(int argc, _TCHAR* argv[])
{

	base_link_origin.x = 5.0;
	base_link_origin.y = 4.0;
	base_link_origin.theta = -90;

	base_link_Point2D.x = 1.0;//-0.707;
	base_link_Point2D.y = -1.0;//2.121;


	printf("\nTF_base_link  -->   base_link_map\n");
	printf("Base_Link -> Base_Link_Map: %6.3lf  %6.3lf\n", base_link_map_Point2D.x, base_link_map_Point2D.y);



	TF_base_link_base_link_map(&base_link_Point2D, &base_link_map_Point2D, base_link_origin);
	move2map(&base_link_map_Point2D, base_link_origin);
	move2baselink(&base_link_Point2D, &base_link_map_Point2D, base_link_origin);
	TF_base_link_map_base_link(&base_link_Point2D, &base_link_map_Point2D, base_link_origin);

	//map to baselink //inverse


	return 0;
}
