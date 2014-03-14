//rayTracing

#include "stdafx.h"
#include "math.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "objects.h"
#include <ctime>
#define minWeight 0.1
#define MAX_RECUR 5
#define BACKGROUND colorScalar(20,0,0)
#define XMAX	1000
#define XMIN	-1000
#define YMAX	10000
#define YMIN	-1000
#define ZMAX	1000
#define ZMIN	-50
#define TEXTURETHRESH 10
#define DebugMode 
#define HEIGHT 480
#define WIDTH 640
using namespace std;
using namespace cv;


_point vPoint = _point(0,0,0);
float panelx = 0;
float panelz = 0;
float panely = 0;
float innerProduct(_vector a, _vector b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z;
}


_point computeIntersect(_point start, _vector lawVector,vector<sphere> spheres, _vector & interLawVector, int & index)
{
	float minDis = 10000;
	float minIndex = 0;
	float loc2 = 0;
	float tca =0, thc2 = 0;
	_point its = _point(0,0,0);
	if (lawVector == _vector(0,0,0) ) 
		return its;
	
	for( unsigned int i = 0; i < spheres.size(); i++)
	{
		sphere currentSphere = spheres.at(i);
		_vector loc = _vector(currentSphere.center.x - start.x,currentSphere.center.y - start.y,currentSphere.center.z - start.z);
		loc2 = innerProduct(loc,loc);
		if (loc2 > (currentSphere.radius)*(currentSphere.radius))  // start outside the sphere
		{
			lawVector.normalize();
			tca  = innerProduct(loc,lawVector);
			if (tca < 0)
				continue;
			thc2 = (currentSphere.radius)*(currentSphere.radius) - loc2 + tca*tca;
			if (thc2 >= 0)
			{
				//float t = tca + sqrt(thc2);
				//_point inter0 = _point(start.x+lawVector.x*t,start.y+lawVector.y*t,start.z+lawVector.z*t);
				//_points.push_back(inter0);
				float t = tca - sqrt(thc2);			
				if (minDis > t)
					{
						minDis = t;
						its = _point(start.x+lawVector.x*t,start.y+lawVector.y*t,start.z+lawVector.z*t);	
						float xx = (start.x+lawVector.x*t - currentSphere.center.x)/currentSphere.radius;
						float yy = (start.y+lawVector.y*t - currentSphere.center.y)/currentSphere.radius;
						float zz = (start.z+lawVector.z*t - currentSphere.center.z)/currentSphere.radius;
						interLawVector = _vector(xx,yy,zz);
						interLawVector.normalize();
					}		
				  t = tca + sqrt(thc2);			
					if (minDis > t)
					{
						minDis = t;
						its = _point(start.x+lawVector.x*t,start.y+lawVector.y*t,start.z+lawVector.z*t);	
						float xx = (start.x+lawVector.x*t - currentSphere.center.x)/currentSphere.radius;
						float yy = (start.y+lawVector.y*t - currentSphere.center.y)/currentSphere.radius;
						float zz = (start.z+lawVector.z*t - currentSphere.center.z)/currentSphere.radius;
						interLawVector = _vector(xx,yy,zz);
						interLawVector.normalize();
					}		
			  }			
		}
		else // start inside the sphere
		{
			tca  = innerProduct(loc,lawVector);
			if (tca < 0)
			{
				thc2 = (currentSphere.radius)*(currentSphere.radius) - loc2 + tca * tca;
				float t = sqrt(thc2) - tca;			
				if (minDis > t)
					{
						minDis = t;
						its = _point(start.x+lawVector.x*t,start.y+lawVector.y*t,start.z+lawVector.z*t);	
						float xx = (start.x+lawVector.x*t - currentSphere.center.x)/currentSphere.radius;
						float yy = (start.y+lawVector.y*t - currentSphere.center.y)/currentSphere.radius;
						float zz = (start.z+lawVector.z*t - currentSphere.center.z)/currentSphere.radius;
						interLawVector = _vector(xx,yy,zz);
						interLawVector.normalize();
					}			
			}
			
			else
			{
				thc2 = (currentSphere.radius)*(currentSphere.radius) - loc2 + tca * tca;
				float t = sqrt(thc2) + tca;			
				if (minDis > t)
					{
						minDis = t;
						its = _point(start.x+lawVector.x*t,start.y+lawVector.y*t,start.z+lawVector.z*t);	
						float xx = (start.x+lawVector.x*t - currentSphere.center.x)/currentSphere.radius;
						float yy = (start.y+lawVector.y*t - currentSphere.center.y)/currentSphere.radius;
						float zz = (start.z+lawVector.z*t - currentSphere.center.z)/currentSphere.radius;
						interLawVector = _vector(xx,yy,zz);
						interLawVector.normalize();
					}			

			}
		}
	}

	
	return its;
}

colorScalar Hall(_point intersect,_vector lawVector, _vector & interLawVector,vector<sphere>lightSource, vector<sphere> spheres)
{
	
	colorScalar color = colorScalar(0,0,0);
	if (intersect == _point(0,0,0))
		return color;
	unsigned int sphereIndex = 0;
	for( unsigned int i = 0; i < spheres.size(); i++)
	{
		_point onSphere = intersect - spheres.at(i).center;
		float minus =  onSphere.x *onSphere.x +  onSphere.y *onSphere.y +  onSphere.z *onSphere.z - spheres.at(i).radius * spheres.at(i).radius;
		if (abs(minus) < 0.1)
			sphereIndex = i;

	}
	for( unsigned int i = 0; i < lightSource.size(); i++)
	{
		sphere currentSource = lightSource.at(i);
		_vector L = _vector(currentSource.center.x - intersect.x,	currentSource.center.y - intersect.y, 	currentSource.center.z - intersect.z);
		colorScalar Ip = currentSource.lightIntense;
		float Ka = spheres.at(sphereIndex).Ka;
		float Ks = spheres.at(sphereIndex).Ks;
		float Kds = spheres.at(sphereIndex).Kds;
		int n = 500;
		_vector V = lawVector;
		L.normalize();
		V.normalize();
		_vector H = L + V;
		H.normalize();
		int ambientR = 0,ambientG = 10, ambientB = 0;
	
		// reflection
		color.b = ambientB + Ip.b * Kds * abs(innerProduct(L,interLawVector)) + Ip.b * Ks * pow(abs(innerProduct(H,interLawVector)),n);
		color.g = ambientG + Ip.g * Kds * abs(innerProduct(L,interLawVector)) + Ip.g * Ks * pow(abs(innerProduct(H,interLawVector)),n);
		color.r = ambientR + Ip.r * Kds * abs(innerProduct(L,interLawVector)) + Ip.r * Ks * pow(abs(innerProduct(H,interLawVector)),n);
		
			if (color.b > 255)
				color.b = 255;
			if (color.g > 255)
				color.g = 255;
			if (color.r > 255)
				color.r = 255;
		//Whitted
		
		//if ( innerProduct(lawVector,interLawVector) > 0)
		
		/*color.b =   (color.b  + Is.b * Ks> 255)?	255:	color.b + Is.b * Ks;
		color.g =  (color.g + Is.g * Ks > 255)?		255:	color.g + Is.g * Ks;
		color.r =   (color.r + Is.r * Ks > 255)?	255:	color.r + Is.r * Ks;*/
		

		//Mat wall = imread("wall.jpg");
		//resize(wall,wall,Size(WIDTH,HEIGHT));
		////imshow("wall",wall);
		////waitKey(0);
		//float scale_x = ( XMAX - intersect.x ) / L.x ;
		//_point temp_x = _point(intersect.x+L.x * scale ,intersect.y +L.y *scale,intersect.z +L.z *scale) ;		
		//
		//colorScalar It = colorScalar(0,0,0);
		//It.b = wall.at<Vec3b>(temp_x.y,temp_x.z)[0]; 
		//It.g = wall.at<Vec3b>(temp_x.y,temp_x.z)[1]; 
		//It.r = wall.at<Vec3b>(temp_x.y,temp_x.z)[2]; 
		//float Kt0 = 0.1;
		////if ( innerProduct(lawVector,interLawVector) > 0)
		//
		//color.b =   (color.b  + It.b * Kt0> 255)?	255:	color.b + It.b * Kt0;
		//color.g =  (color.g + It.g * Kt0 > 255)?		255:	color.g + It.g * Kt0;
		//color.r =   (color.r + It.r * Kt0 > 255)?	255:	color.r + It.r * Kt0;

		// transmission, Hall model
		lawVector.normalize();
		interLawVector.normalize();
		float Kt = spheres.at(sphereIndex).Kt;
		float Kdt = spheres.at(sphereIndex).Kdt;
		float ita = 2.0;
		float cos_theta_1 = innerProduct(lawVector,interLawVector);
		//assert((1 - pow(cos_theta_1,2)) < 1);
		
		float cos_theta_2 = sqrt(	1 - (1/ita/ita)*(1 - pow(cos_theta_1,2))	);
		assert(cos_theta_2 <= 1);
		n = 100;
		//Ip = colorScalar(0,25,0);
		_vector T = lawVector *(-1/ita)  - interLawVector*(cos_theta_2 - (1/ita)*cos_theta_1);
		T.normalize();
		color.b  =( color.b + Ip.b * Kdt * (-abs(innerProduct(interLawVector,L)))+ Kt * Ip.b * pow(innerProduct(T,V),n) > 255) ? 255:
			color.b + Ip.b * Kdt * (-abs(innerProduct(interLawVector,L)))+ Kt * Ip.b * pow(innerProduct(T,V),n) ;
		color.g  =( color.g + Ip.g * Kdt * (-abs(innerProduct(interLawVector,L))) + Kt * Ip.g * pow(innerProduct(T,V),n)> 255) ? 255:
			color.g + Ip.g * Kdt * (-abs(innerProduct(interLawVector,L))) + Kt * Ip.g * pow(innerProduct(T,V),n);
		color.r  =( color.r + Ip.r * Kdt * (-abs(innerProduct(interLawVector,L)))+ Kt * Ip.r * pow(innerProduct(T,V),n)  > 255) ? 255:
			color.r + Ip.r * Kdt * (-abs(innerProduct(interLawVector,L)))+ Kt * Ip.r * pow(innerProduct(T,V),n) ;

	   assert(color.b <=255 && color.g <= 255 && color.r<=255);
		

	}
	return color;
}

colorScalar raytracing( vector<sphere> lightSource,_point start,_vector lawVector ,float weight, vector<sphere> spheres, int & cnt )
{
	colorScalar color;
	float w_r = 0.2, w_float = 0.8;

	if (cnt > MAX_RECUR)
		return BACKGROUND;
	cnt++;

	if ( weight < minWeight)
		{
			return color = colorScalar(0,0,0);
		}
	else
	{
		_vector interLawVector(0,0,0);
		int index = 0;
		_point intersect = computeIntersect(start, lawVector,spheres,interLawVector,index);
		
		if (	intersect == _point(0,0,0) )
			{
				
			 if (start.z + TEXTURETHRESH * lawVector.z < ZMIN )        // if this ray reaches background, add texture
				{
					float scale = ( ZMIN - start.z ) / lawVector.z ;
					_point temp = _point(start.x+lawVector.x * scale ,start.y +lawVector.y *scale,start.z +lawVector.z *scale) ;					
					cnt = cnt + MAX_RECUR;
					colorScalar Result ;
					
					if (  (int)(temp.x/ HEIGHT *10 )%5 == 0
								|| (int)(temp.y/ WIDTH *10 )%5 == 0 )
						Result = colorScalar(30,1,1);
					else
						Result = colorScalar(200,150,0);
					
					return Result;
					
				}
				else if ( start.y  + TEXTURETHRESH * lawVector.y > YMAX)
				{
					
					float scale = ( XMAX - start.y ) / lawVector.y ;
					_point temp = _point(start.x+lawVector.x * scale ,start.y +lawVector.y *scale,start.z +lawVector.z *scale) ;
					
					cnt = cnt + MAX_RECUR;
					colorScalar Result ;
					if (  (int(temp.x/ HEIGHT * 8) + int(temp.z/WIDTH*8) ) %2 == 0)
						Result = colorScalar(201,174,255);
					else
						Result = colorScalar(255,0,0);
					return Result;		
				}
				
				
				else
					{
						cnt = cnt + MAX_RECUR;
						return color = BACKGROUND;
					}
			}

		else
		{
			
			colorScalar I_local = Hall(intersect,lawVector, interLawVector,lightSource,spheres);
#ifdef debugMode
			cout << I_local.b << " " << I_local.g << " "<<I_local.r << " "<<endl;
#endif
			_vector R ;
			lawVector.normalize();
			R = interLawVector * 2*abs(innerProduct(lawVector,interLawVector)) - lawVector; // page.142
			R.normalize();
			colorScalar I_r;
			if ( innerProduct(R,interLawVector) < 0 )
				colorScalar I_r =  colorScalar(0,0,0);
			else
				I_r = raytracing(lightSource,	intersect,	R,	weight*w_r,	spheres,	cnt);


			_vector T ;
			float ita = 2.0;
			float cos_theta_1 = abs(innerProduct(lawVector,	interLawVector));
			float cos_theta_2 = sqrt(1 - (1/ita/ita)*(1 - pow(cos_theta_1,2)));
			T = lawVector *(-1/ita)  - interLawVector*(cos_theta_2 - (1/ita)*cos_theta_1);
			T.normalize();
			colorScalar I_float;
			if ( innerProduct(T,interLawVector) < 0 )
				 I_float =  colorScalar(0,0,0);
			else
				 I_float = raytracing(lightSource,	intersect,	T,	weight * w_float,	spheres,	cnt);

			unsigned int sphereIndex = 0;
			for( unsigned int i = 0; i < spheres.size(); i++)
			{
				_point onSphere = intersect - spheres.at(i).center;
				float minus =  onSphere.x *onSphere.x +  onSphere.y *onSphere.y +  onSphere.z *onSphere.z - spheres.at(i).radius * spheres.at(i).radius;
				if (abs(minus) < 0.1)
					sphereIndex = i;

			}
			float Kss = spheres.at(sphereIndex).Kss;
			float Ktt = spheres.at(sphereIndex).Ktt;
			color = I_local/5 +  I_r * Kss + I_float * Ktt ;
			if (color.b > 255)
				color.b = 255;
			if (color.g > 255)
				color.g = 255;
			if (color.r > 255)
				color.r = 255;
			assert(color.b <=255 && color.g <= 255 && color.r<=255);
		}
	}
	
	return color;
}




int main(int argc, char * argv[])
{
	Mat image;
	image.create(480,640,CV_8UC3);
    if( image.empty() )
    {
        std::cout << "Image empty. \n";
        return 0;
    }
    namedWindow( "image", 0 );
	
	int width = WIDTH, height = HEIGHT;
	
	// create landscape and light source

	 // this is light source
	clock_t m_begin = clock();
	vector<sphere> spheres;
	vector<sphere> lightSource;


	//															float ka, float ks, float kds, float kt, float kdt, float kss, float ktt
	spheres.push_back(sphere(600,300,400,400,colorScalar(0,0,0),	0.1,	0.8,	0.9,	0.9,	0.01,           0.6,       0.1));
	spheres.push_back(sphere(300,100,400,200,colorScalar(0,0,0),	0.1,	0.008,	0.01,	0.9,	0.09,            0.01,       1));
	spheres.push_back(sphere(300,700,500,200,colorScalar(0,0,0),	0.1,	0.008,	0.01,	9,	0.09,            20,        5));
	spheres.push_back(sphere(600,0,100,100,colorScalar(0,0,0),		0.1,	0.008,	0.1,	0.9,	0.09,            0.5,        0.5));


	//lightSource.push_back(sphere(330,400,100,0.1,colorScalar(200,200,0)));
	lightSource.push_back(sphere(300,120,800,20,colorScalar(200,200,200)));


	// start ray tracing
	for (int j = 0; j < height; j++)
		for(int i = 0; i < width; i++)
		{

			float viewX = -100, viewY = 300, viewZ = 300;
			_point viewPoint = _point(viewX,viewY,viewZ);
			vPoint = viewPoint;
			_vector lawVector = _vector(panelx - viewX, panely+i -viewY, panelz+j - viewZ);

			float weight = 1;
			_point start = _point(panelx,panely+i,panelz+j);
		    colorScalar currentColor ;
			int cnt = 0;
			
			currentColor = raytracing(lightSource,start, lawVector, weight,spheres, cnt);
			
			assert(currentColor.b <=255 &&currentColor.g <=255 &currentColor.r<=255 );
			image.at<Vec3b>(height - 1 - j,i)[0] = (uchar)currentColor.b;
			image.at<Vec3b>(height - 1 - j,i)[1] = (uchar)currentColor.g;
			image.at<Vec3b>(height - 1 - j,i)[2] = (uchar)currentColor.r;

			//*(image.data + image.step[0] *(int) j + image.step[1] *(int) i + image.elemSize1()*0) = currentColor.b/255.0;
			//*(image.data + image.step[0] *(int) j + image.step[1] *(int) i + image.elemSize1()*1) = currentColor.g/255.0;
			//*(image.data + image.step[0] *(int) j + image.step[1] *(int) i + image.elemSize1()*2) = currentColor.r/255.0;
			
		}
		clock_t m_end = clock();
		std::cout << "render time: " << (float)(m_end -m_begin)/CLOCKS_PER_SEC<< " seconds" <<endl;
        imshow("image",image);
		imwrite("image.jpg",image);
		waitKey(0);
		
	return 0;
}









