/*** note: general equations for cicle are from http://www.ambrsoft.com/trigocalc/circle3d.htm 
 ***/

#include "server3.hpp"
#include <math.h>
#include <vector>


/*** 
 * Following functions and variables are available in server3.hpp
 * ImagePPM cameraView; Our standard 
 * setMotorSpeeds(double dvx,double dvy) ; increments(or decrements if negative)
 *     current motors speed
 * get
 ***/
int vertical(int row, int col, ImagePPM& output){
	return (get_pixel(output, row-1, col-1, 0)*(-1)+get_pixel(output, row-1, col, 0)*(-2)+get_pixel(output, row-1, col+1,0)*(-1)+get_pixel(output, row+1, col-1,0)+get_pixel(output, row+1, col, 0)*(2)+get_pixel(output, row+1, col+1, 0));
}
 int horizintal(int row, int col, ImagePPM& output){
	return (get_pixel(output, row-1, col-1, 0)*(-1)+get_pixel(output, row-1, col+1, 0)+get_pixel(output, row, col-1, 0)*(-2)+get_pixel(output, row, col+1, 0)*(2)+get_pixel(output, row+1, col-1, 0)*(-1)+get_pixel(output, row+1, col+1, 0));
}

bool isRed(double row, double col){
	int red=0;
	int other=0;
	for(int irow=-1; irow<2; irow++){
		for(int icol=-1; icol<2; icol++){
			red+=get_pixel(cameraView, row+irow, col+icol, 0);
			other+=(get_pixel(cameraView, row+irow, col+icol, 1)+get_pixel(cameraView, row+irow, col+icol, 2))/2.0;

		}
	}
	if(red<other*1.15){return false;}
	return true;
}
	
float findRadius(int row, int col, int sunRow, int sunCol){
	return sqrt((row-sunRow)*(row-sunRow) + (col-sunCol)*(col-sunCol));
	}

Point* findCenter(float row1, float col1, float row2, float col2, float row3, float col3){
//std::cout<<"row1="<<row1<<"   col2="<<col2<<"   row3="<<row3<<std::endl;
  float A = 2*col1-2*col2;
  float B = 2*row1-2*row2;
  float C = col2*col2 - col1*col1 + row2*row2 - row1*row1;
  float D = 2*col1-2*col3;
  float E = 2*row1 - 2*row3;
  float F = col3*col3 - col1*col1 +row3*row3 - row1*row1;
  
  float x = (-C*E + F*B)/(A*E - B*D);
  float y = (-A*F +C*D)/(A*E - B*D);
  Point* result = new Point();
  result->row = y;
  result->col = x;
  
  return result;
}
int sunRise(float r, float x0, float y0, int yAim){
	//float t = (asin((yAim-y0)/r*1.0)/(1.0/24.0));
	 //return x0-r*cos((1/24)*t);//x
	 
	 return x0-sqrt(r*r - (yAim-y0)*(yAim-y0));
	
	}
Point* sunPosition(float r, float t, float x0, float y0){
	Point* result = new Point();
	result->col =x0+r*cos(((1.1)/24.0)*t);//x
	result->row =y0+r*sin(((1.1)/24.0)*t);//y
	return result;
	}
/***************MAIN LOOP**************/
int main(int argc, char **argv)
{
  	double vx; // speed of the motors
    double vy;
    
   double originCol=0;
   double originRow=0;
   double r=0;
   
int accOrigin[600][600]={0};//[row][col]
    std::vector<Point*> pastSun; 

   
	if (loadScenario(0)!= 0){
		std::cout<<" Error configuring AVC server"<<std::endl;
		return -1;
	}
	
    drawAll(0);
    sf::Clock TTimer;
    long int t = 0; // current time
    const sf::Time TTime   = sf::seconds(0.5f);
    
    //OpenPPMFile("cameraView.ppm", output);

    
    while ((globalWindow.isOpen())&&(t<1000))
    {
		
        // Process events
        sf::Event event;
        while (globalWindow.pollEvent(event)) {
            // Close window: exit
            if (event.type == sf::Event::Closed){
                     globalWindow.close();
			}
        }
        if (TTimer.getElapsedTime() > TTime) {
           TTimer.restart(); //0.1 secs later ...
           if (DEBUG_OUT) { std::cout<<"t="<<t<<std::endl;}
           drawAll(t++);
	   }
	   	
		
    std::vector<Point*> points; 
	for(int col=0; col<cameraView.width; col++){//for each point: if it is red and an edge: add to vector points
		for(int row=0; row<cameraView.height; row++){
			if((row<1 )| (col<1 )| (row>cameraView.height-2) | (col>cameraView.width-2)){continue;};
			  if(!isRed(row, col)){
				  continue;}
			  float vert = vertical(row,col, cameraView);
			  float hori = horizintal(row,col, cameraView);
			  if(sqrt(vert*vert + hori*hori)>50){
				 	 Point* pnt = new Point;
				 	 pnt->row=row;
				 	 pnt->col=col;
				 	 points.push_back(pnt);
				 	 //if(points.size()>15){
					//	 points.erase(points.begin());
					 //}
				  }
			
			}
		}//for each pixel

		long averageRadius = 0.0;
		long averageCol = 0.0;
		long averageRow = 0.0;

		long numUsed=0;
		std::vector<Point*> checked;
		for(Point* pnt1 : points){//for each permutation of points:  use permutation to make a guess of center and use to increase average
		for(Point* pnt2 : points){
			if(pnt1==pnt2){continue;}
			if(std::count(checked.begin(), checked.end(), pnt2)){continue;}
		for(Point* pnt3 : points){
			if(pnt1==pnt3){continue;}
			if(pnt2==pnt3){continue;}
						if(std::count(checked.begin(), checked.end(), pnt3)){continue;}

			//averageRadius+=findRadius(pnt->row, pnt->col, sunRow, sunCol);
				Point* centerGuess = findCenter(pnt1->row,pnt1->col,pnt2->row,pnt2->col,pnt3->row,pnt3->col);

				if(!(isnan(centerGuess->row)) & !(isnan(centerGuess->col)) & !(isinf(centerGuess->row)) & !(isinf(centerGuess->col))){ //points do not form a circle
				averageRow+=centerGuess->row;
				averageCol+=centerGuess->col;
				numUsed++;		

				}
}//pnt3			
}//pnt2		
checked.push_back(pnt1);	
}//pnt1	
		

			if(points.size()>10){//if there are enough points: use averages to find sun position:  then use this to make votes for the center of rotation
			float centerCol = (averageCol*1.0)/(numUsed*1.0);
			float centerRow= (averageRow*1.0)/(numUsed*1.0);

		
std::vector<Point*> checked;


	if(pastSun.size()>5){pastSun.erase(pastSun.begin());}
		for(Point* pnt1 : pastSun){//for each permutation: find the center of rotation (using current sun as third point):
		for(Point* pnt2 : pastSun){
			if(pnt1==pnt2){continue;}
			if(std::count(checked.begin(), checked.end(), pnt2)){continue;}//check if second value has already been first value
			Point* center = findCenter(    pnt1->row,  pnt1->col,     pnt2->row, pnt2->col,    centerRow, centerCol);//uses 3 points on circumeference to find center of rotation
			
			if(center->row<cameraView.height    &    center->row>=0    &    center->col<cameraView.width    &    center->col>=0){//useful to remove error messages when set pixels  on lower part of circle (beyond image)
			accOrigin[(int)center->row][(int)center->col]++;

			//set_pixel(output,pnt1->centerRow,  pnt1->centerCol, 0,255,0);							
			//set_pixel(output,center->row,  center->col, 0,0,255);
			}
}
checked.push_back(pnt1);
}
	Point* currentValue = new Point;
	currentValue->col =centerCol;
	currentValue->row=centerRow;
pastSun.push_back(currentValue);


int max = 0;
for(int row=1; row<cameraView.height-1; row++){//find center of rotation ->  coord with most votes
	for(int col=1; col<cameraView.width-1; col++){
		if(accOrigin[row][col]>max){
			max=accOrigin[row][col];
			originCol=col;
			originRow=row;
			}
		}
}
//set_pixel(output,originRow,  originCol, 255,0,0);
			r = findRadius(centerRow, centerCol, originRow, originCol); //sets radius (does not take old radius into account);

}//if


/*
for(float testT=0; testT<125; testT+=1){//draws the current circle out on to image
	Point* hello = sunPosition(r, testT, originCol, originRow);
	//set_pixel(output, hello->row, hello->col, 0,0,255);
	}
*/

std::cout<<"r="<<r<<" originCol="<<originCol<<" originRow="<<originRow<<std::endl;//prints calculated values

/* does nothing for now */
double xPosition, yPosition;
getPanelPostion(xPosition, yPosition);
double velocityX = 0.0; //cols
double velocityY = 0.0; //rows
setMotorsSpeed(velocityX, velocityX);

/* set motor position to where equation expects for the given t */
Point* sunPos = sunPosition(r, t, originCol, originRow);
if(sunPos->row>=600){
	sunPos->col= sunRise(r,originCol, originRow, cameraView.height);
	sunPos->row= 600;
	}

aim.loc.x = sunPos->col;
	aim.loc.y = sunPos->row;
	//SavePPMFile("gradients.ppm", output);
    }
    
    return EXIT_SUCCESS;
}
