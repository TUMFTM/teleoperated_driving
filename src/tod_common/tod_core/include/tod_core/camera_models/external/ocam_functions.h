/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/
#ifndef TOD_HELPER__CAMERA_MODELS__EXTERNAL__OCAM_FUNCTIONS_H_
#define TOD_HELPER__CAMERA_MODELS__EXTERNAL__OCAM_FUNCTIONS_H_

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
// #include <cv.h>
// #include <highgui.h>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64
struct ocam_model
{
  double pol[MAX_POL_LENGTH];  // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ...
                               // + x^(N-1)*pol[N-1]
  int length_pol;              // length of polynomial
  double invpol[MAX_POL_LENGTH];  // the coefficients of the inverse polynomial
  int length_invpol;              // length of inverse polynomial
  double xc;                      // row coordinate of the center
  double yc;                      // column coordinate of the center
  double c;                       // affine parameter
  double d;                       // affine parameter
  double e;                       // affine parameter
  int width;                      // image width
  int height;                     // image height
};
/*------------------------------------------------------------------------------
 This function reads the parameters of the omnidirectional camera model from
 a given TXT file
------------------------------------------------------------------------------*/
// int get_ocam_model(struct ocam_model *myocam_model, char *filename);

/*------------------------------------------------------------------------------
 WORLD2CAM projects a 3D point on to the image
    WORLD2CAM(POINT2D, POINT3D, OCAM_MODEL)
    projects a 3D point (point3D) on to the image and returns the pixel coordinates (point2D).

    POINT3D = [X;Y;Z] are the coordinates of the 3D point.
    OCAM_MODEL is the model of the calibrated camera.
    POINT2D = [rows;cols] are the pixel coordinates of the reprojected point

    Copyright (C) 2009 DAVIDE SCARAMUZZA
    Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

    NOTE: the coordinates of "point2D" and "center" are already according to the C
    convention, that is, start from 0 instead than from 1.
------------------------------------------------------------------------------*/
// void world2cam(double point2D[2], double point3D[3], const ocam_model *myocam_model);

/*------------------------------------------------------------------------------
 CAM2WORLD projects a 2D point onto the unit sphere
    CAM2WORLD(POINT3D, POINT2D, OCAM_MODEL)
    back-projects a 2D point (point2D), in pixels coordinates,
    onto the unit sphere returns the normalized coordinates point3D = [x;y;z]
    where (x^2 + y^2 + z^2) = 1.

    POINT3D = [X;Y;Z] are the coordinates of the 3D points, such that (x^2 + y^2 + z^2) = 1.
    OCAM_MODEL is the model of the calibrated camera.
    POINT2D = [rows;cols] are the pixel coordinates of the point in pixels

    Copyright (C) 2009 DAVIDE SCARAMUZZA
    Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

    NOTE: the coordinates of "point2D" and "center" are already according to the C
    convention, that is, start from 0 instead than from 1.
------------------------------------------------------------------------------*/
// void cam2world(double point3D[3], double point2D[2], const struct ocam_model *myocam_model);
/*------------------------------------------------------------------------------
 Create Look Up Table for undistorting the image into a perspective image
 It assumes the the final image plane is perpendicular to the camera axis
------------------------------------------------------------------------------*/
// void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model,
// float sf);
//
// /*------------------------------------------------------------------------------
//  Create Look Up Table for undistorting the image into a panoramic image
//  It computes a trasformation from cartesian to polar coordinates
//  Therefore it does not need the calibration parameters
//  The region to undistorted in contained between Rmin and Rmax
//  xc, yc are the row and column coordinates of the image center
// ------------------------------------------------------------------------------*/
// void create_panoramic_undistortion_LUT ( CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float
// xc, float yc );
#endif  // TOD_HELPER__CAMERA_MODELS__EXTERNAL__OCAM_FUNCTIONS_H_
