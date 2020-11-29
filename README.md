# IMA-estimation
imu mounting angle estimation


#Introduction
This MATLAB code package implements a Kalman filter to estimate the mounting angles of a IMU with respect to the host vehicle, as described in the manuscript “Estimation of IMU Mounting Angles for Land Vehicular GNSS/INS Integrated System” by Qijin Chen et al. It is aimed at helping the readers to understand the manuscript and implement the algorithm. This self-contained package includes all required files and data. 
How to use this package 
1）	Copy this package including all subdirectories and files to your computer. 
2）	Set the directory as the current path of the MATLAB.
3）	Run ‘main.m’ to start the data processing. 

Then try to change the data files (cfg.fins) or the segment of trajectory used for mounting angle estimation. 
#Data format definition. 
Table 1 GNSS/INS smoothing result format
column	quantity	unit	
1	Time	s	double
2	position	latitude	deg	double
3		longitude	deg	double
4		height	m	double
5	Horizontal 
position	east	m	double
6		north	m	double
7	velocity	north	m/s	double
8		east	m/s	double
9		downward	m/s	double
10	attitude	roll	deg	double
11		pitch	deg	double
12		heading	deg	double
13	position standard
 deviation (STD)	north	m	double
14		east	m	double
15		height	m	double
16	velocity standard
 deviation (STD)	north	m/s	double
17		east	m/s	double
18		downward	m/s	double
19	attitude standard
 deviation (STD)	roll	deg	double
20		pitch	deg	double
21		heading	deg	double


#Table 2 DR input data format ( i.e., data_ains matrix)
column	quantity 	unit 	
1	Time	s	double
2	position 	latitude	rad	double
3		longitude	rad	double
4		height	m	double
5	Horizontal 
position	east	m	double
6		north	m	double
7	velocity	north	m/s	double
8		east	m/s	double
9		downward	m/s	double
10	attitude	roll	deg	double
11		pitch	deg	double
12		heading	deg	double
13	position standard
 deviation (STD) 	north	m	double
14		east	m	double
15		height	m	double
16	velocity standard
 deviation (STD) 	north	m/s	double
17		east	m/s	double
18		downward	m/s	double
19	attitude standard
 deviation (STD) 	roll	deg	double
20		pitch	deg	double
21		heading	deg	double
22	distance	m	double
