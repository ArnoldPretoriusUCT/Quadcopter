#include "motioncapture.h"

mat motionCapture::C1,
    motionCapture::C2,
    motionCapture::C3,
    motionCapture::C4,
    motionCapture::C5;
bool motionCapture::isFullyObservable,
     motionCapture::newData[5],
     motionCapture::ekfType,
    motionCapture::isCritical;
vec motionCapture::fValid,
        motionCapture::camValidMean,
        motionCapture::camValidCount,
        motionCapture::xyzValidMean,
        motionCapture::valid;

double motionCapture::xyzValidCount;

vec motionCapture::fDet,
    motionCapture::fDetMean,
    motionCapture::sig,
    motionCapture::dT,
    motionCapture::dTMean,
    motionCapture::meanCount,
    motionCapture::receiveTime;

cube motionCapture::Bp,
     motionCapture::fSizes;

double motionCapture::startTime;

quint8 motionCapture::watchDogCount;

void motionCapture::initialise()
{
    int index = 0;
    double col_buffer[12];
    std::ifstream infile("/home/arnold/Academia/Matlab/MoCap/cameraCalibration/cameraExtrinsics.txt");
    std::string line;
    while( std::getline(infile,line) )
    {
        std::istringstream iss(line);
        if( !(iss >> col_buffer[0] >> col_buffer[1] >> col_buffer[2] >> col_buffer[3] >> col_buffer[4] >> col_buffer[5] >> col_buffer[6] >> col_buffer[7] >> col_buffer[8] >> col_buffer[9] >> col_buffer[10] >> col_buffer[11]) ) {break;}

        if(index == 0)
        {
            C1 << col_buffer[3] << col_buffer[4]   << col_buffer[5]    << col_buffer[0] << endr
                  << col_buffer[6] << col_buffer[7]   << col_buffer[8]    << col_buffer[1] << endr
                  << col_buffer[9] << col_buffer[10] << col_buffer[11] << col_buffer[2] << endr;
        }
        if(index == 1)
        {
            C2 << col_buffer[3] << col_buffer[4]   << col_buffer[5]    << col_buffer[0] << endr
                  << col_buffer[6] << col_buffer[7]   << col_buffer[8]    << col_buffer[1] << endr
                  << col_buffer[9] << col_buffer[10] << col_buffer[11] << col_buffer[2] << endr;
        }
        if(index == 2)
        {
            C3 << col_buffer[3] << col_buffer[4]   << col_buffer[5]    << col_buffer[0] << endr
                  << col_buffer[6] << col_buffer[7]   << col_buffer[8]    << col_buffer[1] << endr
                  << col_buffer[9] << col_buffer[10] << col_buffer[11] << col_buffer[2] << endr;
        }
        if(index == 3)
        {
            C4 << col_buffer[3] << col_buffer[4]   << col_buffer[5]    << col_buffer[0] << endr
                  << col_buffer[6] << col_buffer[7]   << col_buffer[8]    << col_buffer[1] << endr
                  << col_buffer[9] << col_buffer[10] << col_buffer[11] << col_buffer[2] << endr;
        }
        if(index == 4)
        {
            C5 << col_buffer[3] << col_buffer[4]   << col_buffer[5]    << col_buffer[0] << endr
                  << col_buffer[6] << col_buffer[7]   << col_buffer[8]    << col_buffer[1] << endr
                  << col_buffer[9] << col_buffer[10] << col_buffer[11] << col_buffer[2] << endr;
        }
        index++;

        isFullyObservable = false;
    }
    //Intrinsic matrices
    K1 << 0.004115540824139 << 0                                          << -0.679832748295188   << endr
          << 0                                       << 0.004118134030176    << -0.422838437911775   << endr;
    K2 << 0.004155833473703 << 0                                          << -0.676116065144745   << endr
          << 0                                       << 0.004128675777045    << -0.441725669629873   << endr;
    K3 << 0.004051685480399 << 0                                          << -0.631940624356402   << endr
          << 0                                       << 0.004054994950924    << -0.317429772411336   << endr;
    K4 << 0.003878007903993 << 0                                          << -0.694255149688533   << endr
          << 0                                       << 0.003904400833442    << -0.339977857465212   << endr;
    K5 << 0.004170188845405 << 0                                          << -0.686164081538033   << endr
          << 0                                       << 0.004180554819812    << -0.391573957871167   << endr;
    //Distortion vectors;
    kc1 << -0.431719851577236 << 0.185152238519111 << -0.000684598373473 <<  0.000655457763936 << 0 << endr;
    kc2 << -0.436232987214853 << 0.183844583936495 << -0.000591530642416 <<  0.005268659824668 << 0 << endr;
    kc3 << -0.436596262463136 << 0.181503225300774 <<  0.000266381673188 <<  0.001313996506580 << 0 << endr;
    kc4 << -0.427405566633378 << 0.146812469380385 <<  0.015246267345832 << -0.004593206171521 << 0 << endr;
    kc5 << -0.461494419222913 << 0.282080832325038 << -0.003131642443852 <<  0.000923733172911 << 0 << endr;

    B1.zeros(2,3);
    B2.zeros(2,3);
    B3.zeros(2,3);
    B4.zeros(2,3);
    B5.zeros(2,3);

    dataLogger::initialise();
    cameraEKF::initialise();
    ekfSingleBlob::initialise();

    attitudeControl::initialise();
    initialisePixies();
    initialiseTimers(CAMERAEKF_DT*1e3);


    startTime = elapsedTimer.nsecsElapsed()*1e-6;
    fValid.zeros(5);
    valid.zeros(3);
    isCritical = false;
    dTMean.zeros(5);
    meanCount.ones(5);
    fDetMean.zeros(5);
    camValidMean.zeros(5);
    camValidCount.zeros(5);
    xyzValidMean.zeros(3);
    xyzValidCount = 0;
}
//(5x1) signature vector
//(5x1) features detected vector
//(2x3x5) pixel-position cube
//(2x3x5) pixel-size cube
void motionCapture::getData(vec sig,vec fDet,cube Bp,cube fSizes)
{
    uvec index = sort_index(sig);
    B1 = Bp.slice(index(0)); //(2x3)
    B2 = Bp.slice(index(1));
    B3 = Bp.slice(index(2));
    B4 = Bp.slice(index(3));
    B5 = Bp.slice(index(4));

    fSizes1 = fSizes.slice(index(0)); //(2x3)
    fSizes2 = fSizes.slice(index(1));
    fSizes3 = fSizes.slice(index(2));
    fSizes4 = fSizes.slice(index(3));
    fSizes5 = fSizes.slice(index(4));

    signature = sig;
    fDetected.zeros(5);
    for(int i=0;i<5;i++)
    {
        fDetected(i) = fDet(index(i));
    }
}

void motionCapture::backProjectPoints(vec b1,vec b2,vec b3)
{
    vec b1x,b2x,b3x;

    //Camera1
    b1x = C1.cols(0,2)*b1 + C1.col(3);
    b1x = b1x/b1x(2);
    b2x = C1.cols(0,2)*b2 + C1.col(3);
    b2x = b2x/b2x(2);
    b3x = C1.cols(0,2)*b3 + C1.col(3);
    b3x = b3x/b3x(2);
    B1x = join_rows(b1x,b2x);
    B1x = join_rows(B1x,b3x);
    //Camera2
    b1x = C2.cols(0,2)*b1 + C2.col(3);
    b1x = b1x/b1x(2);
    b2x = C2.cols(0,2)*b2 + C2.col(3);
    b2x = b2x/b2x(2);
    b3x = C2.cols(0,2)*b3 + C2.col(3);
    b3x = b3x/b3x(2);
    B2x = join_rows(b1x,b2x);
    B2x = join_rows(B2x,b3x);
    //Camera3
    b1x = C3.cols(0,2)*b1 + C3.col(3);
    b1x = b1x/b1x(2);
    b2x = C3.cols(0,2)*b2 + C3.col(3);
    b2x = b2x/b2x(2);
    b3x = C3.cols(0,2)*b3 + C3.col(3);
    b3x = b3x/b3x(2);
    B3x = join_rows(b1x,b2x);
    B3x = join_rows(B3x,b3x);
    //Camera4
    b1x = C4.cols(0,2)*b1 + C4.col(3);
    b1x = b1x/b1x(2);
    b2x = C4.cols(0,2)*b2 + C4.col(3);
    b2x = b2x/b2x(2);
    b3x = C4.cols(0,2)*b3 + C4.col(3);
    b3x = b3x/b3x(2);
    B4x = join_rows(b1x,b2x);
    B4x = join_rows(B4x,b3x);
    //Camera5
    b1x = C5.cols(0,2)*b1 + C5.col(3);
    b1x = b1x/b1x(2);
    b2x = C5.cols(0,2)*b2 + C5.col(3);
    b2x = b2x/b2x(2);
    b3x = C5.cols(0,2)*b3 + C5.col(3);
    b3x = b3x/b3x(2);
    B5x = join_rows(b1x,b2x);
    B5x = join_rows(B5x,b3x);

    B1x = B1x.rows(0,1);
    B2x = B2x.rows(0,1);
    B3x = B3x.rows(0,1);
    B4x = B4x.rows(0,1);
    B5x = B5x.rows(0,1);
}

vec motionCapture::matchFeatures(mat B,mat Bx,double fDetected,mat fSizes)
{
    mat distMat(3,3);
    vec dist,m;
    rowvec rowI;
    uword rowIndex;
    uvec minIndex;
    double minDistance,fHeight1,fHeight2,fWidth1,fWidth2,fArea1,fArea2;
    distMat.zeros(3,3);
    m.zeros(3);

    //normalised constants
    double BLOB_DELTA_X = 3.0/320;
    double BLOB_DELTA_Y = 3.0/200;
    double MIN_DISTANCE_THRESHOLD = 5.0/200;

    if( fDetected == 3 )
    {
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                dist = B.col(i) - Bx.col(j);
                distMat(i,j) = norm(dist);
            }
            rowI = distMat.row(i);
            minDistance = rowI.min();
            if( minDistance < MIN_DISTANCE_THRESHOLD )
            {
                minIndex = find(rowI == minDistance);
                m(i) = minIndex(0)+1;
            }
        }
    }
    else if( fDetected == 2 )
    {
        fHeight1 = fSizes(0,0);
        fHeight2 = fSizes(0,1);
        fWidth1  = fSizes(1,0);
        fWidth2  = fSizes(1,1);
        fArea1 = fHeight1*fWidth1;
        fArea2 = fHeight2*fWidth2;

        if( (fHeight1 > fWidth1+BLOB_DELTA_X) || (fWidth1 > fHeight1+BLOB_DELTA_Y) || (fHeight2 > fWidth2+BLOB_DELTA_X) || (fWidth2 > fHeight2+BLOB_DELTA_Y) )
        {
            if( fArea2 > fArea1 )
            {
                rowIndex = 0;
            }
            else
            {
                rowIndex = 1;
            }
            for(int j=0;j<3;j++)
            {
                dist = B.col(rowIndex)-Bx.col(j);
                distMat(rowIndex,j) = norm(dist);
            }
            rowI = distMat.row(rowIndex);
            minDistance = rowI.min();
            if( minDistance < MIN_DISTANCE_THRESHOLD )
            {
                minIndex = find(rowI == minDistance);
                m(rowIndex) = minIndex(0)+1;
            }
        }
        else
        {
            for(int i=0;i<2;i++)
            {
                for(int j=0;j<3;j++)
                {
                    dist = B.col(i) - Bx.col(j);
                    distMat(i,j) = norm(dist);
                }
                rowI = distMat.row(i);
                minDistance = rowI.min();
                if( minDistance < MIN_DISTANCE_THRESHOLD )
                {
                    minIndex = find(rowI == minDistance);
                    m(i) = minIndex(0)+1;
                }
            }
        }
    }
    else if( fDetected == 1 )
    {
        fHeight1 = fSizes(0,0);
        fWidth1 = fSizes(1,0);
//        if( (fHeight1 >  2*fWidth1) || (fWidth1 > 2*fHeight1) )
        if( (fHeight1 >  fWidth1 + BLOB_DELTA_X) || (fWidth1 > fHeight1 + BLOB_DELTA_Y) )
        {
            //do nothing
        }
        else
        {
            for(int j=0;j<3;j++)
            {
                dist = B.col(0) - Bx.col(j);
                distMat(0,j) = norm(dist);
            }
            rowI = distMat.row(0);
            minDistance = rowI.min();
            if( minDistance < MIN_DISTANCE_THRESHOLD )
            {
                minIndex = find(rowI == minDistance);
                m(0) = minIndex(0)+1;
            }
        }
    }
    for(int i=0;i<3;i++)
    {
        double mCheckSum = sum( m == i+1 );
        if( mCheckSum > 1 )
        {
            m.zeros(3);
            break;
        }
    }
//    cout << distMat << endl;
    return m;
}

//normalisation, followed by undistortion
// pc = K * pp
//B1 (2x3)
void motionCapture::normalizeFeatures()
{
    //normalise
    rowvec ones(3);
    ones << 1.0 << 1.0 << 1.0 << endr;

    B1 = K1*join_cols(B1,ones);
    B2 = K2*join_cols(B2,ones);
    B3 = K3*join_cols(B3,ones);
    B4 = K4*join_cols(B4,ones);
    B5 = K5*join_cols(B5,ones);
    //undistort
    B1 = undistort(B1,kc1);
    B2 = undistort(B2,kc2);
    B3 = undistort(B3,kc3);
    B4 = undistort(B4,kc4);
    B5 = undistort(B5,kc5);
}

mat motionCapture::undistort(mat B,vec kc)
{
    vec delta_x(2,1),x(2,1),ones(2,1);
    ones << 1 << 1 << endr;

    double k1 = kc(0);
    double k2 = kc(1);
    double k3 = kc(4);
    double p1 = kc(2);
    double p2 = kc(3);

    for(int i=0;i<3;i++)
    {
        x = B.col(i);
        for(int k=0;k<20;k++)
        {
            double r_2 = sum(x%x);\
            double k_radial =  1 + k1 * r_2 + k2 * r_2*r_2 + k3 * r_2*r_2*r_2;
            delta_x(0) = 2*p1*x(0)*x(1) + p2*( r_2 + 2*x(0)*x(0) ) ;
            delta_x(1) = p1 * (r_2 + 2*x(1)*x(1))+2*p2*x(0)*x(1);
            x = (B.col(i) - delta_x)/(ones*k_radial);
        }
        B.col(i) = x;
    }
    return B;
}

vec motionCapture::orderFeatures(mat B,vec m)
{
    mat B_temp(2,3);
    B_temp.fill(10); //invalid features will be outside unity norm

    uvec index1 = find(m==1);
    uvec index2 = find(m==2);
    uvec index3 = find(m==3);

    if( (index1.is_empty() == false) && (index1.size() == 1) )
    {
        B_temp.col(0) = B.col(index1(0));
    }
    if( (index2.is_empty() == false) && (index2.size() == 1) )
    {
        B_temp.col(1) = B.col(index2(0));
    }
    if( (index3.is_empty() == false) && (index3.size() == 1) )
    {
        B_temp.col(2) = B.col(index3(0));
    }

    vec p(6);
    p.rows(0,1) = B_temp.col(0);
    p.rows(2,3) = B_temp.col(1);
    p.rows(4,5) = B_temp.col(2);
    return p;
}

void motionCapture::validFeatures(vec m1, vec m2, vec m3, vec m4, vec m5)
{
    vec M(15);
    M.rows(0,2)   = m1;
    M.rows(3,5)   = m2;
    M.rows(6,8)   = m3;
    M.rows(9,11)  = m4;
    M.rows(12,14) = m5;

    //per camera number of valid features
    fValid.zeros(5);
    for(int i=0;i<3;i++)
    {
        if( (m1(i) != 0) && newData[0] )
        {
            fValid(0)++;
        }
        if( (m2(i) != 0) && newData[1] )
        {
            fValid(1)++;
        }
        if( (m3(i) != 0) && newData[2] )
        {
            fValid(2)++;
        }
        if( (m4(i) != 0) && newData[3] )
        {
            fValid(3)++;
        }
        if( (m5(i) != 0) && newData[4] )
        {
            fValid(4)++;
        }
    }
    for(int i=0;i<5;i++)
    {
        if( newData[i] )
        {
            camValidMean(i) = ((camValidCount(i)-1)*camValidMean(i)+fValid(i))/camValidCount(i);
            camValidCount(i)++;
        }
    }

    //critical failure check
    uvec i1 = find( M == 1 );
    uvec i2 = find( M == 2 );
    uvec i3 = find( M == 3 );
    valid(0) +=i1.n_rows;
    valid(1) +=i2.n_rows;
    valid(2) +=i3.n_rows;

    if( newData[0] || newData[1] || newData[2] || newData[3] || newData[4] )
    {
        xyzValidCount += (quint8)newData[0] + (quint8)newData[1] + (quint8)newData[2] + (quint8)newData[3] + (quint8)newData[4];
        xyzValidMean = valid/xyzValidCount;
    }
}


void motionCapture::correspondenceMatching(vec pos,vec quat)
{
    featurePositionEstimates(pos,quat);
    backProjectPoints(b1,b2,b3); //generates B1x,B2x,B3x,B4x,B5x

    normalizeFeatures();

    p1 = p2 = p3 = p4 = p5 = 10*ones(6,1); //initialised as invalid feature
    m1 = m2 = m3 = m4 = m5 = zeros(3);
    if(newData[0])
    {
        m1 = matchFeatures(B1,B1x,fDetected(0),fSizes1);
        p1 = orderFeatures(B1,m1);
    }
    if(newData[1])
    {
        m2 = matchFeatures(B2,B2x,fDetected(1),fSizes2);
        p2 = orderFeatures(B2,m2);
    }
    if(newData[2])
    {
        m3 = matchFeatures(B3,B3x,fDetected(2),fSizes3);
        p3 = orderFeatures(B3,m3);
    }
    if(newData[3])
    {
        m4 = matchFeatures(B4,B4x,fDetected(3),fSizes4);
        p4 = orderFeatures(B4,m4);
    }
    if(newData[4])
    {
        m5 = matchFeatures(B5,B5x,fDetected(4),fSizes5);
        p5 = orderFeatures(B5,m5);
    }

    validFeatures(m1,m2,m3,m4,m5);
}

void motionCapture::featurePositionEstimates(vec pos, vec quat)
{
    vec bb1(3),bb2(3),bb3(3);
    bb1 << 0.0  <<  B1_Y << 0.0 << endr;
    bb2 << 0.0  <<  B2_Y << 0.0 << endr;
    bb3 << B3_X <<  0.0  << 0.0 << endr;

    mat R(3,3);
    R = quat2rotationMatrix(quat);

    b1 = pos + R*bb1;
    b2 = pos + R*bb2;
    b3 = pos + R*bb3;
}

void motionCapture::initialisePixies()
{
    pixy[0].initialise(SERIAL_PIXY_COM_PORT0, SERIAL_PIXY_BAUD_RATE);//
    pixy[1].initialise(SERIAL_PIXY_COM_PORT1, SERIAL_PIXY_BAUD_RATE);
    pixy[2].initialise(SERIAL_PIXY_COM_PORT2, SERIAL_PIXY_BAUD_RATE);
    pixy[3].initialise(SERIAL_PIXY_COM_PORT3, SERIAL_PIXY_BAUD_RATE);
    pixy[4].initialise(SERIAL_PIXY_COM_PORT4, SERIAL_PIXY_BAUD_RATE);
}

void motionCapture::initialiseTimers(int t_msec)
{
    QTimer *runTimer = new QTimer(this);
    connect( runTimer, SIGNAL(timeout()),this,SLOT(updateData()) );
    runTimer->setTimerType(Qt::PreciseTimer);
    runTimer->start(t_msec);
}

void motionCapture::updateData()
{
    double k = 1e5/5920*260/287*260/270*260*1.025;
    stateMachine::thrust = k*(imu::gyro[0]-stateMachine::thrust_0);

    sig << pixy[0].blob[0].signature
           << pixy[1].blob[0].signature
           << pixy[2].blob[0].signature
           << pixy[3].blob[0].signature 
           << pixy[4].blob[0].signature
           << endr;
    fDet << pixy[0].objectsDetected
             << pixy[1].objectsDetected
             << pixy[2].objectsDetected
             << pixy[3].objectsDetected
             << pixy[4].objectsDetected
             << endr;
    dT << pixy[0].dT
          << pixy[1].dT
          << pixy[2].dT
          << pixy[3].dT
          << pixy[4].dT
          << endr;
   for(int i=0;i<5;i++)
   {
       dTMean(i) = ((meanCount(i)-1)*dTMean(i)+dT(i))/meanCount(i);
       fDetMean(i) = ((meanCount(i)-1)*fDetMean(i)+fDet(i))/meanCount(i);
       if( pixy[i].newData )
       {
           meanCount(i)++;
       }
   }
//    qDebug() << (int)dT(0) << (int)dT(1) << (int)dT(2) << (int)dT(3) << (int)dT(4);

    receiveTime << pixy[0].receiveTime
                            << pixy[1].receiveTime
                            << pixy[2].receiveTime
                            << pixy[3].receiveTime
                            << pixy[4].receiveTime
                            << endr;

    newData[0] = pixy[0].newData; newData[1] = pixy[1].newData; newData[2] = pixy[2].newData; newData[3] = pixy[3].newData; newData[4] = pixy[4].newData;
    pixy[0].newData = pixy[1].newData = pixy[2].newData = pixy[3].newData = pixy[4].newData = imu::newData = 0;


    Bp.zeros(2,4,5);
    Bp.slice(0) << pixy[0].blob[0].pixelPosition[0] << pixy[0].blob[1].pixelPosition[0] << pixy[0].blob[2].pixelPosition[0] << pixy[0].blob[3].pixelPosition[0] << endr
                << pixy[0].blob[0].pixelPosition[1] << pixy[0].blob[1].pixelPosition[1] << pixy[0].blob[2].pixelPosition[1] << pixy[0].blob[3].pixelPosition[1] << endr;
    Bp.slice(1) << pixy[1].blob[0].pixelPosition[0] << pixy[1].blob[1].pixelPosition[0] << pixy[1].blob[2].pixelPosition[0] << pixy[1].blob[3].pixelPosition[0] << endr
                << pixy[1].blob[0].pixelPosition[1] << pixy[1].blob[1].pixelPosition[1] << pixy[1].blob[2].pixelPosition[1] << pixy[1].blob[3].pixelPosition[1] << endr;
    Bp.slice(2) << pixy[2].blob[0].pixelPosition[0] << pixy[2].blob[1].pixelPosition[0] << pixy[2].blob[2].pixelPosition[0] << pixy[2].blob[3].pixelPosition[0] << endr
                << pixy[2].blob[0].pixelPosition[1] << pixy[2].blob[1].pixelPosition[1] << pixy[2].blob[2].pixelPosition[1] << pixy[2].blob[3].pixelPosition[1] << endr;
    Bp.slice(3) << pixy[3].blob[0].pixelPosition[0] << pixy[3].blob[1].pixelPosition[0] << pixy[3].blob[2].pixelPosition[0] << pixy[3].blob[3].pixelPosition[0] << endr
                << pixy[3].blob[0].pixelPosition[1] << pixy[3].blob[1].pixelPosition[1] << pixy[3].blob[2].pixelPosition[1] << pixy[3].blob[3].pixelPosition[1] << endr;
    Bp.slice(4) << pixy[4].blob[0].pixelPosition[0] << pixy[4].blob[1].pixelPosition[0] << pixy[4].blob[2].pixelPosition[0] << pixy[4].blob[3].pixelPosition[0] << endr
                << pixy[4].blob[0].pixelPosition[1] << pixy[4].blob[1].pixelPosition[1] << pixy[4].blob[2].pixelPosition[1] << pixy[4].blob[3].pixelPosition[1] << endr;
    fSizes.zeros(2,4,5);
    fSizes.slice(0) << pixy[0].blob[0].pixelHeight << pixy[0].blob[1].pixelHeight << pixy[0].blob[2].pixelHeight << pixy[0].blob[3].pixelHeight << endr
                    << pixy[0].blob[0].pixelWidth  << pixy[0].blob[1].pixelWidth  << pixy[0].blob[2].pixelWidth  << pixy[0].blob[3].pixelWidth  << endr;
    fSizes.slice(1) << pixy[1].blob[0].pixelHeight << pixy[1].blob[1].pixelHeight << pixy[1].blob[2].pixelHeight << pixy[1].blob[3].pixelHeight << endr
                    << pixy[1].blob[0].pixelWidth  << pixy[1].blob[1].pixelWidth  << pixy[1].blob[2].pixelWidth  << pixy[1].blob[3].pixelWidth  << endr;
    fSizes.slice(2) << pixy[2].blob[0].pixelHeight << pixy[2].blob[1].pixelHeight << pixy[2].blob[2].pixelHeight << pixy[2].blob[3].pixelHeight << endr
                    << pixy[2].blob[0].pixelWidth  << pixy[2].blob[1].pixelWidth  << pixy[2].blob[2].pixelWidth  << pixy[2].blob[3].pixelWidth  << endr;
    fSizes.slice(3) << pixy[3].blob[0].pixelHeight << pixy[3].blob[1].pixelHeight << pixy[3].blob[2].pixelHeight << pixy[3].blob[3].pixelHeight << endr
                    << pixy[3].blob[0].pixelWidth  << pixy[3].blob[1].pixelWidth  << pixy[3].blob[2].pixelWidth  << pixy[3].blob[3].pixelWidth  << endr;
    fSizes.slice(4) << pixy[4].blob[0].pixelHeight << pixy[4].blob[1].pixelHeight << pixy[4].blob[2].pixelHeight << pixy[4].blob[3].pixelHeight << endr
                    << pixy[4].blob[0].pixelWidth  << pixy[4].blob[1].pixelWidth  << pixy[4].blob[2].pixelWidth  << pixy[4].blob[3].pixelWidth  << endr;

    cube Bp3,fSizes3;
    Bp3.zeros(2,3,5);
    Bp3.slice(0) << pixy[0].blob[0].pixelPosition[0] << pixy[0].blob[1].pixelPosition[0] << pixy[0].blob[2].pixelPosition[0] << endr
                 << pixy[0].blob[0].pixelPosition[1] << pixy[0].blob[1].pixelPosition[1] << pixy[0].blob[2].pixelPosition[1] << endr;
    Bp3.slice(1) << pixy[1].blob[0].pixelPosition[0] << pixy[1].blob[1].pixelPosition[0] << pixy[1].blob[2].pixelPosition[0] << endr
                 << pixy[1].blob[0].pixelPosition[1] << pixy[1].blob[1].pixelPosition[1] << pixy[1].blob[2].pixelPosition[1] << endr;
    Bp3.slice(2) << pixy[2].blob[0].pixelPosition[0] << pixy[2].blob[1].pixelPosition[0] << pixy[2].blob[2].pixelPosition[0] << endr
                 << pixy[2].blob[0].pixelPosition[1] << pixy[2].blob[1].pixelPosition[1] << pixy[2].blob[2].pixelPosition[1] << endr;
    Bp3.slice(3) << pixy[3].blob[0].pixelPosition[0] << pixy[3].blob[1].pixelPosition[0] << pixy[3].blob[2].pixelPosition[0] << endr
                 << pixy[3].blob[0].pixelPosition[1] << pixy[3].blob[1].pixelPosition[1] << pixy[3].blob[2].pixelPosition[1] << endr;
    Bp3.slice(4) << pixy[4].blob[0].pixelPosition[0] << pixy[4].blob[1].pixelPosition[0] << pixy[4].blob[2].pixelPosition[0] << endr
                 << pixy[4].blob[0].pixelPosition[1] << pixy[4].blob[1].pixelPosition[1] << pixy[4].blob[2].pixelPosition[1] << endr;
    fSizes3.zeros(2,3,5);
    fSizes3.slice(0) << pixy[0].blob[0].pixelHeight << pixy[0].blob[1].pixelHeight << pixy[0].blob[2].pixelHeight << endr
                     << pixy[0].blob[0].pixelWidth  << pixy[0].blob[1].pixelWidth  << pixy[0].blob[2].pixelWidth  << endr;
    fSizes3.slice(1) << pixy[1].blob[0].pixelHeight << pixy[1].blob[1].pixelHeight << pixy[1].blob[2].pixelHeight << endr
                     << pixy[1].blob[0].pixelWidth  << pixy[1].blob[1].pixelWidth  << pixy[1].blob[2].pixelWidth  << endr;
    fSizes3.slice(2) << pixy[2].blob[0].pixelHeight << pixy[2].blob[1].pixelHeight << pixy[2].blob[2].pixelHeight << endr
                     << pixy[2].blob[0].pixelWidth  << pixy[2].blob[1].pixelWidth  << pixy[2].blob[2].pixelWidth  << endr;
    fSizes3.slice(3) << pixy[3].blob[0].pixelHeight << pixy[3].blob[1].pixelHeight << pixy[3].blob[2].pixelHeight << endr
                     << pixy[3].blob[0].pixelWidth  << pixy[3].blob[1].pixelWidth  << pixy[3].blob[2].pixelWidth  << endr;
    fSizes3.slice(4) << pixy[4].blob[0].pixelHeight << pixy[4].blob[1].pixelHeight << pixy[4].blob[2].pixelHeight << endr
                     << pixy[4].blob[0].pixelWidth  << pixy[4].blob[1].pixelWidth  << pixy[4].blob[2].pixelWidth  << endr;

    getData(sig,fDet,Bp3,fSizes3);
    correspondenceMatching(cameraEKF::x.rows(0,2),cameraEKF::x.rows(6,9));
    if( motionCapture::ekfType == EKF_MULTI_BLOB )
    {
        cameraEKF::iterate(p1,p2,p3,p4,p5);
    }
    else
    {
        ekfSingleBlob::iterate(B1.col(0),B2.col(0),B3.col(0),B4.col(0),B5.col(0));
    }
    attitudeControl::iterate();

    if( dataLogger::isLogging ){ dataLogger::logData(); }
}

void motionCapture::watchDog()
{
//    if( ((xyzValidMean(0) < .5) || (xyzValidMean(1) < .5) || (xyzValidMean(2) < .5))  && cameraEKF::isActive )
//    {
//        isCritical = true;
//    }
//    if(isCritical)
//    {
//        qDebug() << "EKF:Critical failure!" << xyzValidMean(0) << xyzValidMean(1) << xyzValidMean(2);
//    }
}
