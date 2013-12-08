#include "epuck.h"
#include <math.h>
#include <QList>
#include <QDebug>

/**
Copyright 2010 Manuel Martín Ortiz <mmartinortiz@gmail.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
MA 02110-1301, USA.
**/

Epuck::Epuck(const QString port)
{
    QByteArray ba = port.toLocal8Bit();
    connectionPort = ba.data();

    mStep = 0;

    mLogHeader = "[ePuck]";
    mLapseTime = 0;
    mIsConnected = false;
    mMotorPosition = QPoint(0,0);
    mMotorSpeed = QPoint(0,0);
    mNewSpeed = QPoint(0,0);
    mProxSensor = QVector<int>(8, 0);
    mMic = QVector<int>(3, 0);
    mFloor = QVector<int>(3, 0);
    mLeds = QVector<bool>(10, false);
    mLedsNew = QVector<bool>(10, false);
    mLightAvg = 0;
    mImage = QImage();
    mAbortThread = false;
    getSensorData = true;
    mSetMotorSpeed = false;
    mSetMotorPosition = false;
    mSetCameraParameters = false;
    mLedChanged = true;
    mStop = false;
    mReset = false;

    mSensorProxEnabled = false;
    mSensorAccEnabled = false;
    mSensorMicEnabled = false;
    mSensorMotorPosEnabled = false;
    mSensorMotorSpeedEnabled = false;
    mSensorSelectorEnabled = false;
    mSensorLightEnabled = false;
    mSensorFloorEnabled = false;

    mPrevLapseTime = mTime.elapsed();
    mMediumLapseTime = 0;

    setCameraParameters();
}

void Epuck::setPort(QString port)
{
    QByteArray ba = port.toLocal8Bit();
    connectionPort = ba.data();
}

/**
 * @brief
 *
 * @param port
 */
bool Epuck::initPort()
{
    int error = 0;

#ifdef __WIN32__
    mComm = new TCommPort();
    mComm->SetCommPort(connectionPort);
    mComm->SetBaudRate(115200);
    mComm->SetParity(NOPARITY);      // NOPARITY and friends are #defined in windows.h
    mComm->SetByteSize(8);
    mComm->SetStopBits(ONESTOPBIT);  // ONESTOPBIT is also from windows.h
    error = mComm->OpenCommPort();
#else
    mComm = new SerialComm();
    error = mComm->connect(connectionPort);
#endif

    if (error == -1)
    {
        qDebug() << mLogHeader << "Unable to open serial port" << "'" << connectionPort<< "'" << ". Error:" << error;
        mIsConnected = false;
        emit cannotOpenPort();
    }
    else
    {
        mIsConnected = true;
        emit portOpened();

        send("R\r", sizeof("R\r"));
        recv(mRxBuffer, sizeof(mRxBuffer));
        qDebug() << mLogHeader << mRxBuffer;

        while (!strstr(mRxBuffer,"v,"))
        {
            send("V\r", sizeof("V\r"));
            recv(mRxBuffer, sizeof(mRxBuffer));
            qDebug() << mLogHeader << mRxBuffer;
        }

    }

    return mIsConnected;
}

int Epuck::send(const char *msg, int size)
{
    int bytes = 0;

#ifdef __WIN32__
    mComm->PurgeCommPort();
    mComm->WriteBuffer((BYTE*)mCommand, size);
    mComm->FlushCommPort();
    Sleep(100);
#else
    bytes = mComm->writeData(msg, size, 1000000);
#endif

    return bytes;
}

int Epuck::recv(char *buffer, int size)
{
    int bytes;

#ifdef __WIN32__
    bytes = mComm->ReadBytes((BYTE*)buffer, size, 10000000);
#else
    bytes = mComm->readData(buffer, size, 1000000);
#endif

    return bytes;
}

void Epuck::run()
{
    if (!initPort())
        return;

    char command[20];
    int bytes;
    QList <char>sensorsToRead;

    while (! mAbortThread)
    {
        if (mSetMotorSpeed)
        {
            mSetMotorSpeed = false;
            lock.lockForRead();
            char high_left = (mNewSpeed.x() >> 8) & 0xFF;
            char low_left = mNewSpeed.x() & 0xFF;

            char high_right = (mNewSpeed.y() >> 8) & 0xFF;
            char low_right = mNewSpeed.y() & 0xFF;
            lock.unlock();

            memset(command, 0x0, sizeof(command));
            sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
            send(command, 8);
        }

        if (mSetMotorPosition)
        {
            mSetMotorPosition = false;
            lock.lockForRead();
            char high_left = (mMotorPosition.x() >> 8) & 0xFF;
            char low_left = mMotorPosition.x() & 0xFF;

            char high_right = (mMotorPosition.y() >> 8) & 0xFF;
            char low_right = mMotorPosition.y() & 0xFF;
            lock.unlock();

            memset(command, 0x0, sizeof(command));
            sprintf(command, "%c%c%c%c%c%c",-'P', low_left, high_left, low_right, high_right,0);
            send(command, 8);
        }

        if (mLedChanged)
        {
            mLedChanged = false;
            memset(command, 0x0, sizeof(command));
            for (int i = 0; i < mLeds.size(); i++)
                if (mLeds[i] != mLedsNew[i])
                {
                    mLeds[i] = mLedsNew[i];
                    sprintf(command, "%c%c%c%c",-'L', i, (int) mLedsNew[i], 0);
                    send(command, 4);
                }
        }

        if (mSetCameraParameters && mSensorCameraEnabled)
        {
            mSetCameraParameters = false;

            memset(command, 0x0, sizeof(command));
            sprintf(command, "J,%d,%d,%d,%d,\r", mType, mWidth, mHeight, mZoom);
            send(command, sizeof(command));

            bytes = recv(mRxBuffer, 3);
        }

        if (getSensorData)
        {
            sensorsToRead.clear();

            if (mSensorAccEnabled)
                sensorsToRead.append('A');

            if (mSensorMotorPosEnabled)
                sensorsToRead.append('Q');

            if (mSensorMotorSpeedEnabled)
                sensorsToRead.append('E');

            if (mSensorProxEnabled)
                sensorsToRead.append('N');

            if (mSensorLightEnabled)
                sensorsToRead.append('O');

            if (mSensorMicEnabled)
                sensorsToRead.append('u');

            if (mSensorFloorEnabled)
                sensorsToRead.append('M');

            if (mSensorCameraEnabled)
                sensorsToRead.append('I');

            if (sensorsToRead.size() > 0)
            {
                memset(command, 0x0, sizeof(command));
                for (int i = 0; i < sensorsToRead.size(); i++)
                    command[i] =- sensorsToRead[i];

                send(command, sensorsToRead.size() + 1);

                // ACC SENSOR
                if (mSensorAccEnabled)
                {
                    // Acceleration
                    bytes = recv(mRxBuffer, 4);
                    if (bytes < 4)
                        mAcceleration[0] = 0.0;
                    else
                        mAcceleration[0] = processAcceleration(mRxBuffer);

                    // Oritentation
                    bytes = recv(mRxBuffer, 4);
                    if (bytes < 4)
                        mAcceleration[1] = 0.0;
                    else
                        mAcceleration[1] = processAcceleration(mRxBuffer);

                    if (mAcceleration[1] < 0.0 )
                        mAcceleration[1] = 0.0;

                    if (mAcceleration[1] > 360.0 )
                        mAcceleration[1] = 360.0;

                    // Inclination
                    bytes = recv(mRxBuffer, 4);
                    if (bytes < 4)
                        mAcceleration[2] = 0.0;
                    else
                        mAcceleration[2] = processAcceleration(mRxBuffer);

                    if (mAcceleration[2] < 0.0 )
                        mAcceleration[2] = 0.0;
                    if (mAcceleration[2] > 180.0 )
                        mAcceleration[2] = 180.0;

                    emit newAccelerationdata(mAcceleration);
                }

                // MOTOR POSITION
                if (mSensorMotorPosEnabled)
                {
                    bytes = recv(mRxBuffer, 4);
                    if (bytes < 4 )
                    {
                        mMotorPosition.setX(-1);
                        mMotorPosition.setY(-1);
                    }
                    else
                    {
                        mMotorPosition.setX((mRxBuffer[0] & 0xFF) + (mRxBuffer[1] * 256));
                        mMotorPosition.setY((mRxBuffer[2] & 0xFF) + (mRxBuffer[3] * 256));

                        emit newMotorPosition(mMotorPosition);
                    }
                }

                // MOTOR SPEED
                if (mSensorMotorSpeedEnabled)
                {
                    bytes = recv(mRxBuffer, 4);
                    if (bytes < 4 )
                    {
                        mMotorSpeed.setX(-1);
                        mMotorSpeed.setY(-1);
                    }
                    else
                    {
                        mMotorSpeed.setX((mRxBuffer[0] & 0xFF) + (mRxBuffer[1] * 256));
                        mMotorSpeed.setY((mRxBuffer[2] & 0xFF) + (mRxBuffer[3] * 256));

                        emit newMotorSpeed(mMotorSpeed);
                    }
                }

                // PROXIMITY SENSOR
                if (mSensorProxEnabled)
                {
                    bytes = recv(mRxBuffer, 16);

                    if (bytes < 16)
                        mProxSensor.fill(0);
                    else
                    {
                        mProxSensor[0] = ((mRxBuffer[0] & 0xFF) + (mRxBuffer[1] * 256));
                        mProxSensor[1] = ((mRxBuffer[2] & 0xFF) + (mRxBuffer[3] * 256));
                        mProxSensor[2] = ((mRxBuffer[4] & 0xFF) + (mRxBuffer[5] * 256));
                        mProxSensor[3] = ((mRxBuffer[6] & 0xFF) + (mRxBuffer[7] * 256));
                        mProxSensor[4] = ((mRxBuffer[8] & 0xFF) + (mRxBuffer[9] * 256));
                        mProxSensor[5] = ((mRxBuffer[10] & 0xFF) + (mRxBuffer[11] * 256));
                        mProxSensor[6] = ((mRxBuffer[12] & 0xFF) + (mRxBuffer[13] * 256));
                        mProxSensor[7] = ((mRxBuffer[14] & 0xFF) + (mRxBuffer[15] * 256));

                        for (int i = 0; i < mProxSensor.size(); i++)
                            if (mProxSensor[i] < 0)
                                mProxSensor[i] = 0;

                        emit newProximityData(mProxSensor);
                    }
                }

                // LIGHT SENSOR
                if (mSensorLightEnabled)
                {
                    bytes = recv(mRxBuffer, 16);

                    if (bytes < 16)
                        mLightAvg = 0;
                    else
                    {
                        mLightAvg += ((mRxBuffer[0] & 0xFF) + (mRxBuffer[1] * 256));
                        mLightAvg += ((mRxBuffer[2] & 0xFF) + (mRxBuffer[3] * 256));
                        mLightAvg += ((mRxBuffer[4] & 0xFF) + (mRxBuffer[5] * 256));
                        mLightAvg += ((mRxBuffer[6] & 0xFF) + (mRxBuffer[7] * 256));
                        mLightAvg += ((mRxBuffer[8] & 0xFF) + (mRxBuffer[9] * 256));
                        mLightAvg += ((mRxBuffer[10] & 0xFF) + (mRxBuffer[11] * 256));
                        mLightAvg += ((mRxBuffer[12] & 0xFF) + (mRxBuffer[13] * 256));
                        mLightAvg += ((mRxBuffer[14] & 0xFF) + (mRxBuffer[15] * 256));
                        mLightAvg = (int) (mLightAvg / 8);

                        if (mLightAvg < 0)
                            mLightAvg = 0;

                        emit newLightData(mLightAvg);
                    }
                }

                // MIC SENSOR
                if (mSensorMicEnabled)
                {
                    bytes = recv(mRxBuffer, 6);

                    if (bytes < 6)
                        mMic.fill(0);
                    else
                    {
                        mMic[0] = ((mRxBuffer[0] & 0xFF) + (mRxBuffer[1] * 256));
                        mMic[1] = ((mRxBuffer[2] & 0xFF) + (mRxBuffer[3] * 256));
                        mMic[2] = ((mRxBuffer[4] & 0xFF) + (mRxBuffer[5] * 256));

                        for (int i = 0; i < mMic.size(); i++)
                            if (mMic[i] < 0)
                                mMic[i] = 0;

                        emit newMicrophoneData(mMic);
                    }
                }

                // FLOOR SENSOR
                if (mSensorFloorEnabled)
                {
                    bytes = recv(mRxBuffer, 6);
                    if (bytes < 6 )
                        mFloor.fill(0);
                    else
                    {
                        mFloor[0] = (mRxBuffer[0] & 0xFF) + (mRxBuffer[1] * 256);
                        mFloor[1] = (mRxBuffer[2] & 0xFF) + (mRxBuffer[3] * 256);
                        mFloor[2] = (mRxBuffer[4] & 0xFF) + (mRxBuffer[5] * 256);

                        emit newFloor(mFloor);
                    }
                }

                // CAMERA
                if (mSensorCameraEnabled)
                {
                    bytes = recv(imgBuffer, mPixels + 3);

                    // converting the image to QImage
                    switch(imgBuffer[0])
                    {
                    case GREY:
                    {
                        mImage = QImage(mWidth, mHeight, QImage::Format_RGB32);
                        int i = 0;
                        for(int y = 0; y < mHeight; y++)
                        {
                            for(int x = 0; x < mWidth; x++)
                            {
                                int r = (int)imgBuffer[i];
                                int g = (int)imgBuffer[i];
                                int b = (int)imgBuffer[i];
                                mImage.setPixel(x, y, qRgb(r, g, b));
                                i++;
                            }
                        }
                        break;
                    }

                    case COLOR:
                    {
                        mImage = QImage(mWidth, mHeight, QImage::Format_RGB16);
                        int i = 0;
                        for(int y = 0; y < mHeight; y++)
                        {
                            for(int x = 0; x < mWidth; x++)
                            {
                                int r = (int) imgBuffer[i*2] & 0xF8;
                                int g = (int)(imgBuffer[i*2] & 0x07) << 5 | (imgBuffer[i*2+1] & 0xE0) >> 3;
                                int b = (int)(imgBuffer[i*2+1]&0x1F) << 3;
                                mImage.setPixel(x, y, qRgb(r, g, b));
                                i++;
                            }
                        }
                        break;
                    }

                    default: break;
                    }

                    emit newImage(mImage);

                }
            }

        }

        if (mStop)
        {
            mStop = false;
            send("S\r", sizeof("S\r"));
            recv(mRxBuffer, 256);
//            qDebug() << mLogHeader << "Stop!!";
        }

        if (mReset)
        {
            mReset = false;
            send("R\r", sizeof("R\r"));
            recv(mRxBuffer, 256);
//            qDebug() << mLogHeader  << "Reset!!";
        }

        mLapseTime = mTime.elapsed() - mPrevLapseTime;
        mPrevLapseTime = mTime.elapsed();
        mMediumLapseTime += mLapseTime;

        lock.lockForWrite();
        mStep++;
        lock.unlock();
        emit newStep();
    }

    // Stoping the robot
    send("S\r", sizeof("S\r"));
    recv(mRxBuffer, 256);

}

void Epuck::setEnabled(Epuck::sensors_t sensor, bool enabled)
{
    switch(sensor)
    {
    case Epuck::ACCELEROMETER:
        mSensorAccEnabled = enabled;
        break;
    case Epuck::MIC:
        mSensorMicEnabled = enabled;
        break;
    case Epuck::MOTOR_POSITION:
        mSensorMotorPosEnabled = enabled;
        break;
    case Epuck::MOTOR_SPEED:
        mSensorMotorSpeedEnabled = enabled;
        break;
    case Epuck::SELECTOR:
        mSensorSelectorEnabled = enabled;
        break;
    case Epuck::PROXIMITY:
        mSensorProxEnabled = enabled;
        break;
    case Epuck::LIGHT:
        mSensorLightEnabled = enabled;
        break;
    case Epuck::FLOOR:
        mSensorFloorEnabled = enabled;
        break;
    case Epuck::CAMERA:
        mSensorCameraEnabled = enabled;
        break;
    }
}



void Epuck::setMotorSpeed(QPoint speed)
{
    if (speed != mMotorSpeed)
    {
        mSetMotorSpeed = true;
        lock.lockForWrite();
        mNewSpeed = speed;
        lock.unlock();
    }
}

void Epuck::setMotorPosition(QPoint pos)
{
    mSetMotorPosition = true;
    lock.lockForWrite();
    mMotorPosition = pos;
    lock.unlock();
}

void Epuck::setCameraParameters(int width, int height, image_t type, int zoom)
{
    mWidth = width;
    mHeight = height;
    mType = type;
    mZoom = zoom;

    switch(mType)
    {
    case COLOR:
        mPixels = mWidth * mHeight * 2;
        break;
    case GREY:
        mPixels = mWidth * mHeight;
        break;
    }

    mSetCameraParameters = true;
}

QPoint Epuck::getMotorSpeed(int *step)
{
    lock.lockForRead();
    QPoint p = mMotorSpeed;
    *step = mStep;
    lock.unlock();
    return p;
}

QPoint Epuck::getMotorPosition(int *step)
{
    lock.lockForRead();
    QPoint p = mMotorPosition;
    *step = mStep;
    lock.unlock();
    return p;
}

int Epuck::getStep()
{
    int t;
    lock.lockForRead();
    t = mStep;
    lock.unlock();
    return t;
}

QImage Epuck::getImage(int *step)
{
    lock.lockForRead();
    QImage image = mImage;
    *step = mStep;
    lock.unlock();
    return image;
}

float Epuck::getLapseTime(int *step)
{
    int t;
    lock.lockForRead();
    *step = mStep;
    t = mLapseTime;
    lock.unlock();
    return t;
}

QVector<int> Epuck::getMic(int *step)
{
    lock.lockForRead();
    *step = mStep;
    QVector<int> m = mMic;
    lock.unlock();
    return m;
}

QVector<int> Epuck::getProximity(int *step)
{
    lock.lockForRead();
    *step = mStep;
    QVector<int> prox = mProxSensor;
    lock.unlock();
    return prox;
}

int Epuck::getLight(int *step)
{
    lock.lockForRead();
    *step = mStep;
    int l = mLightAvg;
    lock.unlock();
    return l;
}

double Epuck::processAcceleration(const char *data)
{
    double mantis = (data[0] & 0xFF) + ((data[1] & 0xFFL) << 8) +
            (((data[2] &0x7FL) | 0x80) << 16);

    double exp = (data[3] & 0x7f) * 2 + ((data[2] & 0x80) ? 1 : 0);

    if (data[3] & 0x80)
        mantis = -mantis;

    return (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
}

QVector<int> Epuck::getAcceleration(int *step)
{
    lock.lockForRead();
    *step = mStep;
    QVector<int> acc = mAcceleration;
    lock.unlock();
    return acc;
}

void Epuck::setLed(int led, bool state)
{
    if ((led > 9) || (led < 0))
        return;

    lock.lockForWrite();
    mLedChanged = true;
    mLedsNew[led] = state;
    lock.unlock();
}

void Epuck::setBodyLed(bool state)
{
    lock.lockForWrite();
    mLedChanged = true;
    mLedsNew[8] = state;
    lock.unlock();
}

void Epuck::setFrontLed(bool state)
{
    lock.lockForWrite();
    mLedChanged = true;
    mLedsNew[9] = state;
    lock.unlock();
}

QVector<int> Epuck::getFloor(int *step)
{
    lock.lockForRead();
    *step = mStep;
    QVector<int> floor = mFloor;
    lock.unlock();
    return floor;
}

void Epuck::stop()
{
    lock.lockForWrite();
    mStop = true;
    lock.unlock();
}

void Epuck::reset()
{
    lock.lockForWrite();
    mReset = true;
    lock.unlock();
}



