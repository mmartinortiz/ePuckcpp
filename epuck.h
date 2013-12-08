#ifndef EPUCK_H
#define EPUCK_H

#include <QObject>

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

/**
 * @brief Esta clase está inspirada en la que desarrollé anteriormente y se llama qEpuckSignal
 *
 */
#include <QThread>
#include <QPoint>
#include <QTime>
#include <QString>
#include <QVector>
#include <QReadWriteLock>
#include <QWaitCondition>
#include <QImage>

#include "serialcomm.h"

class Epuck : public QThread
{
    Q_OBJECT

public:
    explicit Epuck(const QString port = "");

    enum sensors_t {ACCELEROMETER, MIC, MOTOR_POSITION, MOTOR_SPEED, SELECTOR, PROXIMITY, LIGHT, FLOOR, CAMERA};

    enum image_t {GREY, COLOR};

    const static int MAX_LEDS = 8;

    void setEnabled(sensors_t sensor, bool enabled);
    void setPort(QString port);
    void setMotorSpeed(QPoint speed);
    void setMotorSpeed(int left, int right) {setMotorSpeed(QPoint(left, right));}
    void setMotorPosition(QPoint pos);
    void setMotorPosition(int left, int right) {setMotorPosition(QPoint(left, right));}

    void setCameraParameters(int width = 40, int height = 40, image_t type = COLOR, int zoom = 8);

    QPoint getMotorSpeed(int *step);
    QPoint getMotorPosition(int *step);
    QVector<int> getMic(int *step);
    QVector<int> getProximity(int *step);
    QVector<int> getAcceleration(int *step);
    QVector<int> getFloor(int *step);
    int getLight(int *step);
    int getStep();
    QImage getImage(int *step);
    int getMediumLapseTime() {return mMediumLapseTime/mStep; }

    float getLapseTime(int *step);
    bool isConnected() {return mIsConnected;}

    void setLed(int led, bool state);
    void setBodyLed(bool state);
    void setFrontLed(bool state);
    void stop();
    void reset();
    void finish() {mAbortThread = true;}

signals:
    void portOpened();
    void cannotOpenPort();

    void newAccelerationdata(QVector<int> acceleration);
    void newProximityData(QVector<int> proximity);
    void newMotorSpeed(QPoint motorSpeed);
    void newMotorPosition(QPoint motorPosition);
    void newMicrophoneData(QVector<int> microphone);
    void newLightData(int light);
    void newFloor(QVector<int> floor);
    void newImage(QImage image);

    void newStep();

protected:
    void run();

private:
    int mStep;

    QReadWriteLock lock;
    QWaitCondition wait;

    bool initPort();
    int send(const char *msg, int size);
    int recv(char *buffer, int size);

    bool mAbortThread;
    bool getSensorData;
    bool mSetMotorSpeed;
    bool mSetMotorPosition;
    bool mSetCameraParameters;
    bool mStop;
    bool mReset;

    bool mSensorProxEnabled;
    bool mSensorAccEnabled;
    bool mSensorMicEnabled;
    bool mSensorMotorPosEnabled;
    bool mSensorMotorSpeedEnabled;
    bool mSensorSelectorEnabled;
    bool mSensorLightEnabled;
    bool mSensorFloorEnabled;
    bool mSensorCameraEnabled;

    // Log
    QString mLogHeader;

    // Connection
    char *connectionPort;
    bool mIsConnected;

    // Timer
    QTime mTime;
    int mLapseTime;
    int mPrevLapseTime;
    int mMediumLapseTime;

    // Motor
    QPoint mMotorPosition;
    QPoint mMotorSpeed;
    QPoint mNewSpeed;

    // Proximity
    QVector<int> mProxSensor;

    // Light
    int mLightAvg;

    // Mic
    QVector<int> mMic;

    // Acceleration
    double processAcceleration(const char* data);
    QVector<int> mAcceleration;

    // Leds
    bool mLedChanged;
    QVector<bool> mLeds;
    QVector<bool> mLedsNew;

    // Floor
    QVector<int> mFloor;

    // Camera
    QImage mImage;
    char imgBuffer[4050];				/**< image data; 4050 is the maximum number of bytes that can be received at one time from the robot.*/
    int mType;						/**< type of the image: color (value 1) or grayscale (value 0)*/
    int mWidth;						/**< width of the image to be received*/
    int mHeight;					/**< height of the image to be received*/
    int mPixels;					/**< total number of pixels (bytes) to be received; in case of grayscale image it is width*height, in case of color image it is width*height*2 (RGB565)*/
    int mZoom;

    // Communication
#ifdef __WIN32__
    TCommPort *mComm;    /**< pointer to the serial port for the bluetooth device (Windows)*/
#else
    SerialComm *mComm;   /**< pointer to the serial port for the bluetooth device (Linux, MacOS)*/
#endif

    char mRxBuffer[256];
    char mCommand[20];
};

#endif // EPUCK_H
