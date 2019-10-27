---
layout: post
title:  RadarDetect
date:   2019-10-26 19:49:31
comments: True
categories: C++
---

做一个雷达探测的模型，输入数据是通过采样得到的，类似于下图：
<img src="{{ site.baseurl }}/img/radar_detect.png" width="450px" height="300px" alt="雷达探测" />
目前只考虑雷达包络范围探测，不考虑目标截面对雷达回波的影响。主要分为雷达模型探测后端部分和前端绘制部分。

# 1. 雷达探测

## 直角坐标与极坐标转换
虽然提供的探测点是基于直角坐标的，但是雷达发射探测波束肯定是基于极坐标的。所以模拟雷达探测包络的时候应该将数据点转化为极坐标。

## 极坐标线性插值
直角坐标系下的线性插值很简单：
$$ y = y_1 + (x - x_1) * \frac{y_2 - y_1}{x_2 - x_1} $$
但是极坐标下怎么做还没有转过弯来。
最开始的想法是通过三角函数将极坐标参数转化为直角坐标，进行线性插值后再反转回去，但是后来比较复杂，而且一想不对，那样做插值出来点与点之间不还是直线吗，那直接在直角坐标下做就完了，干嘛还费事地进行两次坐标转换呢。而且线性插值应该指的是对参数的线性化表示，所以，如法炮制：
$$ r = r_1 + (\theta - \theta_1) * \frac{r_2 - r_1}{\theta_2 - \theta_1} $$
这样出来的点与点之间的图形不是直线，而是一条类似螺旋曲线性质的曲线：
$$ x = r * cos(\theta), y = r * sin(\theta) $$
$r$与$\theta$是线性增长的关系，但是转换到直角坐标系下，$y$与$x$就不是了。

## 全局坐标与局部坐标转换
这个问题是由地球曲率引申出来的。
在雷达的局部坐标系下，由于地球曲率的影响，在雷达探测范围400km以外的地方大概10km的离地高度计算雷达包络是可以探测到的，但是实际是探测不到的。有一个近似公式：
$$ h_{target} = \frac{1}{2} * (\sqrt{h_{radar}} + \sqrt{L_{distance}}) $$
其中$h_{target}$为目标的离地高度，$h_{radar}$为雷达的离地高度，$L_{distance}$为目标与雷达距离。
所以400km外差不多10km高是探测不到的，故应该考虑将探测目标的全局坐标转化为雷达的局部坐标。

```
//Radar.cpp

bool Radar::Detect( PlanePtr Target )
{
    bool IsTargetDetected = false;

    if (Target->GetIdentification() == GetEntity()->GetIdentification())
    {
        continue;
    }
    
    Vector3d TargetPosition = Target->GetPosition();
    Vector3d SensorPosition = GetBSE()->GetPosition();
    
    Vector3d TargetSensorVec = TargetPosition - SensorPosition;
    
    // transform coordinate from global to local
    Matrix4d SensorRotateMatrix;
    Conversion::ECEF2BodyMatrix(SensorPosition, Vector3d(1, 0, 0), SensorRotateMatrix);
    Vector3d TargetInSensorCoor = TargetSensorVec * SensorRotateMatrix;
    
    double TargetPosition_X = sqrt(TargetInSensorCoor.x*TargetInSensorCoor.x + TargetInSensorCoor.y*TargetInSensorCoor.y);
    double TargetPosition_Y = TargetInSensorCoor.z;
    double Target_theta = atan(TargetPosition_Y / TargetPosition_X);
    double Target_r = sqrt(TargetPosition_X*TargetPosition_X + TargetPosition_Y*TargetPosition_Y);
    size_t VecSize = _ParamVec.size();

    for (size_t i = VecSize-1; i > 0; i--)
    {
        TableModelParams point_1 = _ParamVec.at(i);
        TableModelParams point_2 = _ParamVec.at(i-1);
        double theta_1 = atan(point_1.CoordinateY / point_1.CoordinateX);
        double r_1 = 1000 * sqrt(point_1.CoordinateX*point_1.CoordinateX + point_1.CoordinateY*point_1.CoordinateY);
        double theta_2 = atan(point_2.CoordinateY / point_2.CoordinateX);
        double r_2 = 1000 * sqrt(point_2.CoordinateX*point_2.CoordinateX + point_2.CoordinateY*point_2.CoordinateY);
        if (theta_1 < Target_theta && Target_theta < theta_2)
        {
            // linear interpolation
            double r = r_1 + (Target_theta - theta_1) * (r_2 - r_1) / (theta_2 - theta_1);
            if (r >= Target_r)
            {
                IsTargetDetected = true;
                break;
            }
            else
            {
                IsTargetDetected = false;
                break;
            }
        }
    }

    return IsTargetDetected;
}
```

# osg绘制
创建三维画笔与基本配置信息：

```
// RadarPainter.cpp

void RadarPainter::CreateGrid()
{
    _p->_rotateCount++;

    double range = _p->_Sensor->Data()->GetMaxRange();
    float detectInterval = _p->_Sensor->GetDetectInterval().total_seconds();
    double azimuthFOVAngle = _p->_Sensor->GetFOVParameters()->_AzimuthFOVAngle;
    azimuthFOVAngle = osg::RadiansToDegrees(azimuthFOVAngle);
    double elevationFOVAngle = _p->_Sensor->GetFOVParameters()->_ElevationFOVAngle;
    elevationFOVAngle = osg::RadiansToDegrees(elevationFOVAngle);
    double offCenterAngle = _p->_Sensor->GetFOVParameters()->_OffCenterAngle;
    offCenterAngle = osg::RadiansToDegrees(offCenterAngle);
    float azimuthPointAngle = _p->_Sensor->GetAntennaPoint()->_AzimuthPointAngle;
    azimuthPointAngle = osg::RadiansToDegrees(azimuthPointAngle);
    float elevationPointAngle = _p->_Sensor->GetAntennaPoint()->_ElevationPointAngle;
    elevationPointAngle = osg::RadiansToDegrees(elevationPointAngle);
    
    double vStartAngle = -elevationFOVAngle*0.5 + elevationPointAngle;
    if (vStartAngle > 90.0)
        vStartAngle = 90.0;
    if (vStartAngle < -90.0)
        vStartAngle = -90.0;
    
    double vEndAngle = elevationFOVAngle*0.5 + elevationPointAngle;
    if (vEndAngle > 90.0)
        vEndAngle = 90.0;
    if (vEndAngle < -90.0)
        vEndAngle = -90.0;
    
    if(_p->_Sensor->GetFOVParameters()->_FOVType == FOVType::Rectangle)
    {
        float hStartAngle = -azimuthFOVAngle*0.5 + 90.0 - azimuthPointAngle;
        float hEndAngle = azimuthFOVAngle*0.5 + 90.0 - azimuthPointAngle;
        float hStartAngle2 = hEndAngle - 10.0;
        float hEndAngle2 = hEndAngle;
        float widthAngle = 10.0;
        float scanAngle = hEndAngle - hStartAngle;

        //cycle scan in angle range
        int scanIndex = (int)(scanAngle);
        int rotIndex = 0;
        if (_p->_rotateCount > scanIndex)
        {
            _p->_rotateCount = 0;
        }

        osg::Vec4d lineColor(1.0,0.0,0.0,0.2);
        osg::Vec4d gridColor(1.0,0.0,0.0,0.2);
        osg::Vec4d scanColor(0.2,1.0,1.0,0.1);
        osg::ref_ptr<RadarNode> RadarNode = new RadarNode(_p->_rotateCount,range,_p->_Sensor->GetParamVec(),SensorType::Sector,
            scanAngle,hStartAngle,hEndAngle,hStartAngle2,hEndAngle2,vStartAngle,vEndAngle,widthAngle,lineColor,gridColor,scanColor);            
        _p->_Root->addChild(RadarNode.get());
    
    }
    else if(_p->_Sensor->GetFOVParameters()->_FOVType == FOVType::Circle)
    {
        float hStartAngle = -offCenterAngle + 90.0 - azimuthPointAngle;
        float hEndAngle = offCenterAngle + 90.0 - azimuthPointAngle;
        float hStartAngle2 = hEndAngle - 10.0;
        float hEndAngle2 = hEndAngle;
        float widthAngle = 10.0;
        float scanAngle = hEndAngle - hStartAngle;
        
        //cycle scan in angle range
        int scanIndex = (int)(scanAngle);
        int rotIndex = 0;
        if (_p->_rotateCount > scanIndex)
        {
            _p->_rotateCount = 0;
        }

        osg::Vec4d lineColor(1.0,0.0,0.0,0.3);
        osg::Vec4d gridColor(1.0,0.0,0.0,0.3);
        osg::Vec4d scanColor(0.2,1.0,1.0,0.2);
        osg::ref_ptr<RadarNode> RadarNode = new RadarNode(_p->_rotateCount,range,_p->_Sensor->GetParamVec(),SensorType::Sector,
            scanAngle,hStartAngle,hEndAngle,hStartAngle2,hEndAngle2,vStartAngle,vEndAngle,widthAngle,lineColor,gridColor,scanColor);                                        
        _p->_Root->addChild(RadarNode.get());
    }

    if (GetElement())
    {
        osg::Matrixd mat;
        TSVector3d pos = GetElement()->GetLLAPosition();
        _p->_Ellipsoid->computeLocalToWorldTransformFromLatLongHeight(
            osg::DegreesToRadians(pos.y), osg::DegreesToRadians(pos.x), pos.z, mat);
        if(_p->_Sensor->GetAntennaPoint()->_AntennaPointMode == PointMode::Relative)
        {
            osg::Vec3d hpr(GetBSE()->GetHeading(), GetBSE()->GetPitch(), GetBSE()->GetRoll());
            osg::Vec3 x(1.0, 0.0, 0.0), y(0.0, 1.0, 0.0), z(0.0, 0.0, 1.0);
            osg::Quat quat(-hpr[1], x, hpr[2], y, -hpr[0], z);
            _p->_Root->setMatrix(osg::Matrix::rotate(quat)*mat);
        }
        else
        {
            _p->_Root->setMatrix(mat);
        }
    }
}
```

## 雷达包络

```
//RadarNode.h

#ifndef RADARNODE_H
#define RADARNODE_H

#include "BotomTriangleNode.h"
#include "PulseConeNode.h"
#include "SectorNode.h"
#include <osg/Group>
#include <osg/Timer>

enum SensorType
{
    Triangle,
    Sector,
    Cone
};


class SensorCallBack : public osg::NodeCallback
{
public:
    SensorCallBack(osg::Vec3d scanScle);

    ~SensorCallBack();

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);

    void SetPlay(bool play);

    void SetPlayRate(int playRate);

private:
    int _Count;
    int _PlayRate;
    bool _Play;
    osg::Vec3d _ScanScale;
    osg::Timer_t _CurrentTime;
};

class RadarNode : public osg::MatrixTransform
{
public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Constructor. </summary>
    ///
    /// <remarks>   zhuojiaoshou, 2019/10/11. </remarks>
    ///
    /// <param name="rotateCount">  Number of rotates.旋转计数（用于绘制扫描） </param>
    /// <param name="range">        The range.雷达半径 </param>
    /// <param name="table_param">  A variable-length parameters list containing table parameter.雷达包络剖面 </param>
    /// <param name="type">         The type.探测类型 </param>
    /// <param name="scanAngle">    The scan angle.平面扇形角度 </param>
    /// <param name="hStartAngle">  The start angle. </param>
    /// <param name="hEndAngle">    The end angle. </param>
    /// <param name="hStartAngle2"> The start angle 2. </param>
    /// <param name="hEndAngle2">   The end angle 2. </param>
    /// <param name="vStartAngle">  The v start angle. </param>
    /// <param name="vEndAngle">    The v end angle. </param>
    /// <param name="widthAngle">   The width angle. </param>
    /// <param name="color">        The color.边线颜色 </param>
    /// <param name="gridColor">    The grid color.网格颜色 </param>
    /// <param name="scanColor">    The scan color.扇形颜色 </param>
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    RadarNode(int rotateCount,double range, RadarTableParamsVec table_param, SensorType type, double scanAngle,float hStartAngle, float hEndAngle, float hStartAngle2, float hEndAngle2,
        float vStartAngle, float vEndAngle, float widthAngle, const osg::Vec4d& color, const osg::Vec4d& gridColor, const osg::Vec4d& scanColor );
    ~RadarNode();

    void SetPlay(bool play);

    void SetPlayRate(int playRate);

    void SetColor(const osg::Vec4d& color);

    void SetTableParam(RadarTableParamsVec table_param);

private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    /// <summary>   Creates a radar envelope. </summary>
    ///
    /// <remarks>   zhuojiaoshou, 2019/10/12. </remarks>
    ///
    /// <param name="range">        The range. </param>
    /// <param name="hStartAngle">  The start angle. </param>
    /// <param name="hEndAngle">    The end angle. </param>
    /// <param name="vStartAngle">  The v start angle. </param>
    /// <param name="vEndAngle">    The v end angle. </param>
    /// <param name="color">        The color. </param>
    /// <param name="gridColor">    The grid color. </param>
    ///
    /// <returns>   . </returns>
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    osg::ref_ptr<osg::Geode> CreateRadarEnvelope(double range, float hStartAngle, float hEndAngle, float vStartAngle, float vEndAngle, const osg::Vec4d& color, const osg::Vec4d& gridColor);

private:
    osg::ref_ptr<BotomTriangleNode>  _BotomTriangleNode;
    osg::ref_ptr<SectorNode> _SectorNode;
    osg::ref_ptr<PulseConeNode> _PlauseNode;
    
    osg::Vec3d _ScanScale;
    RadarTableParamsVec _Table_Param;
};

#endif //RADARNODE_H
```


```
//RadarNode.cpp

#include "stdAfx.h"
#include "RadarNode.h"
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Depth>
#include <osg/MatrixTransform>

SensorCallBack::SensorCallBack(osg::Vec3d scanScale)
{
    _Count = 0;
    _PlayRate = 1;
    _Play = false;
    _ScanScale = scanScale;
    _CurrentTime = osg::Timer::instance()->tick();
}

SensorCallBack::~SensorCallBack()
{

}

void SensorCallBack::SetPlay(bool play)
{
    _Play = play;
}

void SensorCallBack::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && nv->getFrameStamp() ) 
    {
        float t = osg::Timer::instance()->delta_s(_CurrentTime, osg::Timer::instance()->tick());
        if(t < 0.25 || !_Play)
        {
            traverse(node, nv);
            return ;
        }

        _CurrentTime = osg::Timer::instance()->tick();

        osg::ref_ptr<osg::MatrixTransform> mt = dynamic_cast<osg::MatrixTransform*>(node);
        if (mt.valid())
        {
            osg::Matrixd mat;;
            mat.makeRotate(osg::DegreesToRadians((double)(_Count)), osg::Vec3d(0.0, 0.0, 1.0));
            mt->setMatrix(mat*osg::Matrix::scale(_ScanScale));
        }
        _Count =  _Count+_PlayRate;
        if(_Count >= 359)
        {
            _Count = 0;
        }

    }

    traverse(node, nv);
}

void SensorCallBack::SetPlayRate( int playRate )
{
    if(playRate>20)
    {
        playRate =10;
    }
    else if (playRate <1)
    {
        playRate =1 ;
    }
    _PlayRate = playRate;
}

RadarNode::RadarNode( int rotateCount, double range, RadarTableParamsVec table_param, SensorType type, double scanAngle,float hStartAngle, float hEndAngle ,float hStartAngle2, float hEndAngle2, float vStartAngle,float vEndAngle, float widthAngle, const osg::Vec4d& lineColor, const osg::Vec4d& gridColor, const osg::Vec4d& scanColor )
{
    _ScanScale = osg::Vec3d(1,1,range/range);//high
    osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
    switch(type)
    {
    case SensorType::Triangle:
        {
            _BotomTriangleNode = new BotomTriangleNode(range,30,scanColor);
            _BotomTriangleNode->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(scanAngle),osg::Y_AXIS)*osg::Matrix::rotate(0,osg::X_AXIS));
            mt->setMatrix(osg::Matrix::scale(_ScanScale));
            mt->addChild(_BotomTriangleNode);
        }
        break;
    case SensorType::Cone:
        {
            _PlauseNode = new PulseConeNode(range,range*0.2,scanColor);//high
            _PlauseNode->setMatrix(osg::Matrix::rotate(-scanAngle,osg::X_AXIS));
            mt->addChild(_PlauseNode);
        }
        break;
    case SensorType::Sector:
        {
            osg::Vec4d scanLineColor(0.0,1.0,0.0,0.5);
            _SectorNode = new SectorNode(rotateCount,range,table_param,hStartAngle,hEndAngle,vStartAngle,vEndAngle,widthAngle,scanLineColor);
            mt->addChild(_SectorNode);
        }
        break;
    }

    this->SetTableParam(table_param);
    this->addChild(mt.get());
    this->addChild(CreateRadarEnvelope(range, hStartAngle, hEndAngle, vStartAngle, vEndAngle, lineColor, gridColor));
    mt->setUpdateCallback(new SensorCallBack(_ScanScale));
    this->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    this->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    this->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    this->getOrCreateStateSet()->setRenderBinDetails(6, "RenderBin");
}

RadarNode::~RadarNode()
{

}

osg::ref_ptr<osg::Geode> RadarNode::CreateRadarEnvelope( double range, float hStartAngle, float hEndAngle, float vStartAngle, float vEndAngle, const osg::Vec4d& lineColor, const osg::Vec4d& gridColor )
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->setDataVariance(osg::Object::DYNAMIC);
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    
    geometry->setDataVariance(osg::Object::DYNAMIC);
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

    double dtH = 5.00;
    double dtV = 5.00;
    int hNum = (hEndAngle - hStartAngle) / dtH + 1;
    int vNum = _Table_Param.size();

    hStartAngle = int(hStartAngle/dtH) * dtH;
    vStartAngle = int(vStartAngle/dtV) * dtV;

    double radius, x, y, z;
    
    for (int i = 0; i < vNum; i++)
    {
        for (int j = 0; j < hNum; j++)
        {
            x = 1000 * _Table_Param.at(i).CoordinateX * cos(osg::DegreesToRadians(j*dtH));
            y = 1000 * _Table_Param.at(i).CoordinateX * sin(osg::DegreesToRadians(j*dtH));
            z = 1000 * _Table_Param.at(i).CoordinateY;
            vertices->push_back(osg::Vec3d(x, y, z));
        }
    }
    geometry->setVertexArray(vertices.get());
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;

    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    osg::StateSet* stateSet = geometry->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND,true);
    stateSet->setMode(GL_LIGHTING,false);
    
    //entity sheltered from sensor
    osg::Depth *depth = new osg::Depth;
    depth->setWriteMask(false);
    geode->getOrCreateStateSet()->setAttributeAndModes(depth,osg::StateAttribute::ON);

    //vertical line
    for (int i = 0; i < vNum; i+=2)
    {
        geometry->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, i*hNum, hNum));
        colors->push_back(lineColor);
    }

    //horizontal line
    for (int i = 0; i < hNum; i+=2)
    {
        osg::ref_ptr<osg::DrawElementsUInt> lineStripeGrid = new osg::DrawElementsUInt(osg::PrimitiveSet::LINE_STRIP);
        for (int j = 0; j < vNum; j+=2)
        {
            lineStripeGrid->push_back(i + j*hNum);
        }
        lineStripeGrid->push_back(vertices->size());
        geometry->addPrimitiveSet(lineStripeGrid);
        colors->push_back(lineColor);
    }
    colors->push_back(gridColor);

    osg::ref_ptr<osg::DrawElementsUInt> primitives = new osg::DrawElementsUInt(GL_QUADS);
    for(int row=0;row<vNum-1;++row)
    {
        for(int col=0;col<hNum-1;++col)
        {
            primitives->push_back(col    +(row+1)*hNum);
            primitives->push_back(col    +row*hNum);
            primitives->push_back((col+1)+row*hNum);
            primitives->push_back((col+1)+(row+1)*hNum);
        }
    }
    geometry->addPrimitiveSet(primitives);

    geode->addDrawable(geometry.get());
    return geode;
}

void RadarNode::SetPlay( bool play )
{
    SensorCallBack* xkzCallback = dynamic_cast<SensorCallBack*>(this->getChild(0)->getUpdateCallback());
    if(xkzCallback)
    {
        xkzCallback->SetPlay(play);
    }
}

void RadarNode::SetPlayRate( int playRate )
{
    SensorCallBack* xkzCallback = dynamic_cast<SensorCallBack*>(this->getChild(0)->getUpdateCallback());
    if(xkzCallback)
    {
        xkzCallback->SetPlayRate(playRate);
    }
}

void RadarNode::SetColor( const osg::Vec4d& color )
{
    if (this->getNumChildren() == 2)
    {
        if(osg::Geode* geode = dynamic_cast<osg::Geode*>(this->getChild(1)))
        {
            osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode->getDrawable(0));
            if (!geometry)
            {
                return;
            }

            if(osg::Vec4Array* Colors = dynamic_cast<osg::Vec4Array*>(geometry->getColorArray()))
            {
                if (Colors->empty())
                    return;

                osg::Vec4& endColor = Colors->at(Colors->size()-1);
                endColor.set(color.r(), color.g(), color.b(), endColor.a());
                Colors->dirty();
            }
        }
    }
}

void RadarNode::SetTableParam( RadarTableParamsVec table_param )
{
    _Table_Param.clear();
    _Table_Param.assign(table_param.begin(),table_param.end());
}
```

## 扫描截面

```
//SectorNode.h

#ifndef SECTORNODE_H_
#define SECTORNODE_H_

#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Geometry>

class SectorNode :public osg::MatrixTransform
{
public:
    class RotateCallBack :public osg::NodeCallback
    {
    public:
        RotateCallBack(osg::Vec3d axis, double rotAngular)
            :_axis(axis),_speed(rotAngular),_currentRotation(0.0)
        {
        
        }

        virtual void operator()(osg::Node *node, osg::NodeVisitor *nv)
        {
            osg::MatrixTransform * mt = dynamic_cast<osg::MatrixTransform*>(node);
            if (mt)
            {
                _currentRotation += _speed;
                if (_currentRotation >( 2 * osg::PI ))
                {
                    _currentRotation -= (osg::PI * 2);
                }

                osg::Quat rotQuat(_currentRotation, _axis);
                osg::Matrix rotMatrix(rotQuat);
                mt->setMatrix(rotMatrix);
            }

            osg::NodeCallback::traverse(node, nv);
        }
    protected:
        osg::Vec3d _axis;
        double _speed;
        double _currentRotation;
    };
    
public:
    SectorNode(int roateCount, float radius, RadarTableParamsVec table_param, float hStartAngle, float hEndAngle, float vStartAngle, float vEndAngle, float angle, osg::Vec4d color);
    ~SectorNode();
private: 
    void  RenderGeometryShader(osg::ref_ptr<osg::Geometry> &geomtry,int quadSize);
private:
    osg::ref_ptr<osg::MatrixTransform> _mat;
    int _index; 
};
#endif //SECTORNODE_H
```

```
//SectorNode.cpp

#include "stdafx.h"
#include "SectorNode.h"

#pragma push_macro("CP")
#undef CP
#include <osg/LineWidth>
#include <osg/Depth>
#include <osgDB/ReadFile>
#pragma pop_macro("CP")

SectorNode::SectorNode( int roateCount, float radius, RadarTableParamsVec table_param, float hStartAngle, float hEndAngle, float vStartAngle, float vEndAngle, float angle, osg::Vec4d color )
{
    this->getOrCreateStateSet()->setMode(GL_LIGHTING,false);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    _mat = new osg::MatrixTransform;
    _mat->addChild(geode);
    this->addChild(_mat);

    osg::ref_ptr<osg::Geometry> flankgeom  = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> vt = new osg::Vec3Array;
    float dt = osg::PI/180.0;
    float x,y,z;
    float R = radius*0.98;
    int num = angle / dt;
    
    if (hStartAngle > 0)
    {
        if ((roateCount < hStartAngle) || (roateCount > hEndAngle))
        {
            return;
        }
    }
    else if (((roateCount) >= hEndAngle) &&(roateCount) <(360.0 + hStartAngle))
    {
        return;
    }

    double dtH = 1.00;
    double dtV = 1.00;
    int hNum = 2;
    int vNum = table_param.size();
    
    //flank
    for (int i = 0; i < vNum; i++)
    {
        x = 1000 * table_param.at(i).CoordinateX * cos(osg::DegreesToRadians(roateCount*dtH));
        y = 1000 * table_param.at(i).CoordinateX * sin(osg::DegreesToRadians(roateCount*dtH));
        z = 1000 * table_param.at(i).CoordinateY;
        vt->push_back(osg::Vec3d(x, y, z));
    }

    flankgeom->setVertexArray(vt);
    osg::Vec4Array* vc = new osg::Vec4Array;
    vc->push_back(color);
    flankgeom->setColorArray(vc);
    flankgeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    flankgeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_FAN,0,vt->size()));
    geode->addDrawable(flankgeom.get());
    
    //topface
    osg::ref_ptr<osg::Geometry> topfacegeom = new osg::Geometry;
    osg::Vec3Array *topvt = new osg::Vec3Array;

    float m,n, x2,y2, m2,n2, z2;
    for (int i = 0; i < hNum; i++)
    {
        for (int j = 0; j < vNum-1; j++)
        {
            x = 1000 * table_param.at(j).CoordinateX * cos(osg::DegreesToRadians((roateCount-angle*i)*dtH));        
            y = 1000 * table_param.at(j).CoordinateX * sin(osg::DegreesToRadians((roateCount-angle*i)*dtH));
            m = 1000 * table_param.at(j).CoordinateX * cos(osg::DegreesToRadians((roateCount-angle*(i+1))*dtH));
            n = 1000 * table_param.at(j).CoordinateX * sin(osg::DegreesToRadians((roateCount-angle*(i+1))*dtH));        
            z = 1000 * table_param.at(j).CoordinateY;

            x2 = 1000 * table_param.at(j+1).CoordinateX * cos(osg::DegreesToRadians((roateCount-angle*i)*dtH));     
            y2 = 1000 * table_param.at(j+1).CoordinateX * sin(osg::DegreesToRadians((roateCount-angle*i)*dtH));
            m2 = 1000 * table_param.at(j+1).CoordinateX * cos(osg::DegreesToRadians((roateCount-angle*(i+1))*dtH));
            n2 = 1000 * table_param.at(j+1).CoordinateX * sin(osg::DegreesToRadians((roateCount-angle*(i+1))*dtH));
            z2 = 1000 * table_param.at(j+1).CoordinateY;

            topvt->push_back(osg::Vec3d(x,y,z));
            topvt->push_back(osg::Vec3d(m,n,z));
            topvt->push_back(osg::Vec3d(x2,y2,z2));
            topvt->push_back(osg::Vec3d(m2,n2,z2));
        }
    }

    topfacegeom->setVertexArray(topvt);
    osg::Vec4Array* gd = new osg::Vec4Array;
    for (int i = 0; i < topvt->size(); i++)
    {
        if (i < (topvt->size() / 2))
        {
            if (i % 2 == 0)
            {
                gd->push_back(color);
            }
            else
            {
                gd->push_back(osg::Vec4(0.5,1.0,0.0,0.1));
            }
        }
        else
        {
            if (i % 2 == 0)
            {
                gd->push_back(osg::Vec4(0.5,1.0,0.0,0.1));
            }
            else
            {
                gd->push_back(osg::Vec4(1.0,1.0,0.0,0.0));
            }
        }
    }
    topfacegeom->setColorArray(gd);
    topfacegeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    topfacegeom->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP,0,topvt->size()));
    RenderGeometryShader(topfacegeom,vNum);
    geode->addDrawable(topfacegeom.get());

    //entity sheltered from sensor
    osg::Depth *depth = new osg::Depth;
    depth->setWriteMask(false);
    geode->getOrCreateStateSet()->setAttributeAndModes(depth,osg::StateAttribute::ON);
}

SectorNode::~SectorNode()
{

}

void SectorNode::RenderGeometryShader( osg::ref_ptr<osg::Geometry>& geomtry,int quadSize )
{
    if (!geomtry)
    {
        return;
    }
    osg::StateSet * stateSet = geomtry->getOrCreateStateSet();
    stateSet->setMode(GL_BLEND,true);
    stateSet->setMode(GL_DEPTH_TEST,true);
    stateSet->setMode(GL_LIGHTING,false);
    
    osg::ref_ptr<osg::DrawElementsUInt> primitives = new osg::DrawElementsUInt(GL_QUAD_STRIP);
    for (int i = 0; i < quadSize; i++)
    {
        primitives->push_back(2*i);
        primitives->push_back((2*i)+1);
        primitives->push_back((2*i)+2);
        primitives->push_back((2*i)+3);
    }
    geomtry->addPrimitiveSet(primitives);
}
```

最终实现效果如下：
[osg雷达探测包络](https://www.bilibili.com/video/av73542029")