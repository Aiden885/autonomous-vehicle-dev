/**
 * @brief �����ļ�
 * @file main.cpp
 * @version 0.0.1
 * @author Hongyu Liu
 * @date 2023-12-18
 */

#include <iostream>
#include "coordinateTransFunctional.h"
#include "ztgeographycoordinatetransform.h"

int main()
{
    ZtGeographyCoordinateTransform J;
    /**
     * @brief �������ݳ�ʼ����
     */
    double x = 10;
    double y = 10;
    double z  = 10;
    double lat = 1;
    double lon = 1;
    double high = 1;
    double x_v = 5;
    double y_v = 5;
    double z_v = 5;
    double theta = 60;
    double x_w = 1;
    double y_w = 1;
    double z_w = 1;
    /**
     * @brief ENUתLLA(��ά)
     */
    XY2BLParam XY2BL{
        .Zt = J,
    };
    XY2BLInput iXY2BL{
        .xInput = x,
        .yInput = y,
    };
    XY2BLOutput oXY2BL{
        .latOutput = lat,
        .lonOutput = lon,
    };
    /**
     * @brief LLAתENU����ά��
     */
    BL2XYParam BL2XY{
        .Zt = J,
    };
    BL2XYInput iBL2XY{
        .latInput = lat,
        .lonInput = lon,
    };
    BL2XYOutput oBL2XY{
        .xOutput = x,
        .yOutput = y,
    };
    /**
     * @brief ENUתLLA(��ά)
     */
    XYZ2BLHParam XYZ2BLH{
        .Zt = J,
    };
    XYZ2BLHInput iXYZ2BLH{
        .xInput = x,
        .yInput = y,
        .zInput = z,
    };
    XYZ2BLHOutput oXYZ2BLH{
        .latOutput = lat,
        .lonOutput = lon,
        .highOutput = high,        
    };
    /**
     * @brief LLAתENU����ά��
     */
    BLH2XYZParam BLH2XYZ{
         .Zt = J,
    };
    BLH2XYZInput iBLH2XYZ{
        .latInput = lat,
        .lonInput = lon,
        .highInput = high, 
    };
    BLH2XYZOutput oBLH2XYZ{
        .xOutput = x,
        .yOutput = y,
        .zOutput = z,
    };
    /**
     * @brief ��������ϵתȫ������ϵ��ENU��:
     */
    V2BLHParam V2BLH{
        .Zt = J,
    };
    V2BLHInput iV2BLH{
        .x0Input = x,
        .y0Input = y,
        .z0Input = z,
        .x_vInput = x_v,
        .y_vInput = y_v,
        .z_vInput = z_v,
        .thetaInput = theta,
    };
    V2BLHOutput oV2BLH{
        .x_wOutput = x_w,
        .y_wOutput = y_w,
        .z_wOutput = z_w,
    };
    /**
     * @brief ȫ������ϵ��ENU��ת��������ϵ
     */
    BLH2VParam BLH2V{
        .Zt = J,
    };
    BLH2VInput iBLH2V{
        .x0Input = x,
        .y0Input = y,
        .z0Input = z,
        .x_wInput = x_w,
        .y_wInput = y_w,
        .z_wInput = z_w,
        .thetaInput = theta,
    };
    BLH2VOutput oBLH2V{
        .x_vOutput = x_v,
        .y_vOutput = y_v,
        .z_vOutput = z_v,
    };
    /**
     * @brief ���������
     */
    ztGeographyCoordinateTransformXY2BL(XY2BL, iXY2BL, oXY2BL);
    std::cout << "this is XY2BL lat output: " << oXY2BL.latOutput << std::endl;
    std::cout << "this is XY2BL lon output: " << oXY2BL.lonOutput << std::endl;
    ztGeographyCoordinateTransformBL2XY(BL2XY, iBL2XY, oBL2XY);
    std::cout << "this is BL2XY x output: " << oBL2XY.xOutput << std::endl;
    std::cout << "this is BL2XY y output: " << oBL2XY.yOutput << std::endl;
    ztGeographyCoordinateTransformXYZ2BLH(XYZ2BLH, iXYZ2BLH, oXYZ2BLH);
    std::cout << "this is XYZ2BLH lat output: " << oXYZ2BLH.latOutput << std::endl;
    std::cout << "this is XYZ2BLH lon output: " << oXYZ2BLH.lonOutput << std::endl;
    std::cout << "this is XYZ2BLH height output: " << oXYZ2BLH.highOutput << std::endl;
    ztGeographyCoordinateTransformBLH2XYZ(BLH2XYZ, iBLH2XYZ, oBLH2XYZ);
    std::cout << "this is BLH2XYZ x output: " << oBLH2XYZ.xOutput << std::endl;
    std::cout << "this is BLH2XYZ y output: " << oBLH2XYZ.yOutput << std::endl;
    std::cout << "this is BLH2XYZ z output: " << oBLH2XYZ.zOutput << std::endl;
    ztGeographyCoordinateTransformV2BLH(V2BLH, iV2BLH, oV2BLH);
    std::cout << "this is V2BLH x_w output: " << oV2BLH.x_wOutput << std::endl;
    std::cout << "this is V2BLH y_w output: " << oV2BLH.y_wOutput << std::endl;
    std::cout << "this is V2BLH z_w output: " << oV2BLH.z_wOutput << std::endl;
    ztGeographyCoordinateTransformBLH2V(BLH2V, iBLH2V, oBLH2V);
    std::cout << "this is BLH2V x_v output: " << oBLH2V.x_vOutput << std::endl;
    std::cout << "this is BLH2V y_v output: " << oBLH2V.y_vOutput << std::endl;
    std::cout << "this is BLH2V z_v output: " << oBLH2V.z_vOutput << std::endl;
    return 0;
}