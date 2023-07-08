#include "hmi_driver.h"

// xdata,ydata：0-3.3V正弦数据输入
/*! 
*  \brief        画李萨如图形,蓝底红线，300*300区域
*  \param  xdata x轴数据(0-3.3)
*  \param  ydata y轴数据(0-3.3)
*  \param  x0    图形原点x坐标
*  \param  y0    图形远点y坐标
*  \param  N     采样点数
*/
void lissajous_figures( uint16* xdata, uint16* ydata, uint16 x0,uint16 y0, uint16 N)
{
    SetBcolor(0x001F);
    SetFcolor(0x001F);
    GUI_RectangleFill(x0-150,y0-150,x0+150,y0+150);
    SetFcolor(0xF800);

    for (int i = 0;i < N-2; i++)
    {
        GUI_Line(30*xData[i] + x0, 30*yData[i] + y0, 30*xData[i + 1] + x0, 30*yData[i + 1] + y0);
    }
}