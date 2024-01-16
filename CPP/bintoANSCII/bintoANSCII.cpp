#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <iostream>
#include <fstream>
#include <ostream>
using namespace std;
//文件头，共84字节
struct Head
{
	char partName[80];//零件名称
	int  faceNum;//面的数目
};

//点，三个float类型的，大小为12字节
struct Point
{
	float x;
	float y;
	float z;
};

//法线
struct Normal
{
	float i;
	float j;
	float k;
};

//三角面，由一个法线，三个点，和一个两字节的保留项，一共50字节
struct Face
{
	Normal normal;//法线结构体占12个字节
	Point  p1;//三角面第一个点结构体占12个字节
	Point  p2;//三角面第二个点结构体占12个字节
	Point  p3;//三角面第三个点结构体占12个字节
	char  info[2];//保留数据，可以不用管
};

int main()
{
	Head head;
	Normal normal;
	ifstream fileIn;   //定义输入流fileIn
	ofstream fileOut;//定义输出流fileOut
	//char fileName[128];
	fileIn.open("E:\\Files\\STL\\rect_cenlei_p37.stl", ios::binary);
	fileIn.read(head.partName, 80);
	fileIn.read((char*)&head.faceNum, 4);
	Face* faces = new Face[(head.faceNum * sizeof(Face))];//最重要一点，申请一个动态内存即根据三角面片的数目，创建一个Face类型的数组，然后将三角面的数据读到这里面即下面操作
	//循环读取三角面片数据
	for (int i = 0; i < head.faceNum; i++)
	{

		fileIn.read((char*)&faces[i].normal, 12);//读取法线数据

		fileIn.read((char*)&faces[i].p1, 12);//读取顶点1的数据

		fileIn.read((char*)&faces[i].p2, 12);//读取顶点2的数据

		fileIn.read((char*)&faces[i].p3, 12);//读取顶点3的数据

		fileIn.read((char*)&faces[i].info, 2);//读取保留项数据，这一项一般没什么用，这里选择读取是为了移动文件指针
	}
	fileIn.close();

	fileOut.open("E:\\Files\\STL\\rect_cenlei_p37_ANSCII.stl", ios::out);
	fileOut << head.partName << "  " << endl;
	for (int i = 0; i < head.faceNum; i++)
	{
		fileOut << "facet normal" << " " << faces[i].normal.i << " " << faces[i].normal.j << " " << faces[i].normal.k << " " << endl;
		fileOut << "outer loop" << endl;
		fileOut << " vertex" << " " << faces[i].p1.x << " " << faces[i].p1.y << " " << faces[i].p1.z << " " << endl;
		fileOut << " vertex" << " " << faces[i].p2.x << " " << faces[i].p2.y << " " << faces[i].p2.z << " " << endl;
		fileOut << " vertex" << " " << faces[i].p3.x << " " << faces[i].p3.y << " " << faces[i].p3.z << " " << endl;
		fileOut << "endloop" << endl;
		fileOut << "endfacet" << endl;
	}
	fileOut << "endsolid" << endl;

	return 0;
}