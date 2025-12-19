#include<iostream>
#include<iomanip>//格式化输出 
#include<cmath>
using namespace std;
//角度转弧度 
double toradians(double degrees)
{
	return degrees * 3.1415926 / 180;
}

class Matrix
{
private:
	double data[3][3];
public:
	//构造函数 
	Matrix()
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				data[i][j]=(i==j) ? 1 : 0;
			}
		}
	}
	
	void set(int i,int j,double value)
	{
		data[i][j]=value;
	}
	
	//矩阵乘法
    Matrix multiply(Matrix &matrix2)
	{
        Matrix result;
        for (int i=0;i<3;i++) 
		{
            for (int j=0;j<3;j++) 
			{
                double sum=0.0;
                for (int k=0;k<3;k++) 
				{
                    sum+=data[i][k]*matrix2.data[k][j];
                }
                result.data[i][j]=sum;
            }
        }
        return result;
    }
    //最终结果 
    void transform(double x, double y, double &xout, double &yout)
	{
        xout=data[0][0]*x+data[0][1]*y+data[0][2];
        yout=data[1][0]*x+data[1][1]*y+data[1][2];
    }
};

// 创建旋转矩阵
Matrix rotation(double degree) 
{
    Matrix mat; 
    double theta=toradians(degree);
    double cos_theta=cos(theta);
    double sin_theta=sin(theta);
    
    mat.set(0,0,cos_theta);
    mat.set(0,1,-sin_theta);
    mat.set(1,0,sin_theta);
    mat.set(1,1,cos_theta);
    return mat;
}
// 创建平移矩阵
Matrix translation(double tx, double ty) 
{
    Matrix mat;
    mat.set(0,2,tx);
    mat.set(1,2,ty);
    return mat;
}

int main()
{
	double x,y,theta,tx,ty;
	cout<<"请输入x,y,theta,tx,ty的值:"<<endl;
	cin>>x>>y>>theta>>tx>>ty;
	Matrix rot=rotation(theta);
	Matrix tra=translation(tx,ty);
	
	//先旋转后平移
	Matrix m1=tra.multiply(rot);//运算矩阵 
	double x1,y1;
	m1.transform(x,y,x1,y1);
	//先平移后旋转
	Matrix m2=rot.multiply(tra);
	double x2,y2;
	m2.transform(x,y,x2,y2);
	
	cout<<fixed<<setprecision(2);
	cout<<"x1:"<<x1<<"y1:"<<y1<<"x2:"<<x2<<"y2:"<<y2<<endl;
	
	return 0;
 } 
