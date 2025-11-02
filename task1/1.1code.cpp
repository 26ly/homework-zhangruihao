#include <iostream> 
#include <vector>
using namespace std;

int main() 
{
    cout << "请按照以下格式输入数据：" << endl;
    cout << "1. 第一行输入运算符（+ 或 x）" << endl;
    cout << "2. 第二行输入第一个矩阵的行数和列数" << endl;
    cout << "3. 接下来的r1行输入第一个矩阵的元素" << endl;
    cout << "4. 随后一行第二个输入矩阵的行数和列数" << endl;
    cout << "5. 接下来的r2行输入第二个矩阵的元素" << endl;
    cout << "请输入数据：" << endl;
    
    char op;
    cin >> op;
    int r1, c1;
    cin >> r1 >> c1;
    vector<vector<int> > matrix1(r1, vector<int>(c1));
    for (int i = 0; i < r1; i++) 
	{
        for (int j = 0; j < c1; j++) 
		{
            cin >> matrix1[i][j];
        }
    }
    
    int r2, c2;
    cin >> r2 >> c2;
    vector<vector<int> > matrix2(r2, vector<int>(c2));
    for (int i = 0; i < r2; i++) 
	{
        for (int j = 0; j < c2; j++) 
		{
            cin >> matrix2[i][j];
        }
    }
    
    if (op == '+') 
	{
		//判断是否符合加法规则 
        if (r1 != r2 || c1 != c2) 
		{
            cout << "BAD INPUT!" << endl;
            return 0;
        }
        cout << "矩阵加法结果：" << endl;
        vector<vector<int> > result(r1, vector<int>(c1));
        for (int i = 0; i < r1; i++) 
		{
            for (int j = 0; j < c1; j++) 
			{
                result[i][j] = matrix1[i][j] + matrix2[i][j];
            }
        }
        for (int i = 0; i < r1; i++) 
		{
            for (int j = 0; j < c1; j++) 
			{
                if (j != 0) cout << " ";
                cout << result[i][j];
            }
            cout << endl;
        }
    }
	else if (op == 'x') 
	{
        if (c1 != r2) 
		{
            cout << "BAD INPUT!" << endl;
            return 0;
        }
        cout << "矩阵乘法结果：" << endl;
        vector<vector<int> > result(r1, vector<int>(c2, 0));
        for (int i = 0; i < r1; i++) 
		{
            for (int j = 0; j < c2; j++) 
			{
                for (int k = 0; k < c1; k++) 
				{
                    result[i][j] += matrix1[i][k] * matrix2[k][j];
                }
            }
        }
        for (int i = 0; i < r1; i++) 
		{
            for (int j = 0; j < c2; j++) 
			{
                if (j != 0) cout << " ";//空格分隔 
                cout << result[i][j];
            }
            cout << endl;
        }
    } 
	else 
	{
        cout << "BAD INPUT!" << endl;
    }
    return 0;
}
