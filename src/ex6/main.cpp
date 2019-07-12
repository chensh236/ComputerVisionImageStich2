#include "ImageProcess.h"

int main() {
	//路径
	string file;
	cin>>file;
	string ss = "../../" + file + "/";
	int picSum;
	cout<<"Please input the sum of the images"<<endl;
	cin>>picSum;
	ImageProcess ip(ss, picSum);
	return 0;
}
