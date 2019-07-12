#include "ImageProcess.h"
// 拼合过程
ImageProcess::ImageProcess(string fileDic, const int picSum){
    // 文件处理与sift提取
    readFile(fileDic, picSum);
	matching();
    
}

// 文件读取
void ImageProcess::readFile(string fileDic, const int picSum){
    for(int i = 0; i < picSum; i++){
		Image currentImg;
        // format: fileDic/index.bmp
        // 顺序读取
		string inputFileName = fileDic + to_string (i + 1) + ".bmp";
		//求出投影
		currentImg.projectedSrc = Projection::imageProjection(CImg<unsigned char>(inputFileName.c_str()));
        // 转换为灰度图之后求sift特征
		currentImg.features = siftAlgorithm( toGrayScale(currentImg.projectedSrc) );
		imgs.push_back(currentImg);
        // sift计算特征
	}
}

// 转换灰度图像
CImg<unsigned char> ImageProcess::toGrayScale(const CImg<unsigned char>& src){
    // 如果是灰度图像返回
    if (src.spectrum() == 1){
        return src;
    }
    // 创建灰度图像
    CImg<unsigned char> gray(src.width(), src.height(), src.depth(), 1, 0);
    // 灰度图转换

    cimg_forXY(src, x, y){
        gray(x, y) = 0.299 * (float)src(x, y, 0) + 0.587 * (float)src(x, y, 1) + 0.114 * (float)src(x, y, 2);
    }
    return gray;
}

// sift特征提取
//https://www.cnblogs.com/wangguchangqing/p/9176103.html
map<vector<float>, VlSiftKeypoint> ImageProcess::siftAlgorithm(const CImg<unsigned char>& projectedSrc) {

    // 新建vlFeat的图像
	vl_sift_pix *vlImg = new vl_sift_pix[projectedSrc.width() * projectedSrc.height()];

	cimg_forXY(projectedSrc, x, y){
		vlImg[y * projectedSrc.width() + x] = (float)projectedSrc(x, y, 0);
	}

	// 金字塔层数 每层中的图像
	int noctaves = NOTAVES_NUM, nlevels = LEVEL_NUM;
	VlSiftFilt *siftFilt = vl_sift_new(projectedSrc.width(), projectedSrc.height(), noctaves, nlevels, 0);

	map<vector<float>, VlSiftKeypoint> features;

	// 计算DOG
	if (vl_sift_process_first_octave(siftFilt, vlImg) != VL_ERR_EOF) {
		while (true) {
			// 查找关键点
			vl_sift_detect(siftFilt);
			VlSiftKeypoint *pKeypoint = siftFilt->keys;
			// 对于每个关键点进行描述子构造
			for (int i = 0; i < siftFilt->nkeys; i++, pKeypoint++) {
				VlSiftKeypoint vlKeypoint = *pKeypoint;

				// 四个角度
				double angles[4];
				int angleCount = vl_sift_calc_keypoint_orientations(siftFilt, angles, &vlKeypoint);
				for (int j = 0; j < angleCount; j++) {
					double tempAngle = angles[j];
                    // 初始化描述子
					vl_sift_pix descriptors[DESCRIPTOR_SUM];

					// 获得每个关键点的描述子
					vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &vlKeypoint, tempAngle);

					vector<float> descriptorVec;
					for(int j = 0; j < DESCRIPTOR_SUM; j++){
						descriptorVec.push_back(descriptors[j]);
					}
					vlKeypoint.ix = vlKeypoint.x;
					vlKeypoint.iy = vlKeypoint.y;
					features.insert(pair<vector<float>, VlSiftKeypoint>(descriptorVec, vlKeypoint));
				}
			}
			if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
				break;
			}
		}
	}
    // 销毁指针
	vl_sift_delete(siftFilt);
	delete[] vlImg;
	vlImg = NULL;
	return features;
}

void ImageProcess::matching(){

	// 用于判断两张图像是否需要进行拼接，适合乱序图片
	stichingMat = new bool* [imgs.size()];
	//初始化矩阵
	for(int i = 0; i < imgs.size(); i++){
		stichingMat[i] = new bool[imgs.size()];
		for(int j = 0; j < imgs.size(); j++){
			stichingMat[i][j] =false;
		}
	}

	// 对于该图片，存储下一张图片序号
	vector< vector<int> > nextIndex(imgs.size());

	// 对所有图像进行遍历
	for(int i = 0; i < imgs.size(); ++i){
		for(int j = 0; j < imgs.size(); ++j){

			// 去除自身
			if (i == j)
				continue;

			//避免重复计算
			if(stichingMat[j][i] == true){
				stichingMat[i][j] = true;
				nextIndex[i].push_back(j);
				continue;
			}

			vector<ImgPair> ImgPairs = getImgPair(imgs[i], imgs[j]);
			if (ImgPairs.size() >= THRESHOLD) {
				stichingMat[i][j] = true;
				nextIndex[i].push_back(j);
			}
		}
	}
//	// 显示是否是匹配的
//	for(int i = 0; i < imgs.size(); i++){
//		for(int j = 0; j < imgs.size(); j++){
//			cout<<stichingMat[i][j]<<" ";
//		}
//		cout<<endl;
//	}

	// 获得这组图像的中间图像,从中间图像开始拼合
	int startIndex = getMiddleIndex(nextIndex);

	// 记录前一张被拼合的图像
	int preStichingIndex = startIndex;

	// 记录仍然没有被拼合的图像
	queue<int> waitForStiching;
	waitForStiching.push(startIndex);

	// 当前被拼合的图像
	result = imgs[startIndex].projectedSrc;
	int stichingCount = 0;
	while (!waitForStiching.empty()) {
		// 源图像索引
		int srcIndex = waitForStiching.front();
		waitForStiching.pop();

		// 图像从后往前遍历
		for (int i = nextIndex[srcIndex].size() - 1; i >= 0; i--){

			// 目标图像索引
			int dstIndex = nextIndex[srcIndex][i];

			// 不是拼合图像
			if (!stichingMat[srcIndex][dstIndex]){
				continue;
			}
			stichingMat[srcIndex][dstIndex] = stichingMat[dstIndex][srcIndex] = false;
			waitForStiching.push(dstIndex);
			// 通过KT树寻找匹配点
			vector<ImgPair> srcToDstPair = getImgPair(imgs[srcIndex], imgs[dstIndex]);
			vector<ImgPair> dstToSrcPair = getImgPair(imgs[dstIndex], imgs[srcIndex]);
            // 直方图规定化
             //transfer tran(imgs[dstIndex].projectedSrc, imgs[srcIndex].projectedSrc, imgs[dstIndex].projectedSrc);
            //else
            //transfer tran(imgs[dstIndex].projectedSrc, result, imgs[dstIndex].projectedSrc);
            cout<< srcIndex<<" "<<dstIndex<<endl;
			// 获取匹配点的数组
			if (srcToDstPair.size() > dstToSrcPair.size()) {
				dstToSrcPair.clear();
				for (int i = 0; i < srcToDstPair.size(); i++) {
					ImgPair input(srcToDstPair[i].dst, srcToDstPair[i].src);
					dstToSrcPair.push_back(input);
				}
			}
			else {
				srcToDstPair.clear();
				for (int i = 0; i < dstToSrcPair.size(); i++) {
					ImgPair input(dstToSrcPair[i].dst, dstToSrcPair[i].src);
					srcToDstPair.push_back(input);
				}
			}

			// Finding homography by RANSAC.
			Homography forward_H = RANSAC(dstToSrcPair);
			Homography backward_H = RANSAC(srcToDstPair);

			// Calculate the size of the image after stitching.
			// 获得新图像大小
			float min_x = getMinXAfterWarping(imgs[dstIndex].projectedSrc, forward_H);
			min_x = (min_x < 0) ? min_x : 0;
			float min_y = getMinYAfterWarping(imgs[dstIndex].projectedSrc, forward_H);
			min_y = (min_y < 0) ? min_y : 0;
			float max_x = getMaxXAfterWarping(imgs[dstIndex].projectedSrc, forward_H);
			max_x = (max_x >= result.width()) ? max_x : result.width();
			float max_y = getMaxYAfterWarping(imgs[dstIndex].projectedSrc, forward_H);
			max_y = (max_y >= result.height()) ? max_y : result.height();

			int new_width = ceil(max_x - min_x);
			int new_height = ceil(max_y - min_y);

			CImg<unsigned char> a(new_width, new_height, 1, 3, 0);
			CImg<unsigned char> b(new_width, new_height, 1, 3, 0);

			// 进行双线性插值求出坐标
			warpingImageByHomography(imgs[dstIndex].projectedSrc, a, backward_H, min_x, min_y);
			// 不需用双线性插值求出坐标
			movingImageByOffset(result, b, min_x, min_y);
			// 更新位置
			updateFeaturesByHomography(imgs[dstIndex].features, forward_H, min_x, min_y);
			updateFeaturesByOffset(imgs[preStichingIndex].features, min_x, min_y);

			// Blending two images.
			result = blendTwoImages(a, b);

			preStichingIndex = dstIndex;
			result.display();
		}
		stichingCount++;
	}
	CImg<unsigned char> tmp = result;
	equalization(tmp, 1);

	CImg<float> yCbCrResult(result.width(), result.height(), 1, 3, 0); //To one channel
	cimg_forXY(result, x, y){
		float Y = 0.299 * (float)result(x, y, 0) + 0.857 * (float)result(x, y, 1) + 0.114 * (float)result(x, y, 2);
		float Cb = 128.0 - 0.168736 * (float)result(x, y, 0) - 0.331264 * (float)result(x, y, 1) + 0.5 * (float)result(x, y, 2);
		float Cr = 128.0 + 0.5 * (float)result(x, y, 0) - 0.418688 * (float)result(x, y, 1) - 0.081312 * (float)result(x, y, 2);
		yCbCrResult(x, y, 0) = Y > 0 ? (Y < 256 ? Y : 255) : 0;
		yCbCrResult(x, y, 1) = Cb > 0 ? (Cb < 256 ? Cb : 255) : 0;
		yCbCrResult(x, y, 2) = Cr > 0 ? (Cr < 256 ? Cr : 255) : 0;
	}

	CImg<float> yCbCrtmp(result.width(), result.height(), 1, 3, 0); //To one channel
	cimg_forXY(tmp, x, y){
		float Y = 0.299 * (float)tmp(x, y, 0) + 0.857 * (float)tmp(x, y, 1) + 0.114 * (float)tmp(x, y, 2);
		float Cb = 128.0 - 0.168736 * (float)tmp(x, y, 0) - 0.331264 * (float)tmp(x, y, 1) + 0.5 * (float)tmp(x, y, 2);
		float Cr = 128.0 + 0.5 * (float)tmp(x, y, 0) - 0.418688 * (float)tmp(x, y, 1) - 0.081312 * (float)tmp(x, y, 2);
		yCbCrtmp(x, y, 0) = Y > 0 ? (Y < 256 ? Y : 255) : 0;
		yCbCrtmp(x, y, 1) = Cb > 0 ? (Cb < 256 ? Cb : 255) : 0;
		yCbCrtmp(x, y, 2) = Cr > 0 ? (Cr < 256 ? Cr : 255) : 0;
	}

	cimg_forXY(yCbCrResult, x, y){
		yCbCrResult(x, y, 0) = yCbCrResult(x, y, 0) * 19.0 / 20.0 + yCbCrtmp(x, y, 0) / 20.0;
		float R = (float)yCbCrResult(x, y, 0) + 1.402 * ((float)yCbCrResult(x, y, 2) - 128.0);
		float G = (float)yCbCrResult(x, y, 0) - 0.34414 * ((float)yCbCrResult(x, y, 1) - 128.0) - 0.71414 * ((float)yCbCrResult(x, y, 2) - 128.0);
		float B = (float)yCbCrResult(x, y, 0) + 1.772 * ((float)yCbCrResult(x, y, 1) - 128.0);
		result(x, y, 0) = R > 0 ? (R < 256 ? R : 255) : 0;
		result(x, y, 1) = G > 0 ? (G < 256 ? G : 255) : 0;
		result(x, y, 2) = B > 0 ? (B < 256 ? B : 255) : 0;
	}

	result.display();
}

vector<ImgPair> ImageProcess::getImgPair(Image& imgA, Image& imgB){
	// 这里使用KD树节省查找时间
	// dataType	type of data (VL_TYPE_FLOAT or VL_TYPE_DOUBLE)
	// dimension	data dimensionality.
	// numTrees	number of trees in the forest.
	// distance	type of distance norm (VlDistanceL1 or VlDistanceL2). 这里选择L1范数，但是我在另外一篇使用的是L2范数

	VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, DESCRIPTOR_SUM, 1, VlDistanceL1);

	// 共有128 * number of keypoints 的特征点
	float *imgA_Descriptors = new float[imgA.features.size() * DESCRIPTOR_SUM];

	// 将其放入到数组中
	auto it = imgA.features.begin();
	for (int i = 0; it != imgA.features.end(); it++, i++){
		const vector<float> &descriptors = it->first;
		for (int j = 0; j < DESCRIPTOR_SUM; j++) {
			imgA_Descriptors[i * DESCRIPTOR_SUM + j] = descriptors[j];
		}
	}

	// numData	number of data points.
	// data	pointer to the data.
	// The function builds the KDTree by processing the data data.
	// For efficiency, KDTree does not make a copy the data, but retains a pointer to it.
	// Therefore the data buffer must be valid and unchanged for the lifespan of the object.
	// The number of data points numData must not be smaller than one.
	// 建立KD树
	vl_kdforest_build(forest, imgA.features.size(), imgA_Descriptors);

	vector<ImgPair> res;

	// 利用KD树对特征点进行寻找
	VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
	// VlKDForestNeighbor : Neighbor of a query point.
	VlKDForestNeighbor neighbours[2];

	// 图像B的特征点数据
	for (auto it = imgB.features.begin(); it != imgB.features.end(); it++){
		float temp_data[DESCRIPTOR_SUM];

		for (int i = 0; i < DESCRIPTOR_SUM; i++) {
			temp_data[i] = (it->first)[i];
		}

	// neighbors	list of nearest neighbors found (output).
	// numNeighbors	number of nearest neighbors to find.
	// query	query point.
	// Returns
	// number of tree leaves visited.
	// A neighbor is represented by an instance of the structure VlKDForestNeighbor.
	// Each entry contains the index of the neighbor (this is an index into the KDTree data)
	// and its distance to the query point. Neighbors are sorted by increasing distance.

		vl_kdforestsearcher_query(searcher, neighbours, 2, temp_data);

		float ratio = neighbours[0].distance / neighbours[1].distance;
		// 寻找到的两个点的距离比较远
		if (ratio < RATIO_THRESHOLD) {
			// 对应点上的描述子集合
			vector<float> dstDesciptors(DESCRIPTOR_SUM);
			// 利用A图像的DESCRIPTOR_SUM个描述子来反过来求出A的坐标
			for (int j = 0; j < DESCRIPTOR_SUM; j++) {
				dstDesciptors[j] = imgA_Descriptors[neighbours[0].index * DESCRIPTOR_SUM + j];
			}
			VlSiftKeypoint left = imgA.features.find(dstDesciptors)->second;
			VlSiftKeypoint right = it->second;
			// 获得匹配的对
			res.push_back(ImgPair(left, right));
		}
	}

	vl_kdforestsearcher_delete(searcher);
	vl_kdforest_delete(forest);

	delete[] imgA_Descriptors;
	imgA_Descriptors = NULL;
	return res;
}

int ImageProcess::getMiddleIndex(vector< vector<int> >& nextIndex){
	// 这组图像的边缘
	int edge = 0;
	for (int i = 0; i < nextIndex.size(); i++) {
		if (nextIndex[i].size() == 1) {
			edge = i;
			break;
		}
	}

	// 通过边缘探测求出图像vector的中间图像
	int nextOne = edge;
	vector<int> imgQue;
	imgQue.clear();
	for(int index = 0; index < imgs.size(); index++){
		if(imgQue.empty())
			imgQue.push_back(edge);
		// 遍历判断矩阵
		for(int i = 0; i < imgs.size(); i++){
			// 跳过本图像
			if(nextOne == i) continue;
			bool flag = true;
			// 对于符合条件，判断是否是已经出现的，如果不是push，是的话跳过
			if(stichingMat[nextOne][i]){
				for(int j = 0; j < imgQue.size(); j++){
					if(i == j){
						flag = false;
						break;
					}
				}
				if(!flag) continue;
				if(i != edge)
					imgQue.push_back(i);
				nextOne = i;
				break;
			}
		}
	}
	cout<<imgQue[imgQue.size() / 2] << endl;
	return imgQue[imgQue.size() / 2];
}

Homography ImageProcess::RANSAC(const vector<ImgPair> &pairs){

	srand(666666);
	int k = ceil(log(1 - 0.99) / log(1 - pow(0.5, NUM_OF_PAIR)));

	//最大投票值
	vector<int> maxInlinerInex;

	while (k--) {
		// 随机选取
		vector<ImgPair> randomPairs;
		set<int> randomIndex;

		// 初始化
		for (int i = 0; i < NUM_OF_PAIR; i++) {
			// 随机选择
			int index = rand() % (pairs.size());
			// 在已经选择的序号中不能出现
			while (randomIndex.find(index) != randomIndex.end()){
				index = rand() % (pairs.size());
			}
			randomIndex.insert(index);
			randomPairs.push_back(pairs[index]);
		}
	//		Homography应用：图像对齐，单应性矩阵
	//		上面公式得出的H ，对于图一中的所有点都是正确的，换句话说，可以用H将第一个图中的点映射到第二张图。
	//
	//		如何得到一个Homography
	//		要得到两张图片的H,就必须至少知道4个相同对应位置的点，opencv中可以利用findHomography正确得到
		Homography H = getHomographyMat(randomPairs);

		vector<int> inlinerIndex = getInlinerIndex(pairs, H, randomIndex);
		if (inlinerIndex.size() > maxInlinerInex.size()) {
			maxInlinerInex = inlinerIndex;
		}
	}
	// 求出最大数量的inlier对应的Homography矩阵
	Homography t = getInlinerHomography(pairs, maxInlinerInex);

	return t;

}


Homography ImageProcess::getHomographyMat(const vector<ImgPair> &pair){

	// 求解AH = b
	CImg<double> A(4, NUM_OF_PAIR, 1, 1, 0);
	CImg<double> b(1, NUM_OF_PAIR, 1, 1, 0);

	for (int i = 0; i < NUM_OF_PAIR; i++) {
		A(0, i) = (double)pair[i].src.x;
		A(1, i) = (double)pair[i].src.y;
		A(2, i) = (double)pair[i].src.x * pair[i].src.y;
		A(3, i) = (double)1;
		b(0, i) = (double)pair[i].dst.x;
	}

	CImg<double> x1 = b.get_solve(A);

	for (int i = 0; i < NUM_OF_PAIR; i++) {
		b(0, i) = (double)pair[i].dst.y;
	}

	CImg<double> x2 = b.get_solve(A);

	return Homography(x1(0, 0), x1(0, 1), x1(0, 2), x1(0, 3), x2(0, 0), x2(0, 1), x2(0, 2), x2(0, 3));
}

// 求出原图像到变换后的点
float ImageProcess::getXAfterWarping(float x, float y, Homography& H) {
	return H.H[0][0] * x + H.H[0][1] * y + H.H[0][2] * x * y + H.H[1][0];
}

float ImageProcess::getYAfterWarping(float x, float y, Homography& H) {
	return H.H[1][1] * x + H.H[1][2] * y + H.H[2][0] * x * y + H.H[2][1];
}

vector<int> ImageProcess::getInlinerIndex(const vector<ImgPair> &pairs, Homography& H, set<int> randomIndex){

	// 匹配点
	vector<int> inlinerIndex;
	for (int i = 0; i < pairs.size(); i++) {
		// if (randomIndex.find(i) != randomIndex.end()) {
		// 	continue;
		// }

		float dstX = pairs[i].dst.x;
		float dstY = pairs[i].dst.y;

		// 求出对应的距离
		float x = getXAfterWarping(pairs[i].src.x, pairs[i].src.y, H);
		float y = getYAfterWarping(pairs[i].src.x, pairs[i].src.y, H);

		// l2范式距离
		float distance = sqrt((x - dstX) * (x - dstX) + (y - dstY) * (y - dstY));
		if (distance < RANSAC_THRESHOLD) {
			inlinerIndex.push_back(i);
		}
	}

	return inlinerIndex;
}

// 求出对应的Homography矩阵
Homography ImageProcess::getInlinerHomography(const vector<ImgPair> pairs, vector<int> inliner) {

	// 求解AH = b
	CImg<double> A(4, inliner.size(), 1, 1, 0);
	CImg<double> b(1, inliner.size(), 1, 1, 0);

	for (int i = 0; i < inliner.size(); i++) {
		int cur_index = inliner[i];
		// 对于A
		A(0, i) = (double)pairs[cur_index].src.x;
		A(1, i) = (double)pairs[cur_index].src.y;
		A(2, i) = (double)pairs[cur_index].src.x * pairs[cur_index].src.y;
		A(3, i) = (double)1;
		// 对于B
		b(0, i) = (double)pairs[cur_index].dst.x;
	}

	CImg<double> x1 = b.get_solve(A);

	for (int i = 0; i < inliner.size(); i++) {
		int cur_index = inliner[i];
		// 对于B
		b(0, i) = (double)pairs[cur_index].dst.y;
	}

	CImg<double> x2 = b.get_solve(A);

	return Homography(x1(0, 0), x1(0, 1), x1(0, 2), x1(0, 3), x2(0, 0), x2(0, 1), x2(0, 2), x2(0, 3));

}


float ImageProcess::getMaxXAfterWarping(const CImg<unsigned char> &src, Homography& H) {
	float max_x = getXAfterWarping(0, 0, H);

	if (getXAfterWarping(src.width() - 1, 0, H) > max_x) {
		max_x = getXAfterWarping(src.width() - 1, 0, H);
	}
	if (getXAfterWarping(0, src.height() - 1, H) > max_x) {
		max_x = getXAfterWarping(0, src.height() - 1, H);
	}
	if (getXAfterWarping(src.width() - 1, src.height() - 1, H) > max_x) {
		max_x = getXAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return max_x;
}

float ImageProcess::getMinXAfterWarping(const CImg<unsigned char> &src, Homography& H) {
	float min_x = getXAfterWarping(0, 0, H);

	if (getXAfterWarping(src.width() - 1, 0, H) < min_x) {
		min_x = getXAfterWarping(src.width() - 1, 0, H);
	}
	if (getXAfterWarping(0, src.height() - 1, H) < min_x) {
		min_x = getXAfterWarping(0, src.height() - 1, H);
	}
	if (getXAfterWarping(src.width() - 1, src.height() - 1, H) < min_x) {
		min_x = getXAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return min_x;
}

float ImageProcess::getMaxYAfterWarping(const CImg<unsigned char> &src, Homography& H) {
	float max_y = getYAfterWarping(0, 0, H);

	if (getYAfterWarping(src.width() - 1, 0, H) > max_y) {
		max_y = getYAfterWarping(src.width() - 1, 0, H);
	}
	if (getYAfterWarping(0, src.height() - 1, H) > max_y) {
		max_y = getYAfterWarping(0, src.height() - 1, H);
	}
	if (getYAfterWarping(src.width() - 1, src.height() - 1, H) > max_y) {
		max_y = getYAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return max_y;
}

float ImageProcess::getMinYAfterWarping(const CImg<unsigned char> &src, Homography& H) {
	float min_y = getYAfterWarping(0, 0, H);

	if (getYAfterWarping(src.width() - 1, 0, H) < min_y) {
		min_y = getYAfterWarping(src.width() - 1, 0, H);
	}
	if (getYAfterWarping(0, src.height() - 1, H) < min_y) {
		min_y = getYAfterWarping(0, src.height() - 1, H);
	}
	if (getYAfterWarping(src.width() - 1, src.height() - 1, H) < min_y) {
		min_y = getYAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return min_y;
}

void ImageProcess::warpingImageByHomography(const CImg<unsigned char> &src, CImg<unsigned char> &dst, Homography& H, float offset_x, float offset_y) {
	cimg_forXY(dst, x, y){
		int newX = getXAfterWarping(x + offset_x, y + offset_y, H);
		int newY = getYAfterWarping(x + offset_x, y + offset_y, H);
		if (newX >= 0 && newX < src.width() && newY >= 0 && newY < src.height()) {
			for (int i = 0; i < src.spectrum(); i++) {
				dst(x, y, i) = Projection::bilinearInterpolation(src, newX, newY, i);
			}
		}
	}
}

void ImageProcess::movingImageByOffset(const CImg<unsigned char> &src, CImg<unsigned char> &dst, int offset_x, int offset_y) {
	// 不通过插值填充
	cimg_forXY(dst, x, y){
		int newX = x + offset_x;
		int newY = y + offset_y;
		if (newX >= 0 && newX < src.width() && newY >= 0 && newY < src.height()) {
			for(int i = 0; i < src.spectrum(); i++){
				dst(x, y, i) = src(newX, newY, i);
			}
		}
	}

}

void ImageProcess::updateFeaturesByHomography(map<vector<float>, VlSiftKeypoint> &feature, Homography& H, float offset_x, float offset_y) {
	for (auto iter = feature.begin(); iter != feature.end(); iter++) {
		float cur_x = iter->second.x;
		float cur_y = iter->second.y;
		iter->second.x = getXAfterWarping(cur_x, cur_y, H) - offset_x;
		iter->second.y = getYAfterWarping(cur_x, cur_y, H) - offset_y;
		iter->second.ix = int(iter->second.x);
		iter->second.iy = int(iter->second.y);
	}
}

void ImageProcess::updateFeaturesByOffset(map<vector<float>, VlSiftKeypoint> &feature, int offset_x, int offset_y) {
	for (auto iter = feature.begin(); iter != feature.end(); iter++) {
		iter->second.x -= offset_x;
		iter->second.y -= offset_y;
		iter->second.ix = int(iter->second.x);
		iter->second.iy = int(iter->second.y);
	}
}

bool isEmpty(const CImg<unsigned char> &img, int x, int y) {
	assert(img.spectrum() == 3);

	return (img(x, y, 0) == 0 && img(x, y, 1) == 0 && img(x, y, 2) == 0);
}

CImg<unsigned char> ImageProcess::blendTwoImages(const CImg<unsigned char> &a, const CImg<unsigned char> &b) {
    /* Find overlapped area */
    int w = a.width(), h = a.height(); // a and b has the same size

    int sum_a_x = 0, sum_a_y = 0;
    int width_mid_a = 0; // width of image a's content (calculate in half of height)

    int sum_overlap_x = 0, sum_overlap_y = 0;
    int width_mid_overlap = 0;

    // only consider stitching image horizontally
    int mid_y = h / 2;
    int x = 0;
    while (a(x, mid_y) == 0) ++x; // avoid leading zero
    for (x; x < w; ++x) {
        if (a(x, mid_y) != 0) { // black
            sum_a_x += x;
            ++width_mid_a;
            if (b(x, mid_y) != 0) {
                sum_overlap_x += x;
                ++width_mid_overlap;
            }
        }
    }

    /* 0. Forming a Gaussian Pyramid */
    /* i. Start with the original image G0 */
    int max_len = w >= h ? w : h;
    int level_num = floor(log2(max_len));

    vector<CImg<float>> a_pyramid(level_num);
    vector<CImg<float>> b_pyramid(level_num);
    vector<CImg<float>> mask(level_num);

    a_pyramid[0] = a;
    b_pyramid[0] = b;
    mask[0] = CImg<float>(w, h, 1, 1, 0);

    float ratio = 1.0 * sum_a_x / width_mid_a;
    float overlap_ratio = 1.0 * sum_overlap_x / width_mid_overlap;
    // the x=overlap_ratio line should lie in the overlap area
    if (ratio < overlap_ratio) {
        for (int x = 0; x < overlap_ratio; ++x)
            for (int y = 0; y < h; ++y)
                mask[0](x, y) = 1;
    }
    else {
        for (int x = overlap_ratio + 1; x < w; ++x)
            for (int y = 0; y < h; ++y)
                mask[0](x, y) = 1;
    }


    /* ii. Perform a local Gaussian weighted averaging
    function in a neighborhood about each pixel,
    sampling so that the result is a reduced image
    of half the size in each dimension. */
    for (int i = 1; i < level_num; ++i) {
        int wp = a_pyramid[i - 1].width() / 2;
        int hp = a_pyramid[i - 1].height() / 2;
        int sp = a_pyramid[i - 1].spectrum();
        a_pyramid[i] = a_pyramid[i - 1].get_blur(2, true, true).
                get_resize(wp, hp, 1, sp, 3);
        b_pyramid[i] = b_pyramid[i - 1].get_blur(2, true, true).
                get_resize(wp, hp, 1, sp, 3);
        mask[i] = mask[i - 1].get_blur(2, true, true).
                get_resize(wp, hp, 1, sp, 3);
    } /* iii. Do this all the way up the pyramid Gl = REDUCE(Gl-1) */

    /* iiii. Each level l node will represent a weighted
    average of a subarray of level l. */



    /* 1. Compute Laplacian pyramid of images and mask */
    /* Making the Laplacians Li=Gi-expand(Gi+1)*/
    /*  subtract each level of the pyramid from the next lower one
    EXPAND:  interpolate new samples between those of
    a given image to make it big enough to subtract*/
    for (int i = 0; i < level_num - 1; ++i) {
        int wp = a_pyramid[i].width();
        int hp = a_pyramid[i].height();
        int sp = a_pyramid[i].spectrum();
        a_pyramid[i] -= a_pyramid[i + 1].get_resize(wp, hp, 1, sp, 3);
        b_pyramid[i] -= b_pyramid[i + 1].get_resize(wp, hp, 1, sp, 3);
    }


    /* 2. Create blended image at each level of pyramid */
    /* Forming the New Pyramid
    A third Laplacian pyramid LS is constructed by copying
    nodes from the left half of LA to the corresponding
    nodes of LS and nodes from the right half of LB to the
    right half of LS.
    Nodes along the center line are set equal to
    the average of corresponding LA and LB nodes */
    vector<CImg<float>> blend_pyramid(level_num);
    for (int i = 0; i < level_num; ++i) {
        blend_pyramid[i] = CImg<float>(a_pyramid[i].width(),
                                       a_pyramid[i].height(), 1, a_pyramid[i].spectrum(), 0);
        cimg_forXYC(blend_pyramid[i], x, y, c) {
            blend_pyramid[i](x, y, c) =
                    a_pyramid[i](x, y, c) * mask[i](x, y) +
                    b_pyramid[i](x, y, c) * (1.0 - mask[i](x, y));
        }
    }

    /* 3. Reconstruct complete image */
    /* Using the new Laplacian Pyramid
    Use the new Laplacian pyramid with the reverse of how it
    was created to create a Gaussian pyramid. Gi=Li+expand(Gi+1)
    The lowest level of the new Gaussian pyramid gives the final
    result. */
    // float: cannot be unsigned char(invalid) type!
    CImg<float> expand = blend_pyramid[level_num - 1];
    for (int i = level_num - 2; i >= 0; --i) {
        expand.resize(blend_pyramid[i].width(),
                      blend_pyramid[i].height(), 1, blend_pyramid[i].spectrum(), 3);
        cimg_forXYC(blend_pyramid[i], x, y, c) {
            expand(x, y, c) = blend_pyramid[i](x, y, c) + expand(x, y, c);
            if (expand(x, y, c) > 255) expand(x, y, c) = 255;
            else if (expand(x, y, c) < 0) expand(x, y, c) = 0;
        }
    }
    return expand;
}
