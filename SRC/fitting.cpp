#include "fitting.h"
inline bool less_y(const CvPoint2D32f& p1, const CvPoint2D32f& p2) { return p1.y < p2.y; }
inline bool less_x(const CvPoint2D32f& p1, const CvPoint2D32f& p2) { return p1.x < p2.x; }
bool fitPoints(vector<float>& inx, vector<float>& iny, vector<float>& fitInx, vector<float>& fitIny, int nPoints, int order = 3)
{
	fitInx.clear();
	fitIny.clear();

	int length = (int)inx.size();

	if (length != iny.size())
		return false;

	if (length < nPoints)
		return false;

	vector<float> tinx = inx;
	vector<float> tiny = iny;
	sort(tiny.begin(), tiny.end());

	float step = (tiny[length - 1] - tiny[0]) / (LENGTHOFLANE - 1);

	double* c = new double[order + 1];
	double xi, yi, chisq;
	gsl_matrix *X, *cov;
	gsl_vector *y, *w, *cc;

	X = gsl_matrix_alloc(length, order + 1);
	y = gsl_vector_alloc(length);
	w = gsl_vector_alloc(length);

	cc = gsl_vector_alloc(order + 1);
	cov = gsl_matrix_alloc(order + 1, order + 1);

	for (int i = 0; i < length; i++)
	{
		xi = iny[i];
		yi = inx[i];
		for (int j = 0; j < order + 1; j++)
			gsl_matrix_set(X, i, j, pow(xi, j));
		gsl_vector_set(y, i, yi);
		gsl_vector_set(w, i, 1.0);
	}

	gsl_multifit_linear_workspace* work = gsl_multifit_linear_alloc(length, order + 1);
	gsl_multifit_wlinear(X, w, y, cc, cov, &chisq, work);
	gsl_multifit_linear_free(work);

	for (int i = 0; i < order + 1; i++)
		c[i] = gsl_vector_get(cc, i);

	float pystart = tiny[0];

	for (int index = 0; index < LENGTHOFLANE; index++)
	{
		fitIny.push_back(pystart);
		fitInx.push_back((float)gsl_poly_eval(c, order + 1, pystart));
		pystart += step;
	}

	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(w);
	gsl_vector_free(cc);
	delete[] c;
	c = NULL;
	return true;
}

bool fitPoints2(vector<float>& inx, vector<float>& iny, vector<float>& fitInx, vector<float>& fitIny, int nPoints, int order = 3)
{
	fitInx.clear();
	fitIny.clear();

	int length = (int)inx.size();

	if (length != iny.size())
		return false;

	vector<float> tinx = inx;
	vector<float> tiny = iny;
	sort(tiny.begin(), tiny.end());

	float step = (tiny[length - 1] - tiny[0]) / (2*nPoints - 1);

	double* c = new double[order + 1];
	double xi, yi, chisq;
	gsl_matrix *X, *cov;
	gsl_vector *y, *w, *cc;

	X = gsl_matrix_alloc(length, order + 1);
	y = gsl_vector_alloc(length);
	w = gsl_vector_alloc(length);

	cc = gsl_vector_alloc(order + 1);
	cov = gsl_matrix_alloc(order + 1, order + 1);

	for (int i = 0; i < length; i++)
	{
		xi = iny[i];
		yi = inx[i];
		for (int j = 0; j < order + 1; j++)
			gsl_matrix_set(X, i, j, pow(xi, j));
		gsl_vector_set(y, i, yi);
		gsl_vector_set(w, i, 1.0);
	}

	gsl_multifit_linear_workspace* work = gsl_multifit_linear_alloc(length, order + 1);
	gsl_multifit_wlinear(X, w, y, cc, cov, &chisq, work);
	gsl_multifit_linear_free(work);

	for (int i = 0; i < order + 1; i++)
		c[i] = gsl_vector_get(cc, i);

	float pystart = tiny[0];

	for (int index = 0; index < 2 * nPoints; index++)
	{
		fitIny.push_back(pystart);
		fitInx.push_back((float)gsl_poly_eval(c, order + 1, pystart));
		pystart += step;
	}

	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(w);
	gsl_vector_free(cc);
	delete[] c;
	c = NULL;
	return true;
}

void fitPoints(vector<float>& inx, vector<float>& iny, double* c, double& chisq, int length, int order = 3)
{
	double xi, yi;
	gsl_matrix *X, *cov;
	gsl_vector *y, *w, *cc;

	X = gsl_matrix_alloc(length, order + 1);
	y = gsl_vector_alloc(length);
	w = gsl_vector_alloc(length);

	cc = gsl_vector_alloc(order + 1);
	cov = gsl_matrix_alloc(order + 1, order + 1);

	for (int i = 0; i < length; i++)
	{
		xi = inx[i];
		yi = iny[i];
		for (int j = 0; j < order + 1; j++)
			gsl_matrix_set(X, i, j, pow(xi, j));
		gsl_vector_set(y, i, yi);
		gsl_vector_set(w, i, 1.0);
	}

	gsl_multifit_linear_workspace* work = gsl_multifit_linear_alloc(length, order + 1);
	gsl_multifit_wlinear(X, w, y, cc, cov, &chisq, work);
	gsl_multifit_linear_free(work);

	for (int i = 0; i < order + 1; i++)
		c[i] = gsl_vector_get(cc, i);

	gsl_matrix_free(X);
	gsl_matrix_free(cov);
	gsl_vector_free(y);
	gsl_vector_free(w);
	gsl_vector_free(cc);
}

bool ransacFitPoints(vector<float>& inx, vector<float>& iny, vector<float>& fitInx, vector<float>& fitIny, int nPoints, int order = 3)
{
	int length = (int)inx.size();

	if (length != iny.size())
		return false;

	if (length < nPoints*0.8)
		return false;

	const int indexNum = 20;

	if (length < indexNum)
		return false;

	vector<CvPoint2D32f> pt;
	for (int i = 0; i < length; i++)
		pt.push_back(cvPoint2D32f(inx[i], iny[i]));
	sort(pt.begin(), pt.end(), less_y);
	int maxiterater = 10;
	int maxGoodPointsNum = (int)(length*0.8f);
	int bestGoodPointsNum = 500;
	double min_err = 999;
	float point_err = 0.5;
	float max_err = 0.5;
	float break_err = 0.0009f;
	int index[indexNum] = { 0 };
	int iter = 0;
	double* maybe_c = new double[order + 1];
	double* c = new double[order + 1];
	bool issuccessful = false;
	while (iter < maxiterater)
	{
		vector<float> tinx;
		vector<float> tiny;
		tinx.push_back(pt[0].x);
		tinx.push_back(pt[pt.size() - 1].x);
		tiny.push_back(pt[0].y);
		tiny.push_back(pt[pt.size() - 1].y);
		double this_err = 999;
		srand((unsigned)time(0));
		for (int i = 0; i < (indexNum - 2); i++)
		{
			index[i] = rand() % length;
			for (int j = 0; j < i; j++)
			{
				if (index[i] == index[j])
				{
					index[i] = rand() % length;
					j = -1;
				}
			}
		}
		for (int i = 0; i < indexNum; i++)
		{
			tinx.push_back(inx[index[i]]);
			tiny.push_back(iny[index[i]]);
		}
		fitPoints(tiny, tinx, maybe_c, this_err, (int)(tinx.size()), order);

		tinx.clear();
		tiny.clear();
		vector<float>::iterator ity, itx;
		int goodPointsNum = 0;
		for (ity = iny.begin(), itx = inx.begin(); ity != iny.end(); ++ity, ++itx)
		{
			float pointerr = abs(*itx - (float)gsl_poly_eval(maybe_c, order + 1, *ity));
			if (/*pointerr && */pointerr < point_err)
			{
				goodPointsNum++;
				tinx.push_back(*itx);
				tiny.push_back(*ity);
			}
		}
		if (goodPointsNum > maxGoodPointsNum)
		{
			fitPoints(tiny, tinx, maybe_c, this_err, (int)(tinx.size()), order);
			if (this_err <= break_err || goodPointsNum >= bestGoodPointsNum)
			{
				memcpy(c, maybe_c, sizeof(double)*(order + 1));
				min_err = this_err;
				issuccessful = true;
				break;
			}
			else if (this_err < min_err && this_err <= max_err)
			{
				memcpy(c, maybe_c, sizeof(double)*(order + 1));
				min_err = this_err;
				issuccessful = true;
			}
		}
		iter++;
	}
	if (!issuccessful)
	{
		delete[] maybe_c;
		delete[] c;
		return false;
	}

	fitInx.clear();
	fitIny.clear();
	vector<float> sort_inx = inx;
	vector<float> sort_iny = iny;
	sort(sort_iny.begin(), sort_iny.end());
	float pystart = sort_iny[0];
	float step = (sort_iny[length - 1] - sort_iny[0]) / (LENGTHOFLANE - 1);
	for (int i = 0; i < nPoints; i++)
	{
		fitIny.push_back(pystart);
		fitInx.push_back((float)gsl_poly_eval(c, order + 1, pystart));
		pystart += step;
	}

	delete[] maybe_c;
	delete[] c;
	return true;
}



