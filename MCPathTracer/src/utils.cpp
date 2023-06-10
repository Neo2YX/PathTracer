#include "utils.h"

//检查是否存在nan inf值
#ifdef __INTEL_COMPILER
#include <mathimf.h>
#endif

int isnan_local(double x) {
#ifdef __INTEL_COMPILER
	return isnan(x);
#else
	return std::isnan(x);
#endif
}

int isinf_local(double x) {
#ifdef __INTEL_COMPILER
	return isinf(x);
#else
	return std::isinf(x);
#endif
}


int myChk(float a) {
	if (isnan_local(a))
		__debugbreak();
	//if (isinf_local(a))
		//__debugbreak();
	return 0;
}