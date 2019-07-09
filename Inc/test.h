#ifndef __TEST_H
#define __TEST_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include <cstdint>

class testClass
{
	public:	
	testClass()
	{
		printf("Default Constructor\n");
	}
	
	~testClass()
	{
		printf("Default Destructor\n");
	}
	
	uint32_t uAdd(uint32_t a, uint32_t b)
	{
		return a+b;
	}
	
	uint32_t uMul(uint32_t a, uint32_t b)
	{
		return a*b;
	}
	
	private:
	uint32_t m_A;
};

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */
