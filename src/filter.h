#ifndef __FILTER_H__
#define __FILTER_H__

enum type_t {
	FILTER_HPF,
	FILTER_LPF
};

class filter
{
private:
	enum type_t type;
	double time_const;
	double x;
	double k[4];
public:
	filter( enum type_t type, double tc, double out0 );
	double in( double i );
};

#endif

