#ifndef POWER_GLOBALS_H
#define POWER_GLOBALS_H

#define BUFFER_SIZE     500
#define PI    3.141592653589793238462643383279
#define POWER_EMPTY   9999999


#include <vector>
#include <string>

enum bus_type_e {PQ=1, PV=2, REF=3, NONE=4, AGC=5};

int read_line(const std::string&, std::vector<double>&);
int read_line(const char*, std::vector<double>&);

#endif
