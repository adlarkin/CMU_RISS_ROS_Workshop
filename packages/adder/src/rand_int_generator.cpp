#include <adder/rand_int_generator.h>

int make_rand_int(const int & min, const int & max)
{
  return rand() % (max + 1) + min;
}